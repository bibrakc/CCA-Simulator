#lang racket/base

;; BSD 3-Clause License
;;
;; Copyright (c) 2026, Bibrak Qamar
;;
;; Redistribution and use in source and binary forms, with or without
;; modification, are permitted provided that the following conditions are met:
;;
;; 1. Redistributions of source code must retain the above copyright notice, this
;;    list of conditions and the following disclaimer.
;;
;; 2. Redistributions in binary form must reproduce the above copyright notice,
;;    this list of conditions and the following disclaimer in the documentation
;;    and/or other materials provided with the distribution.
;;
;; 3. Neither the name of the copyright holder nor the names of its
;;    contributors may be used to endorse or promote products derived from
;;    this software without specific prior written permission.
;;
;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
;; DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
;; FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
;; DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
;; SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
;; CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
;; OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
;; OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.



(require racket/match
         racket/list
         racket/hash
         "../ast.rkt")

(provide parse-pass)

;; ═══════════════════════════════════════════════════════════════════════════════
;; Parser pass — transforms S-expression datums into typed AST nodes.
;;
;; This is the second pass in the pipeline (after read-source). It pattern-matches
;; each top-level S-expression form and constructs the corresponding AST struct.
;;
;; Recognized top-level forms:
;;   (define-constant name : Type value)
;;   (define-vertex Name [field : Type ...] ...)
;;   (define-action Name (params ...) body)
;;   (define-application Name #:keyword value ...)
;;
;; Action body parsing supports TWO syntaxes:
;;   1. Canonical:  (predicate EXPR (work STMT...) (diffuse (predicate EXPR) STMT...))
;;   2. Paper-compatible sugar: (predicate EXPR (begin STMT... (diffuse ...)))
;;      where the last form in `begin` is the diffuse block.
;;
;; Inputs:  List of S-expression datums from read-source-pass.
;; Outputs: A `cca-program` AST struct.
;; ═══════════════════════════════════════════════════════════════════════════════

;; ─── Parser pass ──────────────────────────────────────────────────────────────
;; Input: list of S-expression datums (from read-source)
;; Output: cca-program struct
;;
;; Recognizes: define-constant, define-vertex, define-action, define-application

(define (parse-pass forms)
  (define constants '())
  (define vertices '())
  (define actions '())
  (define applications '())

  (for ([form (in-list forms)])
    (match form
      [`(define-constant ,name : ,type ,value)
       (set! constants
             (cons (constant-decl name (parse-type type) value no-span) constants))]

      [`(define-vertex ,name ,fields ...)
       (set! vertices
             (cons (vertex-decl name (map parse-field fields) no-span) vertices))]

      [`(define-action ,name (,params ...) ,body)
       (define parsed-params (map parse-param params))
       (define target (car parsed-params))
       (define payload-params (cdr parsed-params))
       (define-values (pred work dpred diffuse) (parse-action-body body))
       (set! actions
             (cons (action-decl name target payload-params pred work dpred diffuse no-span)
                   actions))]

      [`(define-program ,binary-name ,host-forms ...)
       ;; Driver program with host-level forms
       (set! applications
             (cons (parse-program-decl binary-name host-forms) applications))]

      [other
       (error 'parse "unrecognized top-level form: ~s" other)]))

  ;; Validate cardinality
  (when (null? vertices)
    (error 'parse "program must contain at least one define-vertex"))
  (when (> (length vertices) 1)
    (error 'parse "Draft 0.1 allows exactly one define-vertex"))
  (when (null? actions)
    (error 'parse "program must contain at least one define-action"))
  (when (null? applications)
    (error 'parse "program must contain exactly one define-program"))
  (when (> (length applications) 1)
    (error 'parse "Draft 0.1 allows exactly one define-program"))

  (cca-program (reverse constants)
               (car (reverse vertices))
               (reverse actions)
               (car (reverse applications))
               no-span))

;; ─── Type parsing ─────────────────────────────────────────────────────────────
(define (parse-type type-sexp)
  (match type-sexp
    ['Unit (t-unit)]
    ['Boolean (t-bool)]
    ['UInt32 (t-u32)]
    ['Integer (t-u32)]  ; paper-compatibility alias
    ['Address (t-address)]
    ['Edge (t-edge)]
    [`(Pointer ,v) (t-pointer v)]
    [`(Vector ,e) (t-vector (parse-type e))]
    [other (error 'parse-type "unknown type: ~s" other)]))

;; ─── Field parsing ────────────────────────────────────────────────────────────
(define (parse-field field-sexp)
  (match field-sexp
    [`[,name : ,type #:initial ,init #:mutable]
     (field-decl name (parse-type type) init #t '() no-span)]
    [`[,name : ,type #:initial ,init]
     (field-decl name (parse-type type) init #f '() no-span)]
    [`[,name : ,type #:mutable]
     (field-decl name (parse-type type) #f #t '() no-span)]
    [`[,name : ,type]
     (field-decl name (parse-type type) #f #f '() no-span)]
    [other (error 'parse-field "invalid field definition: ~s" other)]))

;; ─── Parameter parsing ────────────────────────────────────────────────────────
(define (parse-param param-sexp)
  (match param-sexp
    [`[,name : ,type]
     (param-decl name (parse-type type) no-span)]
    [other (error 'parse-param "invalid parameter: ~s" other)]))

;; ─── Action body parsing ──────────────────────────────────────────────────────
;; The action body must decompose into four semantic phases:
;;   1. predicate       — Boolean expression guarding activation
;;   2. work            — statements that mutate the vertex
;;   3. diffuse-predicate — Boolean expression guarding propagation
;;   4. diffuse         — statements that propagate actions to neighbors
;;
;; Two concrete syntax forms are supported:
;;
;; Canonical form (explicit phases):
;;   (predicate EXPR (work STMT ...) (diffuse (predicate EXPR) STMT ...))
;;
;; Paper-compatible sugar (inline begin block):
;;   (predicate EXPR (begin STMT ... (diffuse (predicate EXPR) STMT ...)))
;;   The last form in `begin` must be the `(diffuse ...)` block; everything
;;   before it is the work phase.

(define (parse-action-body body)
  (match body
    ;; Canonical: all four phases spelled out explicitly
    [`(predicate ,pred-expr
        (work ,work-stmts ...)
        (diffuse (predicate ,dpred-expr) ,diffuse-stmts ...))
     (values (parse-expr pred-expr)
             (map parse-stmt work-stmts)
             (parse-expr dpred-expr)
             (map parse-stmt diffuse-stmts))]

    ;; Paper sugar: work and diffuse merged in a begin block.
    ;; We split on the last form, which must be `(diffuse ...)`.
    [`(predicate ,pred-expr
        (begin ,body-forms ...))
     (define-values (work-forms diffuse-form) (split-begin-body body-forms))
     (match diffuse-form
       ;; Diffuse with its own predicate
       [`(diffuse (predicate ,dpred-expr ,diffuse-stmts ...))
        (values (parse-expr pred-expr)
                (map parse-stmt work-forms)
                (parse-expr dpred-expr)
                (map parse-stmt diffuse-stmts))]
       ;; Diffuse without a predicate — reuse the outer predicate
       [`(diffuse ,diffuse-stmts ...)
        (values (parse-expr pred-expr)
                (map parse-stmt work-forms)
                (parse-expr pred-expr)
                (map parse-stmt diffuse-stmts))]
       [other (error 'parse-action-body
                     "expected (diffuse ...) as last form in begin, got: ~s" other)])]

    [other
     (error 'parse-action-body "unrecognized action body shape: ~s" other)]))

;; Splits a begin body into (work-stmts, final-diffuse-form).
;; The last form in the list must be the diffuse block.
(define (split-begin-body forms)
  (cond
    [(null? forms)
     (error 'parse-action-body "empty begin body")]
    [(null? (cdr forms))
     ;; Only one form — it must be the diffuse
     (values '() (car forms))]
    [else
     ;; Recurse: everything except the last form is work
     (define-values (rest-work diffuse) (split-begin-body (cdr forms)))
     (values (cons (car forms) rest-work) diffuse)]))

;; ─── Expression parsing ───────────────────────────────────────────────────────
;; Maps S-expressions to expression AST nodes. Dispatch order matters:
;; literals and symbols are checked first, then compound forms by leading symbol.
(define (parse-expr sexp)
  (match sexp
    [(? exact-nonnegative-integer?) (literal-expr no-span sexp (t-u32))]
    [(? boolean?) (literal-expr no-span sexp (t-bool))]
    [(? symbol?) (var-expr no-span sexp)]
    ;; Primitive operations: arithmetic, comparison, logic
    [`(,(and op (or '+ '- '* '> '>= '< '<= '= 'eq? 'and 'or 'not)) ,args ...)
     (prim-expr no-span (normalize-op op) (map parse-expr args))]
    ;; vertex-id is a special pseudo-field returning the vertex's network address
    [`(vertex-id ,target) (vertex-read-expr no-span 'id (parse-expr target))]
    ;; vertex-FIELD accessor pattern: vertex-level, vertex-distance, etc.
    [`(,(and accessor (? vertex-field-accessor?)) ,target)
     (vertex-read-expr no-span (extract-field-name accessor) (parse-expr target))]
    ;; Edge accessors: destination address and edge weight
    [`(edge-address ,target) (edge-read-expr no-span 'address (parse-expr target))]
    [`(edge-weight ,target) (edge-read-expr no-span 'weight (parse-expr target))]
    [`(let (,bindings ...) ,body)
     (let-expr no-span
               (map parse-binding bindings)
               (parse-expr body))]
    [`(if ,test ,then ,else-b)
     (if-expr no-span (parse-expr test) (parse-expr then) (parse-expr else-b))]
    [other (error 'parse-expr "unrecognized expression: ~s" other)]))

;; Normalize Scheme-style operator names to canonical symbols
(define (normalize-op op)
  (case op
    [(eq?) '=]  ; eq? is an alias for structural equality in CCA
    [else op]))

;; Detects symbols like vertex-level, vertex-distance (but NOT vertex-id or vertex-edges)
(define (vertex-field-accessor? sym)
  (and (symbol? sym)
       (let ([s (symbol->string sym)])
         (and (>= (string-length s) 8)
              (string=? (substring s 0 7) "vertex-")
              (not (string=? s "vertex-id"))
              (not (string=? s "vertex-edges"))))))

;; Strips the "vertex-" prefix to get the field name symbol
(define (extract-field-name accessor)
  (string->symbol (substring (symbol->string accessor) 7)))

;; ─── Statement parsing ────────────────────────────────────────────────────────
;; Statements appear in work and diffuse phases. Two mutation syntaxes are
;; supported: (set-vertex-FIELD! target value) and (set! (vertex-FIELD target) value).
(define (parse-stmt sexp)
  (match sexp
    ;; Mutation via set-vertex-FIELD! accessor (e.g., set-vertex-level!)
    [`(,(and setter (? set-field-accessor?)) ,target ,value)
     (set-field-stmt no-span (extract-set-field-name setter) (parse-expr target) (parse-expr value))]
    ;; Mutation via set! form — alternative syntax
    [`(set! (,(and accessor (? vertex-field-accessor?)) ,target) ,value)
     (set-field-stmt no-span (extract-field-name accessor)
                     (parse-expr target)
                     (parse-expr value))]
    ;; Edge iteration with typed edge variable
    [`(for-each ([,edge-var : Edge] (vertex-edges ,target)) ,body-stmts ...)
     (for-edges-stmt no-span edge-var (parse-expr target) (map parse-stmt body-stmts))]
    ;; Edge iteration with any type annotation (flexibility for future edge types)
    [`(for-each ([,edge-var : ,_type] (vertex-edges ,target)) ,body-stmts ...)
     (for-edges-stmt no-span edge-var (parse-expr target) (map parse-stmt body-stmts))]
    ;; Propagate: send an action to a destination with payload arguments
    [`(propagate ,action-name ,dest ,args ...)
     (propagate-stmt no-span action-name (parse-expr dest) (map parse-expr args))]
    [`(let (,bindings ...) ,body-stmts ...)
     (let-stmt no-span (map parse-binding bindings) (map parse-stmt body-stmts))]
    [`(if ,test (begin ,then-stmts ...) (begin ,else-stmts ...))
     (if-stmt no-span (parse-expr test) (map parse-stmt then-stmts) (map parse-stmt else-stmts))]
    [`(begin ,stmts ...)
     (begin-stmt no-span (map parse-stmt stmts))]
    [other (error 'parse-stmt "unrecognized statement: ~s" other)]))

;; Detects symbols like set-vertex-level! (prefix "set-vertex-", suffix "!")
(define (set-field-accessor? sym)
  (and (symbol? sym)
       (let ([s (symbol->string sym)])
         (and (>= (string-length s) 13)
              (string=? (substring s 0 11) "set-vertex-")
              (string=? (substring s (- (string-length s) 1)) "!")))))

;; Extracts field name from set-vertex-FIELD! → FIELD
(define (extract-set-field-name setter)
  (define s (symbol->string setter))
  ;; set-vertex-FIELD! → FIELD
  (string->symbol (substring s 11 (- (string-length s) 1))))

;; ─── Binding parsing ──────────────────────────────────────────────────────────
(define (parse-binding b)
  (match b
    [`[,name ,expr-sexp]
     (cons name (parse-expr expr-sexp))]
    [other (error 'parse-binding "invalid binding: ~s" other)]))

;; ─── Application parsing ──────────────────────────────────────────────────────
;; ─── define-program parsing ────────────────────────────────────────────────────
;; Parses host-level forms and synthesizes an application-decl that the emitter
;; can use. The host forms are stored in the application-decl's 'verification'
;; field (repurposed) as a list of host-form structs when the new style is used.
;;
;; For backward compatibility, when the emitter sees a list in the verification
;; field, it knows to generate main() from host forms rather than the old template.

(define (parse-program-decl binary-name host-forms)
  (define parsed-host-forms (map parse-host-form host-forms))

  ;; Flatten host-let bodies to find nested host forms for metadata extraction
  (define (flatten-host-forms forms)
    (apply append
           (for/list ([hf (in-list forms)])
             (match hf
               [(host-let _ body) (cons hf (flatten-host-forms body))]
               [_ (list hf)]))))

  (define all-forms (flatten-host-forms parsed-host-forms))

  ;; Extract key metadata from host forms for the application-decl
  (define vertex-type
    (for/or ([hf (in-list all-forms)])
      (and (host-load-graph? hf)
           (hash-ref (host-load-graph-options hf) 'vertex-type #f))))
  (define root-action
    (for/or ([hf (in-list all-forms)])
      (and (host-germinate? hf)
           (host-germinate-action-name hf))))
  (define root-arguments
    (for/or ([hf (in-list all-forms)])
      (and (host-germinate? hf)
           (hash-ref (host-germinate-options hf) 'arguments '()))))
  (define result-field
    (for/or ([hf (in-list all-forms)])
      (and (host-when-verify? hf)
           (hash-ref (host-when-verify-verify-options hf) 'field #f))))
  (define verification-ext
    (for/or ([hf (in-list all-forms)])
      (and (host-when-verify? hf)
           (hash-ref (host-when-verify-verify-options hf) 'extension #f))))

  (application-decl
   (string->symbol binary-name)
   binary-name
   (or vertex-type 'Unknown)
   (or root-action 'unknown)
   (or root-arguments '())
   result-field
   ;; Store host forms here — emitter detects list vs string to choose generation mode
   parsed-host-forms
   no-span))

(define (parse-host-form form)
  (match form
    ;; let binding: binds CLI args/flags to names, body is a host operation
    [`(let (,bindings ...) ,body-forms ...)
     ;; Parse each binding — the value should be a cli-arg or cli-flag
     (define parsed-bindings
       (for/list ([b (in-list bindings)])
         (match b
           [`[,name ,val] (cons name (parse-host-value val))]
           [_ (error 'parse-program "invalid let binding: ~s" b)])))
     ;; Parse the body — should be one or more host forms
     ;; The bindings' cli-arg/cli-flag values need to be collected for CLI setup
     ;; but the actual host form uses the bound variable names
     (define body-host-forms (map parse-host-form body-forms))
     ;; Attach the CLI bindings to the first host form so emitter can collect them
     ;; We wrap this in a special host-let struct
     (host-let parsed-bindings body-host-forms)]

    [`(create-simulator ,options ...)
     (host-create-simulator (parse-host-options options))]

    [`(load-graph ,options ...)
     (host-load-graph (parse-host-options options))]

    [`(register-actions ,action-names ...)
     (host-register-actions action-names)]

    [`(germinate ,action-name ,options ...)
     (host-germinate action-name (parse-host-options options))]

    [`(run)
     (host-run)]

    [`(write-results ,options ...)
     (host-write-results (parse-host-options options))]

    [`(when ,flag-ref (verify ,options ...))
     ;; flag-ref is a symbol (let-bound variable name) referencing a cli-flag
     ;; Store the variable name so the emitter generates if(<mangled-name>)
     (define condition-var
       (match flag-ref
         [(? symbol?) flag-ref]
         [`(cli-flag ,name ,opts ...)  (string->symbol name)]))
     (host-when-verify (hash-set (parse-host-options options)
                                 'condition-var condition-var))]

    [other
     (error 'parse-program "unrecognized host-level form: ~s" other)]))

;; Parse keyword options in host forms, recognizing cli-arg and cli-flag values
(define (parse-host-options opts)
  (let loop ([remaining opts] [h (hash)])
    (cond
      [(null? remaining) h]
      [(keyword? (car remaining))
       (when (null? (cdr remaining))
         (error 'parse-host-options "keyword ~a has no value" (car remaining)))
       (define key (string->symbol (keyword->string (car remaining))))
       (define val (parse-host-value (cadr remaining)))
       (loop (cddr remaining) (hash-set h key val))]
      [else
       (error 'parse-host-options "expected keyword, got: ~s" (car remaining))])))

;; Parse a value in a host option — could be a literal, symbol, or cli-arg/cli-flag
(define (parse-host-value val)
  (match val
    [(? string?) val]
    [(? number?) val]
    [(? symbol?) val]
    [(? boolean?) val]
    [`(cli-arg ,name ,options ...)
     (define opts (parse-host-options options))
     (cli-arg-ref name
                  (hash-ref opts 'long-name name)
                  (hash-ref opts 'type 'String)
                  (hash-ref opts 'default #f)
                  (hash-ref opts 'required #f)
                  (hash-ref opts 'description name))]
    [`(cli-flag ,name ,options ...)
     (define opts (parse-host-options options))
     (cli-flag-ref name
                   (hash-ref opts 'long-name name)
                   (hash-ref opts 'description name))]
    ;; A list of literals (e.g., root-arguments (0))
    [`(,vals ...)
     vals]
    [_ val]))

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

      [`(define-application ,name ,options ...)
       (set! applications
             (cons (parse-application name options) applications))]

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
    (error 'parse "program must contain exactly one define-application"))
  (when (> (length applications) 1)
    (error 'parse "Draft 0.1 allows exactly one define-application"))

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
;; Canonical form:
;;   (predicate EXPR (work STMT ...) (diffuse (predicate EXPR) STMT ...))
;;
;; Paper-compatible sugar:
;;   (predicate EXPR (begin STMT ... (diffuse (predicate EXPR STMT ...))))

(define (parse-action-body body)
  (match body
    ;; Canonical: (predicate EXPR (work ...) (diffuse (predicate EXPR) ...))
    [`(predicate ,pred-expr
        (work ,work-stmts ...)
        (diffuse (predicate ,dpred-expr) ,diffuse-stmts ...))
     (values (parse-expr pred-expr)
             (map parse-stmt work-stmts)
             (parse-expr dpred-expr)
             (map parse-stmt diffuse-stmts))]

    ;; Paper sugar: (predicate EXPR (begin WORK... (diffuse (predicate DPRED DIFFUSE...))))
    [`(predicate ,pred-expr
        (begin ,body-forms ...))
     (define-values (work-forms diffuse-form) (split-begin-body body-forms))
     (match diffuse-form
       [`(diffuse (predicate ,dpred-expr ,diffuse-stmts ...))
        (values (parse-expr pred-expr)
                (map parse-stmt work-forms)
                (parse-expr dpred-expr)
                (map parse-stmt diffuse-stmts))]
       [`(diffuse ,diffuse-stmts ...)
        ;; No diffuse predicate; use same as outer predicate
        (values (parse-expr pred-expr)
                (map parse-stmt work-forms)
                (parse-expr pred-expr)
                (map parse-stmt diffuse-stmts))]
       [other (error 'parse-action-body
                     "expected (diffuse ...) as last form in begin, got: ~s" other)])]

    [other
     (error 'parse-action-body "unrecognized action body shape: ~s" other)]))

;; Split a begin body into work statements and a final diffuse form
(define (split-begin-body forms)
  (cond
    [(null? forms)
     (error 'parse-action-body "empty begin body")]
    [(null? (cdr forms))
     ;; Last form must be diffuse
     (values '() (car forms))]
    [else
     (define-values (rest-work diffuse) (split-begin-body (cdr forms)))
     (values (cons (car forms) rest-work) diffuse)]))

;; ─── Expression parsing ───────────────────────────────────────────────────────
(define (parse-expr sexp)
  (match sexp
    [(? exact-nonnegative-integer?) (literal-expr no-span sexp (t-u32))]
    [(? boolean?) (literal-expr no-span sexp (t-bool))]
    [(? symbol?) (var-expr no-span sexp)]
    [`(,(and op (or '+ '- '* '> '>= '< '<= '= 'eq? 'and 'or 'not)) ,args ...)
     (prim-expr no-span (normalize-op op) (map parse-expr args))]
    [`(vertex-id ,target) (vertex-read-expr no-span 'id (parse-expr target))]
    [`(,(and accessor (? vertex-field-accessor?)) ,target)
     (vertex-read-expr no-span (extract-field-name accessor) (parse-expr target))]
    [`(edge-address ,target) (edge-read-expr no-span 'address (parse-expr target))]
    [`(edge-weight ,target) (edge-read-expr no-span 'weight (parse-expr target))]
    [`(let (,bindings ...) ,body)
     (let-expr no-span
               (map parse-binding bindings)
               (parse-expr body))]
    [`(if ,test ,then ,else-b)
     (if-expr no-span (parse-expr test) (parse-expr then) (parse-expr else-b))]
    [other (error 'parse-expr "unrecognized expression: ~s" other)]))

(define (normalize-op op)
  (case op
    [(eq?) '=]
    [else op]))

(define (vertex-field-accessor? sym)
  (and (symbol? sym)
       (let ([s (symbol->string sym)])
         (and (>= (string-length s) 8)
              (string=? (substring s 0 7) "vertex-")
              (not (string=? s "vertex-id"))
              (not (string=? s "vertex-edges"))))))

(define (extract-field-name accessor)
  (string->symbol (substring (symbol->string accessor) 7)))

;; ─── Statement parsing ────────────────────────────────────────────────────────
(define (parse-stmt sexp)
  (match sexp
    [`(,(and setter (? set-field-accessor?)) ,target ,value)
     (set-field-stmt no-span (extract-set-field-name setter) (parse-expr target) (parse-expr value))]
    [`(set! (,(and accessor (? vertex-field-accessor?)) ,target) ,value)
     (set-field-stmt no-span (extract-field-name accessor)
                     (parse-expr target)
                     (parse-expr value))]
    [`(for-each ([,edge-var : Edge] (vertex-edges ,target)) ,body-stmts ...)
     (for-edges-stmt no-span edge-var (parse-expr target) (map parse-stmt body-stmts))]
    [`(for-each ([,edge-var : ,_type] (vertex-edges ,target)) ,body-stmts ...)
     (for-edges-stmt no-span edge-var (parse-expr target) (map parse-stmt body-stmts))]
    [`(propagate ,action-name ,dest ,args ...)
     (propagate-stmt no-span action-name (parse-expr dest) (map parse-expr args))]
    [`(let (,bindings ...) ,body-stmts ...)
     (let-stmt no-span (map parse-binding bindings) (map parse-stmt body-stmts))]
    [`(if ,test (begin ,then-stmts ...) (begin ,else-stmts ...))
     (if-stmt no-span (parse-expr test) (map parse-stmt then-stmts) (map parse-stmt else-stmts))]
    [`(begin ,stmts ...)
     (begin-stmt no-span (map parse-stmt stmts))]
    [other (error 'parse-stmt "unrecognized statement: ~s" other)]))

(define (set-field-accessor? sym)
  (and (symbol? sym)
       (let ([s (symbol->string sym)])
         (and (>= (string-length s) 13)
              (string=? (substring s 0 11) "set-vertex-")
              (string=? (substring s (- (string-length s) 1)) "!")))))

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
(define (parse-application name options)
  (define (get-option key opts [default #f])
    (cond
      [(null? opts) default]
      [(and (keyword? (car opts)) (equal? (car opts) key))
       (if (null? (cdr opts))
           (error 'parse-application "missing value for ~a" key)
           (cadr opts))]
      [else (get-option key (cdr opts) default)]))

  ;; Parse keyword options manually
  (define opts-hash (parse-keyword-options options))

  (application-decl
   name
   (hash-ref opts-hash '#:binary-name
             (λ () (error 'parse-application "missing #:binary-name")))
   (hash-ref opts-hash '#:vertex-type
             (λ () (error 'parse-application "missing #:vertex-type")))
   (hash-ref opts-hash '#:root-action
             (λ () (error 'parse-application "missing #:root-action")))
   (hash-ref opts-hash '#:root-arguments '())
   (hash-ref opts-hash '#:result-field #f)
   (hash-ref opts-hash '#:verification #f)
   no-span))

(define (parse-keyword-options opts)
  (let loop ([remaining opts] [h (hash)])
    (cond
      [(null? remaining) h]
      [(keyword? (car remaining))
       (when (null? (cdr remaining))
         (error 'parse-application "keyword ~a has no value" (car remaining)))
       (loop (cddr remaining) (hash-set h (car remaining) (cadr remaining)))]
      [else
       (error 'parse-application "expected keyword, got: ~s" (car remaining))])))

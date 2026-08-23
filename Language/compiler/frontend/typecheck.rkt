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
         racket/string
         "../ast.rkt"
         "resolve.rkt")

(provide typecheck-pass)

;; ─── Type & Effect Checker ────────────────────────────────────────────────────
;; Input:  resolved-program (from resolve-pass)
;; Output: resolved-program unchanged if valid
;; Raises: exn:fail with file/line/phase/message if invalid
;;
;; Checks:
;;   1. Expression type consistency
;;   2. Field access type matching
;;   3. Payload parameter scalar/trivially-copyable types
;;   4. Action target parameter is (Pointer VertexType)
;;   5. Propagate argument count and types match target action
;;   6. Application root-arguments match root-action payload types
;;   7. Phase effects (predicate purity, work no-propagate, diffuse no-mutate,
;;      ghost-safety: diffuse propagation args must not read vertex fields)

;; ─── Error reporting ──────────────────────────────────────────────────────────

(define (tc-error phase msg . args)
  (error 'typecheck "~a: ~a" phase (apply format msg args)))

;; ─── Type predicates ──────────────────────────────────────────────────────────

(define (scalar-type? t)
  ;; Types that are trivially copyable and valid as payload parameters
  (or (t-u32? t) (t-bool? t) (t-address? t)))

(define (numeric-type? t)
  (t-u32? t))

(define (type-equal? a b)
  (match* (a b)
    [((t-u32) (t-u32)) #t]
    [((t-bool) (t-bool)) #t]
    [((t-address) (t-address)) #t]
    [((t-edge) (t-edge)) #t]
    [((t-unit) (t-unit)) #t]
    [((t-pointer vn1) (t-pointer vn2)) (eq? vn1 vn2)]
    [((t-vector e1) (t-vector e2)) (type-equal? e1 e2)]
    [(_ _) #f]))

(define (type->string t)
  (match t
    [(t-u32) "UInt32"]
    [(t-bool) "Boolean"]
    [(t-address) "Address"]
    [(t-edge) "Edge"]
    [(t-unit) "Unit"]
    [(t-pointer vn) (format "(Pointer ~a)" vn)]
    [(t-vector e) (format "(Vector ~a)" (type->string e))]
    [_ "unknown"]))

;; ─── Type environment ─────────────────────────────────────────────────────────
;; A type-env is a hash: symbol -> type

(define (make-type-env vertex-decl constants actions)
  (define env (make-hash))
  ;; Register constants
  (for ([c (in-list constants)])
    (hash-set! env (constant-decl-name c) (constant-decl-type c)))
  env)

;; ─── Expression type inference ────────────────────────────────────────────────

;; Returns the type of an expression given the environment.
;; phase: symbol for error reporting
;; env: hash symbol->type
;; vertex: vertex-decl (for field lookups)
;; edge-vars: set of symbols that are edge iteration variables
(define (infer-expr-type expr env vertex edge-vars phase)
  (match expr
    [(literal-expr _ _ type) type]

    [(var-expr _ name)
     (cond
       [(hash-has-key? env name) (hash-ref env name)]
       [else (tc-error phase "undefined variable: ~a" name)])]

    [(prim-expr _ op args)
     (define arg-types (map (λ (a) (infer-expr-type a env vertex edge-vars phase)) args))
     (check-prim-types op arg-types phase)
     (prim-result-type op arg-types)]

    [(vertex-read-expr _ field target)
     (define target-type (infer-expr-type target env vertex edge-vars phase))
     (unless (t-pointer? target-type)
       (tc-error phase "vertex field access on non-pointer type: ~a"
                 (type->string target-type)))
     (cond
       [(eq? field 'id) (t-address)]
       [else
        (define f (find-field vertex field))
        (unless f
          (tc-error phase "unknown vertex field: ~a" field))
        (field-decl-type f)])]

    [(edge-read-expr _ accessor target)
     (define target-type (infer-expr-type target env vertex edge-vars phase))
     (unless (t-edge? target-type)
       (tc-error phase "edge accessor on non-edge type: ~a"
                 (type->string target-type)))
     (match accessor
       ['address (t-address)]
       ['weight (t-u32)]
       [_ (tc-error phase "unknown edge accessor: ~a" accessor)])]

    [(let-expr _ bindings body)
     (define new-env (hash-copy env))
     (for ([b (in-list bindings)])
       (define bname (car b))
       (define bexpr (cdr b))
       (define btype (infer-expr-type bexpr env vertex edge-vars phase))
       (hash-set! new-env bname btype))
     (infer-expr-type body new-env vertex edge-vars phase)]

    [(if-expr _ test then else-branch)
     (define test-type (infer-expr-type test env vertex edge-vars phase))
     (unless (t-bool? test-type)
       (tc-error phase "if condition must be Boolean, got ~a"
                 (type->string test-type)))
     (define then-type (infer-expr-type then env vertex edge-vars phase))
     (define else-type (infer-expr-type else-branch env vertex edge-vars phase))
     (unless (type-equal? then-type else-type)
       (tc-error phase "if branches have different types: ~a vs ~a"
                 (type->string then-type) (type->string else-type)))
     then-type]

    [_ (tc-error phase "unknown expression form")]))

;; ─── Primitive type checking ──────────────────────────────────────────────────

(define (check-prim-types op arg-types phase)
  (match op
    [(or '+ '- '*)
     (unless (and (= (length arg-types) 2)
                  (andmap numeric-type? arg-types))
       (tc-error phase "arithmetic operator '~a' requires two UInt32 operands, got (~a)"
                 op (string-join (map type->string arg-types) ", ")))]
    [(or '> '>= '< '<=)
     (unless (and (= (length arg-types) 2)
                  (andmap numeric-type? arg-types))
       (tc-error phase "comparison '~a' requires two UInt32 operands, got (~a)"
                 op (string-join (map type->string arg-types) ", ")))]
    ['=
     (unless (and (= (length arg-types) 2)
                  (type-equal? (first arg-types) (second arg-types)))
       (tc-error phase "equality '=' requires two operands of same type, got (~a)"
                 (string-join (map type->string arg-types) ", ")))]
    ['and
     (unless (and (= (length arg-types) 2)
                  (andmap t-bool? arg-types))
       (tc-error phase "'and' requires two Boolean operands, got (~a)"
                 (string-join (map type->string arg-types) ", ")))]
    ['or
     (unless (and (= (length arg-types) 2)
                  (andmap t-bool? arg-types))
       (tc-error phase "'or' requires two Boolean operands, got (~a)"
                 (string-join (map type->string arg-types) ", ")))]
    ['not
     (unless (and (= (length arg-types) 1)
                  (t-bool? (first arg-types)))
       (tc-error phase "'not' requires one Boolean operand, got (~a)"
                 (string-join (map type->string arg-types) ", ")))]
    [_ (tc-error phase "unknown primitive operator: ~a" op)]))

(define (prim-result-type op arg-types)
  (match op
    [(or '+ '- '*) (t-u32)]
    [(or '> '>= '< '<= '=) (t-bool)]
    [(or 'and 'or 'not) (t-bool)]
    [_ (t-unit)]))

;; ─── Field lookup ─────────────────────────────────────────────────────────────

(define (find-field vertex field-name)
  (for/first ([f (in-list (vertex-decl-fields vertex))]
              #:when (eq? (field-decl-name f) field-name))
    f))

;; ─── Effect checking on statements ───────────────────────────────────────────

;; Check a statement in a given phase context
;; phase: 'work or 'diffuse
;; env: type environment
;; vertex: vertex-decl
;; edge-vars: set of edge iteration variable names
;; actions: list of action-decl (for propagate type matching)
(define (check-stmt stmt env vertex edge-vars phase actions)
  (match stmt
    [(set-field-stmt _ field target value)
     ;; Effect: mutation
     (when (eq? phase 'diffuse)
       (tc-error "diffuse" "mutation (set-field ~a) is not allowed in diffuse phase" field))
     ;; Type check: target must be a pointer
     (define target-type (infer-expr-type target env vertex edge-vars (symbol->string phase)))
     (unless (t-pointer? target-type)
       (tc-error (symbol->string phase) "set-field target must be a pointer, got ~a"
                 (type->string target-type)))
     ;; Type check: value must match field type
     (define f (find-field vertex field))
     (unless f
       (tc-error (symbol->string phase) "unknown vertex field: ~a" field))
     (unless (field-decl-mutable? f)
       (tc-error (symbol->string phase) "field ~a is not mutable" field))
     (define value-type (infer-expr-type value env vertex edge-vars (symbol->string phase)))
     (define field-type (field-decl-type f))
     (unless (type-equal? value-type field-type)
       (tc-error (symbol->string phase)
                 "type mismatch in set-field ~a: expected ~a, got ~a"
                 field (type->string field-type) (type->string value-type)))]

    [(for-edges-stmt _ edge-var target body)
     ;; target must be a pointer to vertex
     (define target-type (infer-expr-type target env vertex edge-vars (symbol->string phase)))
     (unless (t-pointer? target-type)
       (tc-error (symbol->string phase) "for-edges target must be a pointer, got ~a"
                 (type->string target-type)))
     ;; Add edge-var to env and edge-vars set
     (define new-env (hash-copy env))
     (hash-set! new-env edge-var (t-edge))
     (define new-edge-vars (cons edge-var edge-vars))
     (for ([s (in-list body)])
       (check-stmt s new-env vertex new-edge-vars phase actions))]

    [(propagate-stmt _ action-name destination args)
     ;; Effect: propagation
     (when (eq? phase 'work)
       (tc-error "work" "propagate is not allowed in work phase"))
     ;; Type check destination: must be Address
     (define dest-type (infer-expr-type destination env vertex edge-vars (symbol->string phase)))
     (unless (t-address? dest-type)
       (tc-error (symbol->string phase)
                 "propagate destination must be Address, got ~a"
                 (type->string dest-type)))
     ;; Find the target action
     (define target-action
       (for/first ([a (in-list actions)]
                   #:when (eq? (action-decl-name a) action-name))
         a))
     (unless target-action
       (tc-error (symbol->string phase) "propagate references unknown action: ~a" action-name))
     ;; Check argument count
     (define expected-params (action-decl-params target-action))
     (unless (= (length args) (length expected-params))
       (tc-error (symbol->string phase)
                 "propagate to ~a: expected ~a arguments, got ~a"
                 action-name (length expected-params) (length args)))
     ;; Check argument types
     (for ([arg (in-list args)]
           [param (in-list expected-params)]
           [i (in-naturals 1)])
       (define arg-type (infer-expr-type arg env vertex edge-vars (symbol->string phase)))
       (define param-type (param-decl-type param))
       (unless (type-equal? arg-type param-type)
         (tc-error (symbol->string phase)
                   "propagate to ~a: argument ~a type mismatch, expected ~a got ~a"
                   action-name i (type->string param-type) (type->string arg-type))))
     ;; Ghost-safety: in diffuse, propagation args must not read vertex fields
     (when (eq? phase 'diffuse)
       (for ([arg (in-list args)]
             [i (in-naturals 1)])
         (when (expr-reads-vertex-field? arg)
           (tc-error "diffuse"
                     "ghost-safety violation: propagate argument ~a reads vertex field"
                     i))))]

    [(let-stmt _ bindings body)
     (define new-env (hash-copy env))
     (for ([b (in-list bindings)])
       (define bname (car b))
       (define bexpr (cdr b))
       (define btype (infer-expr-type bexpr env vertex edge-vars (symbol->string phase)))
       (hash-set! new-env bname btype))
     (for ([s (in-list body)])
       (check-stmt s new-env vertex edge-vars phase actions))]

    [(if-stmt _ test then-stmts else-stmts)
     (define test-type (infer-expr-type test env vertex edge-vars (symbol->string phase)))
     (unless (t-bool? test-type)
       (tc-error (symbol->string phase) "if condition must be Boolean, got ~a"
                 (type->string test-type)))
     (for ([s (in-list then-stmts)])
       (check-stmt s env vertex edge-vars phase actions))
     (for ([s (in-list else-stmts)])
       (check-stmt s env vertex edge-vars phase actions))]

    [(begin-stmt _ stmts)
     (for ([s (in-list stmts)])
       (check-stmt s env vertex edge-vars phase actions))]

    [_ (tc-error (symbol->string phase) "unknown statement form")]))

;; ─── Ghost-safety: check if an expression reads a vertex field ────────────────

(define (expr-reads-vertex-field? expr)
  (match expr
    [(vertex-read-expr _ _ _) #t]
    [(literal-expr _ _ _) #f]
    [(var-expr _ _) #f]
    [(prim-expr _ _ args)
     (ormap expr-reads-vertex-field? args)]
    [(edge-read-expr _ _ _) #f]
    [(let-expr _ bindings body)
     (or (ormap (λ (b) (expr-reads-vertex-field? (cdr b))) bindings)
         (expr-reads-vertex-field? body))]
    [(if-expr _ test then else-branch)
     (or (expr-reads-vertex-field? test)
         (expr-reads-vertex-field? then)
         (expr-reads-vertex-field? else-branch))]
    [_ #f]))

;; ─── Predicate purity check ──────────────────────────────────────────────────
;; Predicate expressions must be pure: no mutation, no propagate.
;; Since predicates are expressions (not statements), they cannot contain
;; set-field or propagate by construction. We only need to verify the
;; expression type is Boolean.

(define (check-predicate-expr pred-expr env vertex edge-vars phase-label)
  (define pred-type (infer-expr-type pred-expr env vertex edge-vars phase-label))
  (unless (t-bool? pred-type)
    (tc-error phase-label "predicate must be Boolean, got ~a" (type->string pred-type))))

;; ─── Main typecheck pass ──────────────────────────────────────────────────────

(define (typecheck-pass resolved)
  (define constants (resolved-program-constants resolved))
  (define vertex (resolved-program-vertex resolved))
  (define actions (resolved-program-actions resolved))
  (define app (resolved-program-application resolved))

  ;; Check each action
  (for ([action (in-list actions)])
    (check-action action vertex constants actions))

  ;; Check application root-arguments match root-action payload types
  (check-root-arguments app actions)

  ;; Return unchanged if all checks pass
  resolved)

;; ─── Action checking ──────────────────────────────────────────────────────────

(define (check-action action vertex constants actions)
  (define aname (action-decl-name action))
  (define target (action-decl-target action))
  (define params (action-decl-params action))

  ;; Rule 4: target parameter must be (Pointer VertexType)
  (unless (t-pointer? (param-decl-type target))
    (tc-error "action"
              "action ~a: target parameter must be (Pointer VertexType), got ~a"
              aname (type->string (param-decl-type target))))
  ;; Verify the pointer points to the vertex type
  (unless (eq? (t-pointer-vertex-name (param-decl-type target))
               (vertex-decl-name vertex))
    (tc-error "action"
              "action ~a: target must point to ~a, got (Pointer ~a)"
              aname (vertex-decl-name vertex)
              (t-pointer-vertex-name (param-decl-type target))))

  ;; Rule 3: payload parameters must be scalar types
  (for ([p (in-list params)])
    (unless (scalar-type? (param-decl-type p))
      (tc-error "action"
                "action ~a: payload parameter ~a has non-scalar type ~a"
                aname (param-decl-name p) (type->string (param-decl-type p)))))

  ;; Build type environment for this action
  (define env (make-hash))
  ;; constants
  (for ([c (in-list constants)])
    (hash-set! env (constant-decl-name c) (constant-decl-type c)))
  ;; target
  (hash-set! env (param-decl-name target) (param-decl-type target))
  ;; payload params
  (for ([p (in-list params)])
    (hash-set! env (param-decl-name p) (param-decl-type p)))

  (define edge-vars '())

  ;; Check predicate (must be Boolean and pure)
  (check-predicate-expr (action-decl-predicate action) env vertex edge-vars "predicate")

  ;; Check work statements (may mutate, must NOT propagate)
  (for ([s (in-list (action-decl-work action))])
    (check-stmt s env vertex edge-vars 'work actions))

  ;; Check diffuse predicate (must be Boolean)
  (check-predicate-expr (action-decl-diffuse-predicate action) env vertex edge-vars "diffuse-predicate")

  ;; Check diffuse statements (may propagate, must NOT mutate)
  (for ([s (in-list (action-decl-diffuse action))])
    (check-stmt s env vertex edge-vars 'diffuse actions)))

;; ─── Root-arguments checking ──────────────────────────────────────────────────

(define (check-root-arguments app actions)
  (define root-action-name (application-decl-root-action app))
  (define root-args (application-decl-root-arguments app))

  ;; Find the root action
  (define root-action
    (for/first ([a (in-list actions)]
                #:when (eq? (action-decl-name a) root-action-name))
      a))
  (unless root-action
    (tc-error "application" "unknown root-action: ~a" root-action-name))

  (define expected-params (action-decl-params root-action))

  ;; root-arguments is a list of literal values from the parse
  ;; Check count
  (unless (= (length root-args) (length expected-params))
    (tc-error "application"
              "root-arguments count mismatch: expected ~a, got ~a"
              (length expected-params) (length root-args)))

  ;; Check that each root-arg value is compatible with the param type
  (for ([arg (in-list root-args)]
        [param (in-list expected-params)]
        [i (in-naturals 1)])
    (define param-type (param-decl-type param))
    (define arg-type (infer-literal-type arg))
    (unless (type-equal? arg-type param-type)
      (tc-error "application"
                "root-argument ~a type mismatch: expected ~a, got ~a"
                i (type->string param-type) (type->string arg-type)))))

(define (infer-literal-type val)
  (cond
    [(exact-nonnegative-integer? val) (t-u32)]
    [(boolean? val) (t-bool)]
    [else (t-u32)]  ;; fallback for numeric constants
    ))

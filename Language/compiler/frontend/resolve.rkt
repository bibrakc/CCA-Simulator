#lang racket/base

;; BSD 3-Clause License
;;
;; Copyright (c) 2026, Bibrak Qamar

(require racket/match
         racket/list
         racket/string
         "../ast.rkt")

(provide resolve-pass
         resolved-program
         resolved-program?
         resolved-program-constants
         resolved-program-vertex
         resolved-program-actions
         resolved-program-application
         resolved-program-symbols)

;; ═══════════════════════════════════════════════════════════════════════════════
;; Name resolution pass — builds a symbol table and validates references.
;;
;; This pass runs after parsing and before typechecking. It:
;;   1. Registers all named entities (constants, fields, actions, vertex type)
;;      into a symbol table with their C++-mangled names.
;;   2. Validates cross-references:
;;      - application's root-action references a defined action
;;      - application's vertex-type references the defined vertex
;;      - application's result-field references a defined field
;;      - propagate targets in diffuse bodies reference defined actions
;;   3. Performs name mangling (Scheme → C++ identifier conversion):
;;      - Hyphens become underscores
;;      - Bangs are removed
;;      - C++ keywords get a "cca_" prefix
;;
;; Inputs:  cca-program struct (from parse-pass).
;; Outputs: resolved-program struct (wraps the AST with a symbol table).
;; ═══════════════════════════════════════════════════════════════════════════════

;; ─── Resolved program ─────────────────────────────────────────────────────────
;; After resolution, the program carries a symbol table mapping source names
;; to their kind, type, and C++ mangled name.

(struct resolved-program (constants vertex actions application symbols) #:transparent)

;; symbol-entry: one entry in the symbol table.
;;   kind: 'constant, 'field, 'action, 'param, or 'vertex-type
;;   cpp-name: the C++-safe mangled identifier
(struct symbol-entry (name kind type cpp-name) #:transparent)

;; ─── Name mangling ───────────────────────────────────────────────────────────
(define (mangle-name sym)
  ;; Convert Scheme-style names to C++ identifiers
  ;; - dashes become underscores
  ;; - ! is stripped (used for mutation: set-vertex-level!)
  ;; - ? becomes _p_0x3f (predicate suffix, avoids collision with non-? names)
  (define s (symbol->string sym))
  (define mangled (string-replace (string-replace (string-replace s "-" "_") "!" "") "?" "_p_0x3f"))
  ;; Prefix with cca_ if it would conflict with C++ keywords
  (if (member mangled cpp-keywords) (string-append "cca_" mangled) mangled))

(define cpp-keywords
  '("auto" "break" "case" "char" "const" "continue" "default" "do" "double"
    "else" "enum" "extern" "float" "for" "goto" "if" "int" "long" "register"
    "return" "short" "signed" "sizeof" "static" "struct" "switch" "typedef"
    "union" "unsigned" "void" "volatile" "while" "class" "public" "private"
    "protected" "virtual" "template" "typename" "namespace" "using" "new"
    "delete" "true" "false" "nullptr" "inline" "constexpr"))

;; ─── Resolution pass ──────────────────────────────────────────────────────────
(define (resolve-pass program)
  (define symbols (make-hash))

  ;; Register constants
  (for ([c (in-list (cca-program-constants program))])
    (define name (constant-decl-name c))
    (hash-set! symbols name
               (symbol-entry name 'constant (constant-decl-type c) (mangle-name name))))

  ;; Register vertex type and its fields
  (define vtx (cca-program-vertex program))
  (define vtx-name (vertex-decl-name vtx))
  (hash-set! symbols vtx-name
             (symbol-entry vtx-name 'vertex-type (t-pointer vtx-name) (mangle-name vtx-name)))

  (for ([f (in-list (vertex-decl-fields vtx))])
    (define fname (field-decl-name f))
    (hash-set! symbols (string->symbol (format "vertex-~a" fname))
               (symbol-entry fname 'field (field-decl-type f) (mangle-name fname))))

  ;; Register actions
  (for ([a (in-list (cca-program-actions program))])
    (define aname (action-decl-name a))
    (hash-set! symbols aname
               (symbol-entry aname 'action #f (mangle-name aname))))

  ;; Validate action references in application
  (define app (cca-program-application program))
  (unless (hash-has-key? symbols (application-decl-root-action app))
    (error 'resolve "unknown root-action: ~a" (application-decl-root-action app)))
  (unless (hash-has-key? symbols (application-decl-vertex-type app))
    (error 'resolve "unknown vertex-type: ~a" (application-decl-vertex-type app)))

  ;; Validate field reference in result-field
  (when (application-decl-result-field app)
    (define rf (application-decl-result-field app))
    (define field-key (string->symbol (format "vertex-~a" rf)))
    (unless (hash-has-key? symbols field-key)
      (error 'resolve "unknown result-field: ~a" rf)))

  ;; Validate propagate targets in action bodies
  (for ([a (in-list (cca-program-actions program))])
    (check-propagate-targets (action-decl-diffuse a) symbols))

  (resolved-program
   (cca-program-constants program)
   (cca-program-vertex program)
   (cca-program-actions program)
   (cca-program-application program)
   symbols))

;; Check that propagate references resolve to known actions
(define (check-propagate-targets stmts symbols)
  (for ([s (in-list stmts)])
    (match s
      [(propagate-stmt _ action-name _ _)
       (unless (hash-has-key? symbols action-name)
         (error 'resolve "propagate references unknown action: ~a" action-name))]
      [(for-edges-stmt _ _ _ body)
       (check-propagate-targets body symbols)]
      [(let-stmt _ _ body)
       (check-propagate-targets body symbols)]
      [(if-stmt _ _ then-stmts else-stmts)
       (check-propagate-targets then-stmts symbols)
       (check-propagate-targets else-stmts symbols)]
      [(begin-stmt _ stmts)
       (check-propagate-targets stmts symbols)]
      [_ (void)])))

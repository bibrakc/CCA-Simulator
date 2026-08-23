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
         racket/string
         racket/list
         racket/format
         racket/port)

(provide (all-defined-out))

;; ═══════════════════════════════════════════════════════════════════════════════
;; C++ IR Node Definitions
;; ═══════════════════════════════════════════════════════════════════════════════

;; ─── Types ────────────────────────────────────────────────────────────────────
(struct cpp-type-raw (text) #:transparent)          ; raw type string e.g. "u_int32_t"
(struct cpp-type-template (name args) #:transparent) ; e.g. Graph<BFSVertex<...>>
(struct cpp-type-const (inner) #:transparent)       ; const T
(struct cpp-type-ptr (inner) #:transparent)         ; T*
(struct cpp-type-ref (inner) #:transparent)         ; T&

;; ─── Expressions ──────────────────────────────────────────────────────────────
(struct cpp-literal (value) #:transparent)           ; literal value (number, string, bool)
(struct cpp-ident (name) #:transparent)              ; identifier
(struct cpp-binop (op lhs rhs) #:transparent)        ; binary operation
(struct cpp-unop (op operand) #:transparent)         ; unary operation
(struct cpp-member (obj field) #:transparent)        ; obj.field
(struct cpp-arrow (obj field) #:transparent)         ; obj->field
(struct cpp-index (obj idx) #:transparent)           ; obj[idx]
(struct cpp-call (func args) #:transparent)          ; func(args...)
(struct cpp-template-call (func targs args) #:transparent) ; func<T>(args...)
(struct cpp-cast (type expr) #:transparent)          ; static_cast<T>(expr)
(struct cpp-paren (expr) #:transparent)              ; (expr)
(struct cpp-raw-expr (text) #:transparent)           ; raw expression text

;; ─── Statements ───────────────────────────────────────────────────────────────
(struct cpp-var-decl (type name init) #:transparent)   ; type name = init; or type name;
(struct cpp-var-decl-const (type name init) #:transparent)  ; type const name = init;
(struct cpp-auto-decl (name init) #:transparent)       ; auto name = init;
(struct cpp-auto-ptr-decl (name init) #:transparent)   ; auto* name = init;
(struct cpp-assign (target value) #:transparent)       ; target = value;
(struct cpp-expr-stmt (expr) #:transparent)            ; expr;
(struct cpp-return (expr) #:transparent)               ; return expr;
(struct cpp-if (cond then else-branch) #:transparent)  ; if (cond) { then } else { else }
(struct cpp-for (init cond step body) #:transparent)   ; for (init; cond; step) { body }
(struct cpp-block (stmts) #:transparent)               ; { stmts... }
(struct cpp-raw-stmt (text) #:transparent)             ; raw statement text (includes semicolon/newline)
(struct cpp-blank-line () #:transparent)               ; blank line

;; ─── Declarations (top-level) ─────────────────────────────────────────────────
(struct cpp-include (path system?) #:transparent)           ; #include "path" or <path>
(struct cpp-ifndef-guard (name body) #:transparent)         ; #ifndef / #define / #endif
(struct cpp-constexpr-var (type name value) #:transparent)  ; inline static constexpr type name = val;
(struct cpp-extern-var (type name) #:transparent)           ; extern type name;
(struct cpp-global-var (type name) #:transparent)           ; type name; (at file scope)
(struct cpp-struct-decl (name fields base) #:transparent)   ; struct name : base { fields }
(struct cpp-template-struct (tparam struct-decl body) #:transparent) ; template<typename T> struct...
;; template: #f or string (template param name)
;; inline?: boolean
;; trailing-ret?: boolean (use auto ... -> RetType syntax)
(struct cpp-func-decl (name ret-type params body template inline? trailing-ret?) #:transparent)
(struct cpp-comment (text) #:transparent)                   ; // text
(struct cpp-raw-decl (text) #:transparent)                  ; raw top-level text

;; ─── File ─────────────────────────────────────────────────────────────────────
(struct cpp-file (items) #:transparent)  ; list of top-level items

;; ═══════════════════════════════════════════════════════════════════════════════
;; Pretty Printer
;; ═══════════════════════════════════════════════════════════════════════════════

;; Main entry point: render a cpp-file to a string
(define (cpp-ir->string node)
  (define out (open-output-string))
  (emit-node node 0 out)
  (get-output-string out))

;; ─── Emit top-level node ──────────────────────────────────────────────────────
(define (emit-node node indent out)
  (match node
    [(cpp-file items)
     (for ([item (in-list items)])
       (emit-node item indent out))]

    [(cpp-comment text)
     (indent! indent out)
     (fprintf out "// ~a\n" text)]

    [(cpp-raw-decl text)
     (display text out)]

    [(cpp-include path sys?)
     (if sys?
         (fprintf out "#include <~a>\n" path)
         (fprintf out "#include \"~a\"\n" path))]

    [(cpp-ifndef-guard name body)
     (fprintf out "#ifndef ~a\n#define ~a\n\n" name name)
     (for ([item (in-list body)])
       (emit-node item indent out))
     (fprintf out "#endif // ~a\n" name)]

    [(cpp-constexpr-var type name value)
     (indent! indent out)
     (fprintf out "inline static constexpr ~a ~a = ~a;\n" (type->string type) name value)]

    [(cpp-extern-var type name)
     (indent! indent out)
     (fprintf out "extern ~a ~a;\n" (type->string type) name)]

    [(cpp-global-var type name)
     (indent! indent out)
     (fprintf out "~a ~a;\n" (type->string type) name)]

    [(cpp-struct-decl name fields base)
     (emit-struct node indent out)]

    [(cpp-template-struct tparam struct-decl body)
     (emit-template-struct node indent out)]

    [(cpp-func-decl name ret-type params body tmpl inl? trail?)
     (emit-function node indent out)]

    [(cpp-blank-line)
     (newline out)]

    [(cpp-var-decl type name init)
     (emit-stmt node indent out)]
    [(cpp-var-decl-const type name init)
     (emit-stmt node indent out)]
    [(cpp-auto-decl _ _)
     (emit-stmt node indent out)]
    [(cpp-auto-ptr-decl _ _)
     (emit-stmt node indent out)]
    [(cpp-assign _ _)
     (emit-stmt node indent out)]
    [(cpp-expr-stmt _)
     (emit-stmt node indent out)]
    [(cpp-return _)
     (emit-stmt node indent out)]
    [(cpp-if _ _ _)
     (emit-stmt node indent out)]
    [(cpp-for _ _ _ _)
     (emit-stmt node indent out)]
    [(cpp-block _)
     (emit-stmt node indent out)]
    [(cpp-raw-stmt _)
     (emit-stmt node indent out)]

    [_ (error 'emit-node "unknown node: ~s" node)]))

;; ─── Emit struct ──────────────────────────────────────────────────────────────
(define (emit-struct node indent out)
  (match-define (cpp-struct-decl name fields base) node)
  (indent! indent out)
  (if base
      (fprintf out "struct ~a : ~a\n" name base)
      (fprintf out "struct ~a\n" name))
  (indent! indent out)
  (display "{\n" out)
  (for ([f (in-list fields)])
    (emit-node f (+ indent 4) out))
  (indent! indent out)
  (display "};\n" out))

;; ─── Emit template struct (for vertex) ───────────────────────────────────────
(define (emit-template-struct node indent out)
  (match-define (cpp-template-struct tparam sdecl body) node)
  (indent! indent out)
  (fprintf out "template<typename ~a>\n" tparam)
  (match-define (cpp-struct-decl name fields base) sdecl)
  (indent! indent out)
  (if base
      (fprintf out "struct ~a : ~a\n" name base)
      (fprintf out "struct ~a\n" name))
  (indent! indent out)
  (display "{\n" out)
  (for ([item (in-list body)])
    (emit-node item (+ indent 4) out))
  (indent! indent out)
  (display "};\n" out))

;; ─── Emit function ───────────────────────────────────────────────────────────
(define (emit-function node indent out)
  (match-define (cpp-func-decl name ret-type params body tmpl inl? trail?) node)
  (when tmpl
    (indent! indent out)
    (fprintf out "template<typename ~a>\n" tmpl))
  (when inl?
    (indent! indent out)
    (display "inline " out)
    ;; inline goes on same line as 'auto' if trailing-ret
    (when trail?
      (display "auto\n" out)
      (indent! indent out)
      (fprintf out "~a(~a) -> ~a\n"
               name
               (params->string params)
               (type->string ret-type))
      (emit-body body indent out)
      (return)))

  (cond
    [trail?
     (indent! indent out)
     (display "auto\n" out)
     (indent! indent out)
     (fprintf out "~a(~a) -> ~a\n"
              name
              (params->string params)
              (type->string ret-type))]
    [else
     (indent! indent out)
     (fprintf out "~a ~a(~a)\n"
              (type->string ret-type)
              name
              (params->string params))])

  (emit-body body indent out))

(define-syntax-rule (return) (void))

(define (emit-body body indent out)
  (indent! indent out)
  (display "{\n" out)
  (for ([s (in-list body)])
    (emit-stmt s (+ indent 4) out))
  (indent! indent out)
  (display "}\n" out))

;; ─── Emit statement ──────────────────────────────────────────────────────────
(define (emit-stmt stmt indent out)
  (match stmt
    [(cpp-var-decl type name init)
     (indent! indent out)
     (if init
         (fprintf out "~a ~a = ~a;\n" (type->string type) name (expr->string init))
         (fprintf out "~a ~a;\n" (type->string type) name))]

    [(cpp-var-decl-const type name init)
     (indent! indent out)
     (fprintf out "~a const ~a = ~a;\n" (type->string type) name (expr->string init))]

    [(cpp-auto-decl name init)
     (indent! indent out)
     (fprintf out "auto ~a = ~a;\n" name (expr->string init))]

    [(cpp-auto-ptr-decl name init)
     (indent! indent out)
     (fprintf out "auto* ~a = ~a;\n" name (expr->string init))]

    [(cpp-assign target value)
     (indent! indent out)
     (fprintf out "~a = ~a;\n" (expr->string target) (expr->string value))]

    [(cpp-expr-stmt expr)
     (indent! indent out)
     (fprintf out "~a;\n" (expr->string expr))]

    [(cpp-return expr)
     (indent! indent out)
     (fprintf out "return ~a;\n" (expr->string expr))]

    [(cpp-if test-e then-stmts else-stmts)
     (indent! indent out)
     (fprintf out "if (~a) {\n" (expr->string test-e))
     (for ([s (in-list then-stmts)])
       (emit-stmt s (+ indent 4) out))
     (indent! indent out)
     (if (null? else-stmts)
         (display "}\n" out)
         (begin
           (display "} else {\n" out)
           (for ([s (in-list else-stmts)])
             (emit-stmt s (+ indent 4) out))
           (indent! indent out)
           (display "}\n" out)))]

    [(cpp-for init-s cond-s step-s body-stmts)
     (indent! indent out)
     (fprintf out "for (~a; ~a; ~a) {\n" init-s cond-s step-s)
     (for ([s (in-list body-stmts)])
       (emit-stmt s (+ indent 4) out))
     (indent! indent out)
     (display "}\n" out)]

    [(cpp-block stmts)
     (indent! indent out)
     (display "{\n" out)
     (for ([s (in-list stmts)])
       (emit-stmt s (+ indent 4) out))
     (indent! indent out)
     (display "}\n" out)]

    [(cpp-raw-stmt text)
     (indent! indent out)
     (display text out)
     (newline out)]

    [(cpp-blank-line)
     (newline out)]

    [(cpp-comment text)
     (indent! indent out)
     (fprintf out "// ~a\n" text)]

    [_ (error 'emit-stmt "unknown statement: ~s" stmt)]))

;; ─── Expression to string ─────────────────────────────────────────────────────
(define (expr->string expr)
  (match expr
    [(cpp-literal v)
     (cond
       [(string? v) (format "\"~a\"" v)]
       [(boolean? v) (if v "true" "false")]
       [else (format "~a" v)])]

    [(cpp-ident name) name]

    [(cpp-binop op lhs rhs)
     (format "(~a ~a ~a)" (expr->string lhs) op (expr->string rhs))]

    [(cpp-unop op operand)
     (format "(~a~a)" op (expr->string operand))]

    [(cpp-member obj field)
     (format "~a.~a" (expr->string obj) field)]

    [(cpp-arrow obj field)
     (format "~a->~a" (expr->string obj) field)]

    [(cpp-index obj idx)
     (format "~a[~a]" (expr->string obj) (expr->string idx))]

    [(cpp-call func args)
     (format "~a(~a)" (expr->string func) (string-join (map expr->string args) ", "))]

    [(cpp-template-call func targs args)
     (format "~a<~a>(~a)"
             (expr->string func)
             (string-join targs ", ")
             (string-join (map expr->string args) ", "))]

    [(cpp-cast type e)
     (format "static_cast<~a>(~a)" (type->string type) (expr->string e))]

    [(cpp-paren e)
     (format "(~a)" (expr->string e))]

    [(cpp-raw-expr text) text]

    [_ (error 'expr->string "unknown expression: ~s" expr)]))

;; ─── Type to string ───────────────────────────────────────────────────────────
(define (type->string type)
  (match type
    [(cpp-type-raw text) text]
    [(cpp-type-template name args)
     (format "~a<~a>" name (string-join (map type->string args) ", "))]
    [(cpp-type-const inner)
     (format "~a const" (type->string inner))]
    [(cpp-type-ptr inner)
     (format "~a*" (type->string inner))]
    [(cpp-type-ref inner)
     (format "~a&" (type->string inner))]
    [(? string?) type]
    [_ (error 'type->string "unknown type: ~s" type)]))

;; ─── Params to string ─────────────────────────────────────────────────────────
;; params is a list of (type-string name-string) pairs or raw strings
(define (params->string params)
  (string-join
   (for/list ([p (in-list params)])
     (cond
       [(list? p) (format "~a ~a" (car p) (cadr p))]
       [(string? p) p]
       [else (error 'params->string "bad param: ~s" p)]))
   ",\n               "))

;; ─── Indent helper ────────────────────────────────────────────────────────────
(define (indent! n out)
  (display (make-string n #\space) out))

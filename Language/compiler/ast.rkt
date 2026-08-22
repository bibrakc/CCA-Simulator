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



(provide (all-defined-out))

;; ─── Source spans ─────────────────────────────────────────────────────────────
(struct source-span (file line col) #:transparent)

(define no-span (source-span #f #f #f))

;; ─── Types ────────────────────────────────────────────────────────────────────
(struct t-unit () #:transparent)
(struct t-bool () #:transparent)
(struct t-u32 () #:transparent)
(struct t-address () #:transparent)
(struct t-edge () #:transparent)
(struct t-pointer (vertex-name) #:transparent)
(struct t-vector (element-type) #:transparent)

;; ─── Expressions ──────────────────────────────────────────────────────────────
(struct expr (span) #:transparent)
(struct literal-expr expr (value type) #:transparent)
(struct var-expr expr (name) #:transparent)
(struct prim-expr expr (op args) #:transparent)
(struct vertex-read-expr expr (field target) #:transparent)
(struct edge-read-expr expr (accessor target) #:transparent)  ; accessor: 'address or 'weight
(struct let-expr expr (bindings body) #:transparent)
(struct if-expr expr (test then else-branch) #:transparent)

;; ─── Statements ───────────────────────────────────────────────────────────────
(struct stmt (span) #:transparent)
(struct set-field-stmt stmt (field target value) #:transparent)
(struct for-edges-stmt stmt (edge-var target body) #:transparent)
(struct propagate-stmt stmt (action-name destination args) #:transparent)
(struct let-stmt stmt (bindings body) #:transparent)
(struct if-stmt stmt (test then-stmts else-stmts) #:transparent)
(struct begin-stmt stmt (stmts) #:transparent)

;; ─── Declarations ─────────────────────────────────────────────────────────────
(struct constant-decl (name type value span) #:transparent)

(struct field-decl (name type initial mutable? annotations span) #:transparent)
(struct vertex-decl (name fields span) #:transparent)

(struct param-decl (name type span) #:transparent)
(struct action-decl (name target params predicate work diffuse-predicate diffuse span)
  #:transparent)

(struct application-decl
  (name binary-name vertex-type root-action root-arguments result-field verification span)
  #:transparent)

;; ─── Program ──────────────────────────────────────────────────────────────────
(struct cca-program (constants vertex actions application span) #:transparent)

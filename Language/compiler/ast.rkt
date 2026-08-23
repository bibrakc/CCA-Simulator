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

;; ═══════════════════════════════════════════════════════════════════════════════
;; Abstract Syntax Tree definitions for the CCA language.
;;
;; This module defines the IR that all compiler passes operate on.
;; The grammar it represents:
;;
;;   program     ::= constant* vertex action+ application
;;   vertex      ::= name field+
;;   action      ::= name target param* predicate work diffuse-pred diffuse
;;   application ::= name metadata (binary-name, root-action, etc.)
;;
;; The AST flows through the pipeline:
;;   read-source → parse (produces cca-program)
;;                       → resolve (wraps in resolved-program with symbol table)
;;                       → typecheck (validates, returns resolved-program unchanged)
;;                       → emit-cpp (walks AST to generate C++ code)
;;
;; Inputs:  Constructed by parse-pass from S-expression datums.
;; Outputs: Consumed by resolve, typecheck, cost-model, and emit-cpp passes.
;; ═══════════════════════════════════════════════════════════════════════════════

;; ─── Source spans ─────────────────────────────────────────────────────────────
;; Tracks origin location for error reporting; not populated in Draft 0.1.
(struct source-span (file line col) #:transparent)

(define no-span (source-span #f #f #f))

;; ─── Types ────────────────────────────────────────────────────────────────────
;; CCA's type system mirrors the simulator's data model.
(struct t-unit () #:transparent)           ; void / no value (unused in practice)
(struct t-bool () #:transparent)           ; Boolean guard conditions
(struct t-u32 () #:transparent)            ; 32-bit unsigned integer (vertex data, counters)
(struct t-address () #:transparent)        ; CCA network address (compute cell location)
(struct t-edge () #:transparent)           ; one edge in the adjacency list
(struct t-pointer (vertex-name) #:transparent)  ; typed pointer to a vertex struct instance
(struct t-vector (element-type) #:transparent)  ; homogeneous collection (future use)

;; ─── Expressions ──────────────────────────────────────────────────────────────
;; Expressions are pure (no side effects) and appear in predicates, RHS of
;; assignments, and propagate arguments.
(struct expr (span) #:transparent)
(struct literal-expr expr (value type) #:transparent)   ; compile-time constant (number or bool)
(struct var-expr expr (name) #:transparent)             ; reference to param, local, or constant
(struct prim-expr expr (op args) #:transparent)         ; primitive op: arithmetic, comparison, logic
(struct vertex-read-expr expr (field target) #:transparent)  ; read a field from the target vertex
(struct edge-read-expr expr (accessor target) #:transparent) ; accessor: 'address or 'weight
(struct let-expr expr (bindings body) #:transparent)    ; local binding in expression context
(struct if-expr expr (test then else-branch) #:transparent)  ; conditional expression (ternary)

;; ─── Statements ───────────────────────────────────────────────────────────────
;; Statements produce effects: mutation (work phase) or propagation (diffuse phase).
(struct stmt (span) #:transparent)
(struct set-field-stmt stmt (field target value) #:transparent)    ; write to a vertex field
(struct for-edges-stmt stmt (edge-var target body) #:transparent)  ; iterate over adjacency list
(struct propagate-stmt stmt (action-name destination args) #:transparent) ; send an action message
(struct let-stmt stmt (bindings body) #:transparent)     ; local binding in statement context
(struct if-stmt stmt (test then-stmts else-stmts) #:transparent)  ; conditional statements
(struct begin-stmt stmt (stmts) #:transparent)           ; sequential statement block

;; ─── Declarations ─────────────────────────────────────────────────────────────
;; Top-level program declarations corresponding to define-constant, define-vertex,
;; define-action, and define-application forms.

(struct constant-decl (name type value span) #:transparent)

;; field-decl: one field in a vertex struct.
;;   annotations: reserved for future cost hints or layout directives.
(struct field-decl (name type initial mutable? annotations span) #:transparent)
(struct vertex-decl (name fields span) #:transparent)

(struct param-decl (name type span) #:transparent)

;; action-decl: the four-phase action structure of the CCA execution model.
;;   target      : the vertex pointer parameter (first param)
;;   params      : payload parameters carried by the action message
;;   predicate   : Boolean expr — guards whether this action fires
;;   work        : list of stmts — local computation (may mutate vertex)
;;   diffuse-predicate : Boolean expr — guards whether diffuse propagates
;;   diffuse     : list of stmts — propagation to neighbors (no mutation)
(struct action-decl (name target params predicate work diffuse-predicate diffuse span)
  #:transparent)

;; application-decl: top-level program metadata for code generation.
;;   binary-name    : output executable name for CMake
;;   vertex-type    : which vertex struct this application uses
;;   root-action    : the action to germinate at startup
;;   root-arguments : literal values passed to the root germination
;;   result-field   : which vertex field holds the final answer (for verification)
;;   verification   : verification strategy symbol or #f
(struct application-decl
  (name binary-name vertex-type root-action root-arguments result-field verification span)
  #:transparent)

;; ─── Program ──────────────────────────────────────────────────────────────────
;; The top-level container produced by the parser.
(struct cca-program (constants vertex actions application span) #:transparent)

;; ─── Host-level forms (inside define-program) ─────────────────────────────────
;; These represent simulator API calls in the driver program.
;; Each translates to a specific C++ statement or block in main().

(struct host-create-simulator (options) #:transparent)
;; options: hash of #:shape, #:dim-x, #:dim-y, #:memory-per-cc, etc.

(struct host-load-graph (options) #:transparent)
;; options: hash of #:file, #:name, #:vertex-type, #:weighted

(struct host-register-actions (action-names) #:transparent)
;; action-names: list of symbols naming actions to register

(struct host-germinate (action-name options) #:transparent)
;; options: hash of #:root, #:arguments, #:shuffle

(struct host-run () #:transparent)
;; No options — just calls run_simulation

(struct host-write-results (options) #:transparent)
;; options: hash of #:output-dir, #:trail

(struct host-when-verify (verify-options) #:transparent)
;; options: hash of #:field, #:extension

(struct host-let (bindings body) #:transparent)
;; bindings: list of (name . cli-arg-ref/cli-flag-ref) pairs
;; body: list of host forms that can use the bound names

;; cli-arg and cli-flag are represented as option values in the host forms
(struct cli-arg-ref (name long-name type default required? description) #:transparent)
(struct cli-flag-ref (name long-name description) #:transparent)

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
         "ast.rkt")

(provide (struct-out cost-model)
         default-cost-model
         load-cost-model
         compute-phase-cpi)

;; ─── Cost model structure ─────────────────────────────────────────────────────
;; Each entry maps an operation kind to a cycle cost (non-negative integer).

(struct cost-model
  (field-read       ; reading a vertex field
   field-write      ; writing a vertex field
   comparison       ; <, >, <=, >=, =
   arithmetic       ; +, -, *
   boolean-op       ; and, or, not
   literal-access   ; reading a constant or literal
   variable-access  ; reading a local variable or parameter
   edge-read        ; reading edge address or weight
   propagate        ; creating and sending an action (network, not compute)
   )
  #:transparent)

;; ─── Default cost model ───────────────────────────────────────────────────────
;; Each basic operation costs 1 cycle. Propagate costs 0 because it's network
;; work handled by the diffuse mechanism, not compute cell ALU time.
;; Literal/variable access costs 0 — they're register reads, not memory ops.

(define default-cost-model
  (cost-model
   1    ; field-read: memory load from vertex object
   1    ; field-write: memory store to vertex object
   1    ; comparison
   1    ; arithmetic
   1    ; boolean-op
   0    ; literal-access: immediate value
   0    ; variable-access: register/stack
   1    ; edge-read: memory load from edge array
   0    ; propagate: network operation, not ALU
   ))

;; ─── Load a user-provided cost model from a file ──────────────────────────────
;; File format: S-expression with keyword-value pairs
;;
;; Example cost-model.rkt:
;;   ((field-read 2)
;;    (field-write 2)
;;    (comparison 1)
;;    (arithmetic 1)
;;    (boolean-op 1)
;;    (literal-access 0)
;;    (variable-access 0)
;;    (edge-read 2)
;;    (propagate 0))

(define (load-cost-model path)
  (define entries (with-input-from-file path read))
  (define (get key [default 0])
    (define pair (assoc key entries))
    (if pair (cadr pair) default))
  (cost-model
   (get 'field-read 1)
   (get 'field-write 1)
   (get 'comparison 1)
   (get 'arithmetic 1)
   (get 'boolean-op 1)
   (get 'literal-access 0)
   (get 'variable-access 0)
   (get 'edge-read 1)
   (get 'propagate 0)))

;; ─── Compute CPI for a phase ──────────────────────────────────────────────────
;; Walks the AST of a phase (expression or list of statements) and sums the
;; cost of each operation according to the cost model.
;;
;; For predicates: count expression cost
;; For work: count statement costs
;; For diffuse-predicate: count expression cost
;; For diffuse: 0 (diffuse does not call apply_CPI in the simulator)

(define (compute-phase-cpi phase-ast model #:kind kind)
  (case kind
    [(predicate diffuse-predicate)
     (count-expr-cost phase-ast model)]
    [(work)
     (count-stmts-cost phase-ast model)]
    [(diffuse)
     ;; Diffuse phase does NOT call apply_CPI — it's network work
     0]))

;; ─── Expression cost counter ──────────────────────────────────────────────────
(define (count-expr-cost e model)
  (match e
    [(literal-expr _ _ _)
     (cost-model-literal-access model)]
    [(var-expr _ _)
     (cost-model-variable-access model)]
    [(prim-expr _ op args)
     (define op-cost
       (cond
         [(member op '(> >= < <= =)) (cost-model-comparison model)]
         [(member op '(+ - *)) (cost-model-arithmetic model)]
         [(member op '(and or not)) (cost-model-boolean-op model)]
         [else 1]))
     (+ op-cost (apply + (map (λ (a) (count-expr-cost a model)) args)))]
    [(vertex-read-expr _ _ target)
     (+ (cost-model-field-read model)
        (count-expr-cost target model))]
    [(edge-read-expr _ _ target)
     (+ (cost-model-edge-read model)
        (count-expr-cost target model))]
    [(let-expr _ bindings body)
     (+ (apply + (map (λ (b) (count-expr-cost (cdr b) model)) bindings))
        (count-expr-cost body model))]
    [(if-expr _ test then else-branch)
     ;; Worst case: test + max(then, else)
     (+ (count-expr-cost test model)
        (max (count-expr-cost then model)
             (count-expr-cost else-branch model)))]
    [_ 0]))

;; ─── Statement cost counter ──────────────────────────────────────────────────
(define (count-stmts-cost stmts model)
  (apply + (map (λ (s) (count-stmt-cost s model)) stmts)))

(define (count-stmt-cost s model)
  (match s
    [(set-field-stmt _ _ target value)
     (+ (cost-model-field-write model)
        (count-expr-cost value model))]
    [(for-edges-stmt _ _ target body)
     ;; Per-iteration cost; the actual iteration count is runtime-dependent.
     ;; We report the per-edge body cost as the CPI since apply_CPI is called
     ;; once before the loop, not per iteration.
     (count-stmts-cost body model)]
    [(propagate-stmt _ _ dest args)
     (+ (cost-model-propagate model)
        (count-expr-cost dest model)
        (apply + (map (λ (a) (count-expr-cost a model)) args)))]
    [(let-stmt _ bindings body)
     (+ (apply + (map (λ (b) (count-expr-cost (cdr b) model)) bindings))
        (count-stmts-cost body model))]
    [(if-stmt _ test then-stmts else-stmts)
     (+ (count-expr-cost test model)
        (max (count-stmts-cost then-stmts model)
             (count-stmts-cost else-stmts model)))]
    [(begin-stmt _ stmts)
     (count-stmts-cost stmts model)]
    [_ 0]))

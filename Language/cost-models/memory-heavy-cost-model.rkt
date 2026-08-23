;; ═══════════════════════════════════════════════════════════════════════════════
;; Memory-heavy cost model for CCA Compiler.
;;
;; This model represents an architecture where memory operations (field reads,
;; field writes, edge reads) are significantly more expensive than ALU operations.
;; Useful for modeling systems with high memory latency relative to compute.
;;
;; Used via: racket Language/main.rkt --cost-model Language/cost-models/memory-heavy-cost-model.rkt
;; Format: S-expression association list of (operation-name cycles) pairs.
;; ═══════════════════════════════════════════════════════════════════════════════

;; Example custom cost model for CCA Compiler
;; Each entry: (operation-name cost-in-cycles)
;;
;; This model assumes a hypothetical architecture where memory
;; operations cost 2 cycles and ALU operations cost 1 cycle.

((field-read 2)
 (field-write 2)
 (comparison 1)
 (arithmetic 1)
 (boolean-op 1)
 (literal-access 0)
 (variable-access 0)
 (edge-read 2)
 (propagate 0))

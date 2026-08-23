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

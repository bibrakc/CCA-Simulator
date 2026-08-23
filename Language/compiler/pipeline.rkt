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
         racket/pretty)

(provide run-pipeline
         pipeline-success?
         pipeline-diagnostics
         pipeline-ir
         display-diagnostics
         pretty-print-ir
         current-cost-model)

;; ═══════════════════════════════════════════════════════════════════════════════
;; Compiler pipeline orchestrator.
;;
;; This module sequences the compiler passes and handles error propagation.
;; The pass order is:  read-source → parse → resolve → typecheck → [emit-cpp]
;;
;; Design:
;;   - Passes are stored in `all-passes` as (name . procedure) pairs.
;;   - Each pass takes the previous pass's output (IR) and returns new IR.
;;   - If a pass raises exn:fail, the pipeline stops and packages the error
;;     as a diagnostic in the pipeline-result.
;;   - emit-cpp is special: it needs an output directory, so it runs outside
;;     the standard pass loop when --output is provided.
;;   - The #:through argument allows stopping early (e.g., for --check or --dump-ir).
;;
;; Inputs:  Source file path + options (#:through, #:output, #:cost-model-path).
;; Outputs: pipeline-result struct (success flag, final IR, diagnostics list).
;; ═══════════════════════════════════════════════════════════════════════════════

;; ─── Pipeline result ──────────────────────────────────────────────────────────
(struct pipeline-result (success? ir diagnostics) #:transparent)

(define (pipeline-success? r) (pipeline-result-success? r))
(define (pipeline-diagnostics r) (pipeline-result-diagnostics r))
(define (pipeline-ir r) (pipeline-result-ir r))

;; ─── Diagnostics ──────────────────────────────────────────────────────────────
(struct diagnostic (level file line col phase message) #:transparent)

(define (display-diagnostics diags)
  (for ([d (in-list diags)])
    (match-define (diagnostic level file line col phase msg) d)
    (eprintf "~a:~a:~a [~a] ~a\n"
             (or file "<unknown>") (or line "?") (or col "?")
             phase msg)))

(define (pretty-print-ir ir)
  (pretty-print ir))

;; ─── Pass registry ────────────────────────────────────────────────────────────
;; Ordered list of (name . procedure) pairs. The pipeline runs them sequentially,
;; threading the output of each pass as input to the next.
;; emit-cpp is NOT in this list because it requires an output directory argument
;; and is handled as a post-processing step.

(require "frontend/read-source.rkt"
         "frontend/parse.rkt"
         "frontend/resolve.rkt"
         "frontend/typecheck.rkt"
         "cost-model.rkt"
         "backend/emit-cpp.rkt")

(define all-passes
  `((read-source    . ,read-source-pass)
    (parse          . ,parse-pass)
    (resolve        . ,resolve-pass)
    (typecheck      . ,typecheck-pass)
    ;; emit-cpp is special — it needs an output dir, handled below
    ))

;; ─── Run pipeline ─────────────────────────────────────────────────────────────
(define (run-pipeline source-path
                      #:through [through-pass #f]
                      #:output [output-dir #f]
                      #:cost-model-path [cost-model-path #f])
  ;; Set cost model if user provided one
  (when cost-model-path
    (current-cost-model (load-cost-model cost-model-path)))

  ;; Determine which passes to run: all passes up to and including `through-pass`,
  ;; or all passes if through-pass is #f.
  (define passes-to-run
    (if through-pass
        (let loop ([ps all-passes] [acc '()])
          (cond
            [(null? ps) (reverse acc)]  ; through-pass not found → run all collected
            [(eq? (caar ps) through-pass)
             (reverse (cons (car ps) acc))]  ; include the target pass, then stop
            [else (loop (cdr ps) (cons (car ps) acc))]))
        all-passes))

  ;; Execute passes sequentially, threading the IR through each.
  ;; On failure, short-circuit and return diagnostics.
  (let loop ([remaining passes-to-run]
             [ir source-path]        ; initial IR is just the file path string
             [diags '()])
    (cond
      [(null? remaining)
       ;; All passes ran successfully. If output-dir specified, emit C++ as final step.
       (if output-dir
           (with-handlers ([exn:fail?
                            (λ (e)
                              (pipeline-result #f ir
                                (append diags
                                  (list (diagnostic 'error #f #f #f 'emit-cpp
                                                    (exn-message e))))))])
             (define files (emit-cpp-pass (list ir output-dir)))
             (pipeline-result #t files diags))
           (pipeline-result #t ir diags))]
      [else
       (define pass-name (caar remaining))
       (define pass-fn (cdar remaining))
       ;; Wrap each pass in an exception handler to capture failures as diagnostics
       (define result
         (with-handlers ([exn:fail?
                          (λ (e)
                            (list 'error
                                  (diagnostic 'error #f #f #f pass-name
                                              (exn-message e))))])
           (list 'ok (pass-fn ir))))
       (match result
         [(list 'ok new-ir)
          (loop (cdr remaining) new-ir diags)]
         [(list 'error diag)
          (pipeline-result #f ir (append diags (list diag)))])])))

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
         pretty-print-ir)

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
;; Each pass: (name . procedure)
;; We will add passes incrementally. For now just read-source and parse.

(require "frontend/read-source.rkt"
         "frontend/parse.rkt"
         "frontend/resolve.rkt"
         "frontend/typecheck.rkt"
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
                      #:output [output-dir #f])
  (define passes-to-run
    (if through-pass
        (let loop ([ps all-passes] [acc '()])
          (cond
            [(null? ps) (reverse acc)]
            [(eq? (caar ps) through-pass)
             (reverse (cons (car ps) acc))]
            [else (loop (cdr ps) (cons (car ps) acc))]))
        all-passes))

  (let loop ([remaining passes-to-run]
             [ir source-path]
             [diags '()])
    (cond
      [(null? remaining)
       ;; If output-dir is specified and we ran all passes, emit C++
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

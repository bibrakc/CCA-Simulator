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



(require rackunit
         rackunit/text-ui
         racket/list
         "../compiler/frontend/read-source.rkt"
         "../compiler/frontend/parse.rkt"
         "../compiler/ast.rkt")

;; ─── Reader tests ─────────────────────────────────────────────────────────────
(define reader-tests
  (test-suite
   "read-source"

   (test-case "reads BFS example with 4 top-level forms"
     (define forms (read-source-pass "Language/examples/bfs/bfs.cca"))
     (check-equal? (length forms) 4)
     (check-equal? (car (first forms)) 'define-constant)
     (check-equal? (car (second forms)) 'define-vertex)
     (check-equal? (car (third forms)) 'define-action)
     (check-equal? (car (fourth forms)) 'define-application))

   (test-case "strips #lang cca line"
     (define forms (read-source-pass "Language/examples/bfs/bfs.cca"))
     ;; If #lang was not stripped, it would fail to read
     (check-pred list? forms))

   (test-case "errors on missing file"
     (check-exn exn:fail?
                (λ () (read-source-pass "nonexistent.cca"))))))

;; ─── Parser tests ─────────────────────────────────────────────────────────────
(define parser-tests
  (test-suite
   "parse"

   (test-case "parses BFS into cca-program"
     (define forms (read-source-pass "Language/examples/bfs/bfs.cca"))
     (define prog (parse-pass forms))
     (check-pred cca-program? prog))

   (test-case "parses constant"
     (define forms (read-source-pass "Language/examples/bfs/bfs.cca"))
     (define prog (parse-pass forms))
     (define constants (cca-program-constants prog))
     (check-equal? (length constants) 1)
     (define c (car constants))
     (check-equal? (constant-decl-name c) 'max-level)
     (check-pred t-u32? (constant-decl-type c))
     (check-equal? (constant-decl-value c) 999999))

   (test-case "parses vertex with mutable field"
     (define forms (read-source-pass "Language/examples/bfs/bfs.cca"))
     (define prog (parse-pass forms))
     (define v (cca-program-vertex prog))
     (check-equal? (vertex-decl-name v) 'BFSVertex)
     (define fields (vertex-decl-fields v))
     (check-equal? (length fields) 1)
     (define f (car fields))
     (check-equal? (field-decl-name f) 'level)
     (check-pred t-u32? (field-decl-type f))
     (check-equal? (field-decl-initial f) 'max-level)
     (check-true (field-decl-mutable? f)))

   (test-case "parses action target and payload"
     (define forms (read-source-pass "Language/examples/bfs/bfs.cca"))
     (define prog (parse-pass forms))
     (define act (car (cca-program-actions prog)))
     (check-equal? (action-decl-name act) 'bfs-action)
     ;; Target
     (define target (action-decl-target act))
     (check-equal? (param-decl-name target) 'v)
     (check-pred t-pointer? (param-decl-type target))
     (check-equal? (t-pointer-vertex-name (param-decl-type target)) 'BFSVertex)
     ;; Payload
     (define params (action-decl-params act))
     (check-equal? (length params) 1)
     (check-equal? (param-decl-name (car params)) 'incoming-level)
     (check-pred t-u32? (param-decl-type (car params))))

   (test-case "parses predicate as comparison"
     (define forms (read-source-pass "Language/examples/bfs/bfs.cca"))
     (define prog (parse-pass forms))
     (define act (car (cca-program-actions prog)))
     (define pred (action-decl-predicate act))
     (check-pred prim-expr? pred)
     (check-equal? (prim-expr-op pred) '>)
     (check-equal? (length (prim-expr-args pred)) 2))

   (test-case "parses work as set-field"
     (define forms (read-source-pass "Language/examples/bfs/bfs.cca"))
     (define prog (parse-pass forms))
     (define act (car (cca-program-actions prog)))
     (define work (action-decl-work act))
     (check-equal? (length work) 1)
     (check-pred set-field-stmt? (car work))
     (check-equal? (set-field-stmt-field (car work)) 'level))

   (test-case "parses diffuse predicate as equality"
     (define forms (read-source-pass "Language/examples/bfs/bfs.cca"))
     (define prog (parse-pass forms))
     (define act (car (cca-program-actions prog)))
     (define dpred (action-decl-diffuse-predicate act))
     (check-pred prim-expr? dpred)
     (check-equal? (prim-expr-op dpred) '=))

   (test-case "parses diffuse as for-each with propagate"
     (define forms (read-source-pass "Language/examples/bfs/bfs.cca"))
     (define prog (parse-pass forms))
     (define act (car (cca-program-actions prog)))
     (define diffuse (action-decl-diffuse act))
     (check-equal? (length diffuse) 1)
     (check-pred for-edges-stmt? (car diffuse))
     (define body (for-edges-stmt-body (car diffuse)))
     (check-equal? (length body) 1)
     (check-pred propagate-stmt? (car body))
     (check-equal? (propagate-stmt-action-name (car body)) 'bfs-action))

   (test-case "parses application metadata"
     (define forms (read-source-pass "Language/examples/bfs/bfs.cca"))
     (define prog (parse-pass forms))
     (define app (cca-program-application prog))
     (check-equal? (application-decl-name app) 'BFS)
     (check-equal? (application-decl-binary-name app) "BFS_Generated_CCASimulator")
     (check-equal? (application-decl-vertex-type app) 'BFSVertex)
     (check-equal? (application-decl-root-action app) 'bfs-action)
     (check-equal? (application-decl-result-field app) 'level)
     (check-equal? (application-decl-verification app) 'bfs-level-file))

   (test-case "rejects missing vertex"
     (check-exn exn:fail?
                (λ () (parse-pass '((define-action foo ([x : UInt32]) (predicate #t (work) (diffuse (predicate #t)))))))))

   (test-case "rejects unknown top-level form"
     (check-exn exn:fail?
                (λ () (parse-pass '((define-mystery foo))))))
   ))

;; ─── Run ──────────────────────────────────────────────────────────────────────
(module+ main
  (define result
    (run-tests
     (test-suite "CCA Compiler Tests"
                 reader-tests
                 parser-tests)))
  (exit (if (zero? result) 0 1)))

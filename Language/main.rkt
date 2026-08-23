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



;; ═══════════════════════════════════════════════════════════════════════════════
;; CCA Compiler CLI — command-line entry point.
;;
;; This module provides the user-facing CLI for the CCA-to-C++ compiler.
;; It supports three modes:
;;   --check    : Parse + resolve + typecheck only (no code generation)
;;   --compile  : Full pipeline through C++ emission (requires --output)
;;   --dump-ir  : Run pipeline up to a named pass, then pretty-print the IR
;;
;; Inputs:  A .cca source file path and optional flags.
;; Outputs: Either diagnostic messages (on error), generated C++ files (--compile),
;;          or IR dump (--dump-ir).
;;
;; Pipeline: read-source → parse → resolve → typecheck → emit-cpp
;; ═══════════════════════════════════════════════════════════════════════════════

(require racket/cmdline
         racket/path
         "compiler/pipeline.rkt")

(define current-command (make-parameter #f))
(define current-output-dir (make-parameter #f))
(define current-dump-after (make-parameter #f))
(define current-cost-model-path (make-parameter #f))

;; Runs the pipeline through typecheck only, reporting success or diagnostics.
(define (run-check source-path)
  (printf "Checking ~a...\n" source-path)
  (define result (run-pipeline source-path #:through 'typecheck
                               #:cost-model-path (current-cost-model-path)))
  (if (pipeline-success? result)
      (printf "~a: OK\n" source-path)
      (begin
        (display-diagnostics (pipeline-diagnostics result))
        (exit 1))))

;; Runs the full pipeline and writes generated C++ to output-dir.
(define (run-compile source-path output-dir)
  (printf "Compiling ~a → ~a\n" source-path output-dir)
  (define result (run-pipeline source-path #:output output-dir
                               #:cost-model-path (current-cost-model-path)))
  (if (pipeline-success? result)
      (printf "Generated: ~a\n" output-dir)
      (begin
        (display-diagnostics (pipeline-diagnostics result))
        (exit 1))))

;; Runs the pipeline up to a named pass and pretty-prints the resulting IR.
(define (run-dump-ir source-path after-pass)
  (printf "Dumping IR after ~a for ~a\n" after-pass source-path)
  (define result (run-pipeline source-path #:through after-pass
                               #:cost-model-path (current-cost-model-path)))
  (if (pipeline-success? result)
      (pretty-print-ir (pipeline-ir result))
      (begin
        (display-diagnostics (pipeline-diagnostics result))
        (exit 1))))

(module+ main
  (define source-file
    (command-line
     #:program "cca-compiler"
     #:once-any
     ["--check" "Type-check without emitting files"
      (current-command 'check)]
     ["--compile" "Compile to C++"
      (current-command 'compile)]
     ["--dump-ir" after "Dump IR after named pass"
      (current-command 'dump-ir)
      (current-dump-after (string->symbol after))]
     #:once-each
     ["--output" dir "Output directory for generated files"
      (current-output-dir dir)]
     ["--cost-model" path "Path to a custom cost model file"
      (current-cost-model-path path)]
     #:args (source-file)
     source-file))

  (unless (current-command)
    (eprintf "Error: specify --check, --compile, or --dump-ir\n")
    (exit 1))

  (case (current-command)
    [(check) (run-check source-file)]
    [(compile)
     (unless (current-output-dir)
       (eprintf "Error: --compile requires --output\n")
       (exit 1))
     (run-compile source-file (current-output-dir))]
    [(dump-ir)
     (run-dump-ir source-file (current-dump-after))]))

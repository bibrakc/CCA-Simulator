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

;; ─── #lang cca reader ─────────────────────────────────────────────────────────
;;
;; Implements the reader for `#lang cca` source files.
;; When Racket encounters `#lang cca`, it delegates to this module.
;;
;; The reader:
;;   1. Reads all top-level S-expression forms as data (no evaluation)
;;   2. Wraps them in a module that runs the CCA compiler pipeline
;;
;; This module is the integration point between Racket's module system and the
;; CCA compiler. It enables .cca files to be loaded with `racket -S Language file.cca`,
;; which will parse and typecheck the program.
;;
;; For full compilation to C++, use the CLI instead:
;;   racket Language/main.rkt --compile --output <dir> <file.cca>
;;
;; Inputs:  A port (from Racket's reader protocol) containing CCA source.
;; Outputs: A syntax object representing a Racket module that runs the pipeline.
;; ═══════════════════════════════════════════════════════════════════════════════

(require syntax/strip-context)

(provide (rename-out [cca-read read]
                     [cca-read-syntax read-syntax]))

;; Datum-based read — required by the reader protocol but delegates to read-syntax
(define (cca-read in)
  (syntax->datum (cca-read-syntax #f in)))

;; Syntax-based read — the main entry point for `#lang cca`.
;; Reads all forms from the port, then wraps them in a Racket module that
;; dynamically requires the compiler passes and runs them on the quoted forms.
(define (cca-read-syntax src in)
  ;; Read all top-level datums from the port
  (define forms
    (let loop ([acc '()])
      (define datum (read in))
      (if (eof-object? datum)
          (reverse acc)
          (loop (cons datum acc)))))

  ;; Wrap in a module that parses the quoted forms through the compiler.
  ;; Uses dynamic-require so the reader doesn't need compile-time deps on the compiler.
  (strip-context
   (datum->syntax
    #f
    `(module cca-program racket/base
       (require racket/pretty)

       ;; The source forms as quoted data
       (define source-forms ',forms)

       ;; Dynamically load the compiler passes
       (define parse-pass
         (dynamic-require "Language/compiler/frontend/parse.rkt" 'parse-pass))
       (define resolve-pass
         (dynamic-require "Language/compiler/frontend/resolve.rkt" 'resolve-pass))
       (define typecheck-pass
         (dynamic-require "Language/compiler/frontend/typecheck.rkt" 'typecheck-pass))

       ;; Access AST struct accessors
       (define cca-program-vertex
         (dynamic-require "Language/compiler/ast.rkt" 'cca-program-vertex))
       (define cca-program-actions
         (dynamic-require "Language/compiler/ast.rkt" 'cca-program-actions))
       (define cca-program-application
         (dynamic-require "Language/compiler/ast.rkt" 'cca-program-application))
       (define vertex-decl-name
         (dynamic-require "Language/compiler/ast.rkt" 'vertex-decl-name))
       (define action-decl-name
         (dynamic-require "Language/compiler/ast.rkt" 'action-decl-name))
       (define application-decl-name
         (dynamic-require "Language/compiler/ast.rkt" 'application-decl-name))

       ;; Run the pipeline
       (define parsed (parse-pass source-forms))
       (define resolved (resolve-pass parsed))
       (define checked (typecheck-pass resolved))

       (printf "CCA Program OK\n")
       (printf "  Vertex: ~a\n" (vertex-decl-name (cca-program-vertex parsed)))
       (printf "  Actions: ~a\n" (map action-decl-name (cca-program-actions parsed)))
       (printf "  Application: ~a\n" (application-decl-name (cca-program-application parsed)))))))

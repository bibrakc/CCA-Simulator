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



(require racket/port
         racket/file)

(provide read-source-pass)

;; ═══════════════════════════════════════════════════════════════════════════════
;; Read-source pass — first stage of the CCA compiler pipeline.
;;
;; This pass converts a .cca file on disk into a list of S-expression datums
;; that subsequent passes can pattern-match against. It does NOT evaluate
;; any code — it reads the file purely as data.
;;
;; Key behavior:
;;   - Strips the `#lang cca` line if present (Racket's reader would choke on it)
;;   - Reads all remaining top-level forms using Racket's `read`
;;   - Errors if the file doesn't exist or contains no forms
;;
;; Inputs:  File path (string or path object).
;; Outputs: List of S-expression datums ready for the parse pass.
;; ═══════════════════════════════════════════════════════════════════════════════

;; ─── Read source pass ─────────────────────────────────────────────────────────
;; Input: file path (string or path)
;; Output: list of S-expressions (datums) read from the file
;;
;; This pass reads CCA source as pure data (no eval). It strips the
;; #lang cca line if present and reads all remaining top-level forms.

(define (read-source-pass source-path)
  (define path (if (string? source-path) (string->path source-path) source-path))
  (unless (file-exists? path)
    (error 'read-source "file not found: ~a" source-path))

  (define content (file->string path))

  ;; Strip #lang line if present
  (define stripped
    (cond
      [(regexp-match #rx"^#lang [^\n]*\n" content)
       => (λ (m) (substring content (string-length (car m))))]
      [else content]))

  ;; Read all top-level datums
  (define forms
    (with-input-from-string stripped
      (λ ()
        (let loop ([acc '()])
          (define v (read))
          (if (eof-object? v)
              (reverse acc)
              (loop (cons v acc)))))))

  (when (null? forms)
    (error 'read-source "no declarations found in ~a" source-path))

  forms)

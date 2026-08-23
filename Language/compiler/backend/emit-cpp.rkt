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
         racket/string
         racket/format
         racket/port
         racket/file
         "../ast.rkt"
         "../cost-model.rkt"
         "../frontend/resolve.rkt"
         "cpp/cpp-ir.rkt")

(provide emit-cpp-pass
         current-cost-model)

;; ═══════════════════════════════════════════════════════════════════════════════
;; Cost model parameter — defaults to the built-in model, can be overridden
;; ═══════════════════════════════════════════════════════════════════════════════

(define current-cost-model (make-parameter default-cost-model))

;; ═══════════════════════════════════════════════════════════════════════════════
;; Main entry: emit all generated files
;; ═══════════════════════════════════════════════════════════════════════════════

(define (emit-cpp-pass resolved+output)
  (match-define (list resolved output-dir) resolved+output)
  (define vtx (resolved-program-vertex resolved))
  (define actions (resolved-program-actions resolved))
  (define app (resolved-program-application resolved))
  (define constants (resolved-program-constants resolved))
  (define symbols (resolved-program-symbols resolved))

  ;; Derive names
  (define app-name-lower (string-downcase (symbol->string (application-decl-name app))))
  (define hpp-filename (format "cca_~a_generated.hpp" app-name-lower))
  (define cpp-filename (format "cca_~a_generated.cpp" app-name-lower))
  (define binary-name (application-decl-binary-name app))

  ;; Generate IR and render to strings
  (define hpp-ir (generate-hpp-ir vtx actions app constants symbols))
  (define cpp-ir (generate-cpp-ir vtx actions app constants symbols hpp-filename))
  (define hpp-content (cpp-ir->string hpp-ir))
  (define cpp-content (cpp-ir->string cpp-ir))
  (define cmake-content (generate-cmake binary-name cpp-filename))

  ;; Write files
  (make-directory* output-dir)
  (write-file (build-path output-dir hpp-filename) hpp-content)
  (write-file (build-path output-dir cpp-filename) cpp-content)
  (write-file (build-path output-dir "CMakeLists.txt") cmake-content)

  (list hpp-filename cpp-filename "CMakeLists.txt"))

(define (write-file path content)
  (call-with-output-file path
    (λ (out) (display content out))
    #:exists 'replace))

(define (string-downcase s) (string-foldcase s))

;; ═══════════════════════════════════════════════════════════════════════════════
;; C++ Expression Generation (FULLY RECURSIVE with context)
;; ═══════════════════════════════════════════════════════════════════════════════

;; context: hash mapping CCA variable names (symbols) -> cpp expression strings
;; target-var: the name of the target (vertex) parameter
;; The context maps payload param names to their C++ access expressions.

(define (cpp-expr-from-ast e ctx)
  (match e
    [(literal-expr _ val _)
     (format "~a" val)]

    [(var-expr _ name)
     (define mangled (mangle-cpp name))
     (if (hash-has-key? ctx name)
         (hash-ref ctx name)
         mangled)]

    [(prim-expr _ op args)
     (cond
       [(= (length args) 1)
        (format "(~a~a)" (cpp-op op) (cpp-expr-from-ast (car args) ctx))]
       [(= (length args) 2)
        (format "(~a ~a ~a)"
                (cpp-expr-from-ast (car args) ctx)
                (cpp-op op)
                (cpp-expr-from-ast (cadr args) ctx))]
       [else
        ;; n-ary: fold left
        (define exprs (map (λ (a) (cpp-expr-from-ast a ctx)) args))
        (let fold ([remaining (cdr exprs)] [acc (car exprs)])
          (if (null? remaining)
              acc
              (fold (cdr remaining)
                    (format "(~a ~a ~a)" acc (cpp-op op) (car remaining)))))])]

    [(vertex-read-expr _ field target)
     (format "v->~a" (mangle-cpp field))]

    [(edge-read-expr _ 'address _) "v->edges[i].edge"]
    [(edge-read-expr _ 'weight _) "v->edges[i].weight"]

    [(let-expr _ bindings body)
     ;; In C++ expression context, let is tricky. For now support single binding.
     ;; This would need statement-level let in real use; for expression context
     ;; we inline the binding.
     (define new-ctx
       (for/fold ([c ctx]) ([b (in-list bindings)])
         (hash-set c (car b) (cpp-expr-from-ast (cdr b) ctx))))
     (cpp-expr-from-ast body new-ctx)]

    [(if-expr _ test then else-branch)
     (format "(~a ? ~a : ~a)"
             (cpp-expr-from-ast test ctx)
             (cpp-expr-from-ast then ctx)
             (cpp-expr-from-ast else-branch ctx))]

    [_ (error 'cpp-expr-from-ast "unsupported expr: ~s" e)]))

(define (cpp-op op)
  (case op
    [(+) "+"] [(-) "-"] [(*) "*"] [(/) "/"] [(%) "%"]
    [(>) ">"] [(>=) ">="] [(<) "<"] [(<=) "<="]
    [(=) "=="] [(!=) "!="]
    [(and) "&&"] [(or) "||"] [(not) "!"]
    [else (error 'cpp-op "unknown op: ~a" op)]))

(define (mangle-cpp sym)
  (string-replace (string-replace (symbol->string sym) "-" "_") "!" ""))

;; ═══════════════════════════════════════════════════════════════════════════════
;; Type helpers
;; ═══════════════════════════════════════════════════════════════════════════════

(define (cca-type->cpp t)
  (match t
    [(t-u32) "u_int32_t"]
    [(t-bool) "bool"]
    [(t-address) "Address"]
    [_ "u_int32_t"]))

;; ═══════════════════════════════════════════════════════════════════════════════
;; HPP IR Generation
;; ═══════════════════════════════════════════════════════════════════════════════

(define (generate-hpp-ir vtx actions app constants symbols)
  (define vtx-name (symbol->string (vertex-decl-name vtx)))
  (define guard-name (format "CCA_~a_GENERATED_HPP"
                             (string-upcase (symbol->string (application-decl-name app)))))

  (cpp-file
   (append
    ;; Header comment
    (list (cpp-raw-decl "// Auto-generated by CCA Compiler v0.1.0\n// Do not edit — regenerate from .cca source.\n"))

    ;; Include guard + body
    (list
     (cpp-ifndef-guard
      guard-name
      (append
       ;; Includes
       (list (cpp-include "CCASimulator.hpp" #f)
             (cpp-include "Enums.hpp" #f)
             (cpp-include "Graph.hpp" #f)
             (cpp-include "RecursiveParallelVertex.hpp" #f)
             (cpp-include "cmdparser.hpp" #f)
             (cpp-blank-line)
             (cpp-include "cstring" #t)
             (cpp-include "fstream" #t)
             (cpp-blank-line))

       ;; Constants
       (emit-constants-ir constants)
       (list (cpp-blank-line))

       ;; Vertex struct
       (emit-vertex-ir vtx constants)
       (list (cpp-blank-line))

       ;; For each action: extern events, payload struct, handlers
       (append*
        (for/list ([act (in-list actions)])
          (emit-action-ir act vtx constants symbols actions)))))))))

(define (emit-constants-ir constants)
  (for/list ([c (in-list constants)])
    (cpp-constexpr-var (cpp-type-raw (cca-type->cpp (constant-decl-type c)))
                       (mangle-cpp (constant-decl-name c))
                       (format "~a" (constant-decl-value c)))))

(define (emit-vertex-ir vtx constants)
  (define vtx-name (symbol->string (vertex-decl-name vtx)))
  (define fields (vertex-decl-fields vtx))
  (define field (car fields))
  (define field-name (mangle-cpp (field-decl-name field)))
  (define field-type (cca-type->cpp (field-decl-type field)))
  (define field-init
    (if (field-decl-initial field)
        (mangle-cpp (field-decl-initial field))
        "0"))

  (list
   (cpp-raw-decl
    (format "template<typename Vertex_T>\nstruct ~a : Vertex_T\n{\n    ~a ~a{~a};\n\n    ~a(u_int32_t id_in, u_int32_t total_number_of_vertices_in)\n        : ~a(~a)\n    {\n        this->id = id_in;\n        this->number_of_edges = 0;\n        this->total_number_of_vertices = total_number_of_vertices_in;\n    }\n\n    void configure_derived_class_LCOs() {}\n\n    ~a() {}\n    ~~~a() {}\n};\n"
            vtx-name
            field-type field-name field-init
            vtx-name
            field-name field-init
            vtx-name
            vtx-name))))

(define (emit-action-ir act vtx constants symbols all-actions)
  (define vtx-name (symbol->string (vertex-decl-name vtx)))
  (define act-name (mangle-cpp (action-decl-name act)))
  (define target-param-name (param-decl-name (action-decl-target act)))
  (define payload-params (action-decl-params act))

  ;; Vertex fields
  (define field (car (vertex-decl-fields vtx)))
  (define field-name (mangle-cpp (field-decl-name field)))
  (define field-type (cca-type->cpp (field-decl-type field)))
  (define field-init
    (if (field-decl-initial field)
        (mangle-cpp (field-decl-initial field))
        "0"))

  ;; Build standard context: payload params are accessed as local vars
  (define base-ctx
    (for/fold ([ctx (make-immutable-hash)])
              ([p (in-list payload-params)])
      (hash-set ctx (param-decl-name p) (mangle-cpp (param-decl-name p)))))
  ;; Target var maps to "v" in handlers that use it via vertex-read
  (define handler-ctx (hash-set base-ctx target-param-name "v"))

  ;; Diffuse context: payload params map to current_<field>
  (define diffuse-ctx
    (for/fold ([ctx (hash-set (make-immutable-hash) target-param-name "v")])
              ([p (in-list payload-params)])
      (hash-set ctx (param-decl-name p) (format "current_~a" field-name))))

  ;; Payload field info (for the struct)
  (define payload-field (mangle-cpp (param-decl-name (car payload-params))))

  ;; Expressions
  (define pred-expr (cpp-expr-from-ast (action-decl-predicate act) handler-ctx))
  (define dpred-expr (cpp-expr-from-ast (action-decl-diffuse-predicate act) handler-ctx))

  ;; Work
  (define work-stmt (car (action-decl-work act)))
  (define work-field (mangle-cpp (set-field-stmt-field work-stmt)))
  (define work-value (cpp-expr-from-ast (set-field-stmt-value work-stmt) handler-ctx))

  ;; Diffuse: propagate expression
  (define diff-body (car (action-decl-diffuse act)))  ; for-edges-stmt
  (define prop-stmt (car (for-edges-stmt-body diff-body)))  ; propagate-stmt
  (define prop-arg (car (propagate-stmt-args prop-stmt)))
  (define prop-arg-cpp (cpp-expr-from-ast prop-arg diffuse-ctx))

  ;; Determine which action's handlers the propagate references
  (define prop-action-name (mangle-cpp (propagate-stmt-action-name prop-stmt)))

  (append
   ;; Extern event declarations
   (list (cpp-raw-decl
          (format "extern CCAFunctionEvent ~a_predicate;\nextern CCAFunctionEvent ~a_work;\nextern CCAFunctionEvent ~a_diffuse_predicate;\nextern CCAFunctionEvent ~a_diffuse;\n\n"
                  act-name act-name act-name act-name)))

   ;; Payload struct (ONLY source action parameters, no extra debugging fields)
   (list (cpp-raw-decl
          (format "struct ~aArguments\n{\n~a};\n\n"
                  vtx-name
                  (string-join
                   (for/list ([p (in-list payload-params)])
                     (format "    ~a ~a;\n"
                             (cca-type->cpp (param-decl-type p))
                             (mangle-cpp (param-decl-name p))))
                   ""))))

   ;; Compute CPI for each phase using the cost model
   (let* ([model (current-cost-model)]
          [pred-cpi (max 1 (compute-phase-cpi (action-decl-predicate act) model #:kind 'predicate))]
          [work-cpi (max 1 (compute-phase-cpi (action-decl-work act) model #:kind 'work))]
          [dpred-cpi (max 1 (compute-phase-cpi (action-decl-diffuse-predicate act) model #:kind 'diffuse-predicate))])
     (append
       ;; Predicate handler
       (list (cpp-raw-decl (emit-predicate-handler act-name vtx-name field-type
                                                    payload-field payload-params pred-expr
                                                    pred-cpi)))
       ;; Work handler
       (list (cpp-raw-decl (emit-work-handler act-name vtx-name field-type
                                               payload-field payload-params
                                               work-field work-value
                                               work-cpi)))
       ;; Diffuse predicate handler
       (list (cpp-raw-decl (emit-diffuse-predicate-handler act-name vtx-name field-type
                                                            payload-field payload-params dpred-expr
                                                            dpred-cpi)))
       ;; Diffuse handler
       (list (cpp-raw-decl (emit-diffuse-handler act-name vtx-name field-type field-name field-init
                                                  payload-field payload-params
                                                  prop-arg-cpp prop-action-name)))))))

;; ─── Handler generation functions ─────────────────────────────────────────────

(define (emit-predicate-handler act-name vtx-name field-type payload-field payload-params pred-expr cpi)
  (string-append
   (format "template<typename ghost_type>\nauto\n~a_predicate_T(ComputeCell& cc,\n               const Address addr,\n               const ActionArgumentType args) -> Closure\n{\n    cc.apply_CPI(~a);\n\n    auto* parent_recursive_parralel_vertex = static_cast<ghost_type*>(cc.get_object(addr));\n\n    if (parent_recursive_parralel_vertex->is_ghost_vertex) {\n        return Closure(cc.null_true_event, nullptr);\n    }\n\n    auto* v = static_cast<~a<ghost_type>*>(cc.get_object(addr));\n    ~aArguments const action_args = cca_get_action_argument<~aArguments>(args);\n\n"
           act-name cpi vtx-name vtx-name vtx-name)
   ;; Declare payload params as locals
   (string-join
    (for/list ([p (in-list payload-params)])
      (format "    ~a const ~a = action_args.~a;\n"
              (cca-type->cpp (param-decl-type p))
              (mangle-cpp (param-decl-name p))
              (mangle-cpp (param-decl-name p))))
    "")
   (format "\n    if (~a) {\n        return Closure(cc.null_true_event, nullptr);\n    }\n    return Closure(cc.null_false_event, nullptr);\n}\n\n"
           pred-expr)
   (format "inline auto\n~a_predicate_func(ComputeCell& cc,\n                  const Address addr,\n                  actionType,\n                  const ActionArgumentType args) -> Closure\n{\n    INVOKE_HANDLER_3(~a_predicate_T, cc, addr, args);\n}\n\n"
           act-name act-name)))

(define (emit-work-handler act-name vtx-name field-type payload-field payload-params
                           work-field work-value work-cpi)
  (string-append
   (format "template<typename ghost_type>\nauto\n~a_work_T(ComputeCell& cc,\n           const Address addr,\n           const ActionArgumentType args) -> Closure\n{\n    cc.apply_CPI(~a);\n\n    auto* parent_recursive_parralel_vertex = static_cast<ghost_type*>(cc.get_object(addr));\n\n    if (parent_recursive_parralel_vertex->is_ghost_vertex) {\n        return Closure(cc.null_true_event, nullptr);\n    }\n\n    auto* v = static_cast<~a<ghost_type>*>(cc.get_object(addr));\n    ~aArguments const action_args = cca_get_action_argument<~aArguments>(args);\n\n"
           act-name work-cpi vtx-name vtx-name vtx-name)
   (string-join
    (for/list ([p (in-list payload-params)])
      (format "    ~a const ~a = action_args.~a;\n"
              (cca-type->cpp (param-decl-type p))
              (mangle-cpp (param-decl-name p))
              (mangle-cpp (param-decl-name p))))
    "")
   (format "\n    v->~a = ~a;\n    return Closure(cc.null_true_event, nullptr);\n}\n\n"
           work-field work-value)
   (format "inline auto\n~a_work_func(ComputeCell& cc,\n             const Address addr,\n             actionType,\n             const ActionArgumentType args) -> Closure\n{\n    INVOKE_HANDLER_3(~a_work_T, cc, addr, args);\n}\n\n"
           act-name act-name)))

(define (emit-diffuse-predicate-handler act-name vtx-name field-type payload-field payload-params dpred-expr dpred-cpi)
  (string-append
   (format "template<typename ghost_type>\nauto\n~a_diffuse_predicate_T(ComputeCell& cc,\n                       const Address addr,\n                       const ActionArgumentType args) -> Closure\n{\n    cc.apply_CPI(~a);\n\n    auto* parent_recursive_parralel_vertex = static_cast<ghost_type*>(cc.get_object(addr));\n\n    if (parent_recursive_parralel_vertex->is_ghost_vertex) {\n        return Closure(cc.null_true_event, nullptr);\n    }\n\n    auto* v = static_cast<~a<ghost_type>*>(cc.get_object(addr));\n    ~aArguments const action_args = cca_get_action_argument<~aArguments>(args);\n\n"
           act-name dpred-cpi vtx-name vtx-name vtx-name)
   (string-join
    (for/list ([p (in-list payload-params)])
      (format "    ~a const ~a = action_args.~a;\n"
              (cca-type->cpp (param-decl-type p))
              (mangle-cpp (param-decl-name p))
              (mangle-cpp (param-decl-name p))))
    "")
   (format "\n    if (~a) {\n        return Closure(cc.null_true_event, nullptr);\n    }\n    return Closure(cc.null_false_event, nullptr);\n}\n\n"
           dpred-expr)
   (format "inline auto\n~a_diffuse_predicate_func(ComputeCell& cc,\n                          const Address addr,\n                          actionType,\n                          const ActionArgumentType args) -> Closure\n{\n    INVOKE_HANDLER_3(~a_diffuse_predicate_T, cc, addr, args);\n}\n\n"
           act-name act-name)))

(define (emit-diffuse-handler act-name vtx-name field-type field-name field-init
                              payload-field payload-params
                              prop-arg-cpp prop-action-name)
  (string-append
   (format "template<typename ghost_type>\nauto\n~a_diffuse_T(ComputeCell& cc,\n             const Address addr,\n             const ActionArgumentType args) -> Closure\n{\n    auto* parent_recursive_parralel_vertex = static_cast<ghost_type*>(cc.get_object(addr));\n    bool this_is_ghost_vertex = parent_recursive_parralel_vertex->is_ghost_vertex;\n\n    auto* v = static_cast<~a<ghost_type>*>(cc.get_object(addr));\n\n    ~a current_~a = ~a;\n    if (this_is_ghost_vertex) {\n        ~aArguments const action_args = cca_get_action_argument<~aArguments>(args);\n        current_~a = action_args.~a;\n    } else {\n        current_~a = v->~a;\n    }\n\n"
           act-name vtx-name
           field-type field-name field-init
           vtx-name vtx-name
           field-name payload-field
           field-name field-name)
   (format "    ~aArguments level_to_send;\n    level_to_send.~a = current_~a;\n\n    ActionArgumentType const args_for_ghost_vertices =\n        cca_create_action_argument<~aArguments>(level_to_send);\n\n"
           vtx-name payload-field field-name vtx-name)
   (format "    for (u_int32_t ghosts_iterator = 0; ghosts_iterator < ghost_type::ghost_vertices_max_degree;\n         ghosts_iterator++) {\n        if (v->ghost_vertices[ghosts_iterator].has_value()) {\n            cc.diffuse(Action(v->ghost_vertices[ghosts_iterator].value(),\n                              addr,\n                              actionType::application_action,\n                              true,\n                              args_for_ghost_vertices,\n                              ~a_predicate,\n                              ~a_work,\n                              ~a_diffuse_predicate,\n                              ~a_diffuse));\n        }\n    }\n\n"
           prop-action-name prop-action-name prop-action-name prop-action-name)
   (format "    for (int i = 0; i < v->number_of_edges; i++) {\n        level_to_send.~a = ~a;\n        ActionArgumentType const args_x =\n            cca_create_action_argument<~aArguments>(level_to_send);\n\n        cc.diffuse(Action(v->edges[i].edge,\n                          addr,\n                          actionType::application_action,\n                          true,\n                          args_x,\n                          ~a_predicate,\n                          ~a_work,\n                          ~a_diffuse_predicate,\n                          ~a_diffuse));\n    }\n\n    return Closure(cc.null_false_event, nullptr);\n}\n\n"
           payload-field prop-arg-cpp vtx-name
           prop-action-name prop-action-name prop-action-name prop-action-name)
   (format "inline auto\n~a_diffuse_func(ComputeCell& cc,\n                const Address addr,\n                actionType,\n                const ActionArgumentType args) -> Closure\n{\n    INVOKE_HANDLER_3(~a_diffuse_T, cc, addr, args);\n}\n\n"
           act-name act-name)))

;; ═══════════════════════════════════════════════════════════════════════════════
;; CPP IR Generation (host .cpp file)
;; ═══════════════════════════════════════════════════════════════════════════════

(define (generate-cpp-ir vtx actions app constants symbols hpp-filename)
  (define vtx-name (symbol->string (vertex-decl-name vtx)))
  (define binary-name (application-decl-binary-name app))
  (define field (car (vertex-decl-fields vtx)))
  (define field-name (mangle-cpp (field-decl-name field)))
  (define field-init
    (if (field-decl-initial field)
        (mangle-cpp (field-decl-initial field))
        "0"))

  (cpp-file
   (append
    (list (cpp-raw-decl "// Auto-generated by CCA Compiler v0.1.0\n// Do not edit — regenerate from .cca source.\n"))
    (list (cpp-raw-decl (emit-cpp-includes hpp-filename)))
    (list (cpp-raw-decl (emit-global-events actions)))
    (list (cpp-raw-decl (emit-main-function vtx actions app constants symbols
                                            vtx-name field-name field-init))))))

;; ─── Composable .cpp generation blocks ────────────────────────────────────────

(define (emit-cpp-includes hpp-filename)
  (format "#include \"~a\"\n\n#include \"CyclicMemoryAllocator.hpp\"\n#include \"Graph.hpp\"\n\n#include <chrono>\n#include <fstream>\n\n"
          hpp-filename))

(define (emit-global-events actions)
  (string-join
   (for/list ([act (in-list actions)])
     (define act-name (mangle-cpp (action-decl-name act)))
     (format "CCAFunctionEvent ~a_predicate;\nCCAFunctionEvent ~a_work;\nCCAFunctionEvent ~a_diffuse_predicate;\nCCAFunctionEvent ~a_diffuse;\n"
             act-name act-name act-name act-name))
   "\n"))

(define (emit-main-function vtx actions app constants symbols vtx-name field-name field-init)
  (string-append
   "\nauto\nmain(int argc, char** argv) -> int\n{\n"
   (emit-cli-block)
   (emit-simulator-block)
   (emit-graph-block vtx-name)
   (emit-registration-block actions)
   (emit-germination-block vtx-name actions field-name app)
   (emit-run-block)
   (emit-verification-block vtx-name field-name field-init app)
   "    return 0;\n}\n"))

(define (emit-cli-block)
  "    cli::Parser parser(argc, argv);\n    parser.set_required<std::string>(\"f\", \"graphfile\", \"Path to the input data graph file\");\n    parser.set_required<std::string>(\"g\", \"graphname\", \"Name of the input graph\");\n    parser.set_required<std::string>(\"s\", \"shape\", \"Shape of the compute cell\");\n    parser.set_required<u_int32_t>(\"root\", \"bfsroot\", \"Root vertex for BFS\");\n    parser.set_optional<bool>(\"verify\", \"verification\", 0, \"Enable verification\");\n    parser.set_optional<u_int32_t>(\"m\", \"memory_per_cc\", 512 * 1024, \"Memory per CC in bytes\");\n    parser.set_optional<std::string>(\"od\", \"outputdirectory\", \"./\", \"Output directory\");\n    parser.set_optional<u_int32_t>(\"hx\", \"htree_x\", 3, \"Htree X\");\n    parser.set_optional<u_int32_t>(\"hy\", \"htree_y\", 5, \"Htree Y\");\n    parser.set_optional<u_int32_t>(\"hdepth\", \"htree_depth\", 0, \"Htree depth\");\n    parser.set_optional<u_int32_t>(\"hb\", \"hbandwidth_max\", 64, \"Htree max bandwidth\");\n    parser.set_optional<u_int32_t>(\"mesh\", \"mesh_type\", 0, \"Mesh type: 0=Regular, 1=Torus\");\n    parser.set_optional<u_int32_t>(\"route\", \"routing_policy\", 0, \"Routing algorithm\");\n    parser.set_optional<bool>(\"shuffle\", \"shuffle_vertices\", 0, \"Shuffle vertex list\");\n    parser.set_optional<u_int32_t>(\"trail\", \"trail_number\", 0, \"Trail number\");\n    parser.run_and_exit_if_error();\n\n    auto input_graph_path = parser.get<std::string>(\"f\");\n    auto graph_name = parser.get<std::string>(\"g\");\n    auto shape_arg = parser.get<std::string>(\"s\");\n    auto root_vertex = parser.get<u_int32_t>(\"root\");\n    auto verify = parser.get<bool>(\"verify\");\n    auto memory_per_cc = parser.get<u_int32_t>(\"m\");\n    auto output_dir = parser.get<std::string>(\"od\");\n    auto hdepth = parser.get<u_int32_t>(\"hdepth\");\n    auto hx = parser.get<u_int32_t>(\"hx\");\n    auto hy = parser.get<u_int32_t>(\"hy\");\n    auto hbandwidth_max = parser.get<u_int32_t>(\"hb\");\n    auto mesh_type = parser.get<u_int32_t>(\"mesh\");\n    auto routing_policy = parser.get<u_int32_t>(\"route\");\n    auto shuffle = parser.get<bool>(\"shuffle\");\n\n    if (hdepth == 0) { hbandwidth_max = 0; }\n\n")

(define (emit-simulator-block)
  "    computeCellShape shape_of_compute_cells = computeCellShape::computeCellShape_invalid;\n    if (shape_arg == \"square\") {\n        shape_of_compute_cells = computeCellShape::square;\n    } else {\n        std::cerr << \"Error: shape \" << shape_arg << \" not supported.\\\\n\";\n        return EXIT_FAILURE;\n    }\n\n    CCASimulator cca_simulator(shape_of_compute_cells,\n                               hx, hy, hdepth, hbandwidth_max,\n                               memory_per_cc, mesh_type, routing_policy);\n\n    cca_simulator.print_discription(std::cout);\n\n")

(define (emit-graph-block vtx-name)
  (format "    Graph<~a<SimpleVertex<host_edge_type, edges_min>>> input_graph(input_graph_path, false);\n\n    u_int32_t center_of_chip =\n        (cca_simulator.dim_x * (cca_simulator.dim_y / 2)) + (cca_simulator.dim_y / 2);\n    CyclicMemoryAllocator allocator(center_of_chip, cca_simulator.total_compute_cells);\n\n    input_graph.transfer_graph_host_to_cca<~a<ghost_type_level_1>>(\n        cca_simulator, allocator, std::optional<u_int32_t>(root_vertex), shuffle);\n\n    auto vertex_addr = input_graph.get_vertex_address_in_cca(root_vertex);\n\n"
          vtx-name vtx-name))

(define (emit-registration-block actions)
  (string-join
   (for/list ([act (in-list actions)])
     (define act-name (mangle-cpp (action-decl-name act)))
     (format "    ~a_predicate = cca_simulator.register_function_event(~a_predicate_func);\n    ~a_work = cca_simulator.register_function_event(~a_work_func);\n    ~a_diffuse_predicate = cca_simulator.register_function_event(~a_diffuse_predicate_func);\n    ~a_diffuse = cca_simulator.register_function_event(~a_diffuse_func);\n"
             act-name act-name act-name act-name act-name act-name act-name act-name))
   "\n"))

(define (emit-germination-block vtx-name actions field-name app)
  (define act (car actions))  ; germinate root action
  (define act-name (mangle-cpp (action-decl-name act)))
  (define payload-params (action-decl-params act))
  (define payload-field (mangle-cpp (param-decl-name (car payload-params))))
  (define root-args (application-decl-root-arguments app))

  (string-append
   (format "\n    ~aArguments root_args;\n" vtx-name)
   ;; Set payload fields from root-arguments
   (string-join
    (for/list ([p (in-list payload-params)]
               [val (in-list root-args)])
      (format "    root_args.~a = ~a;\n" (mangle-cpp (param-decl-name p)) val))
    "")
   (format "\n    ActionArgumentType const args_x = cca_create_action_argument<~aArguments>(root_args);\n\n" vtx-name)
   "    std::optional<Address> terminator = cca_simulator.create_terminator();\n    if (!terminator) {\n        std::cerr << \"Error! Memory not allocated for terminator\\\\n\";\n        return EXIT_FAILURE;\n    }\n\n"
   (format "    cca_simulator.germinate_action(Action(vertex_addr,\n                                          terminator.value(),\n                                          actionType::germinate_action,\n                                          true,\n                                          args_x,\n                                          ~a_predicate,\n                                          ~a_work,\n                                          ~a_diffuse_predicate,\n                                          ~a_diffuse));\n\n"
           act-name act-name act-name act-name)))

(define (emit-run-block)
  "    std::cout << \"\\\\nStarting Execution:\\\\n\\\\n\";\n    auto start = std::chrono::steady_clock::now();\n    cca_simulator.run_simulation(terminator.value());\n    auto end = std::chrono::steady_clock::now();\n\n    std::cout << \"Total Cycles: \" << cca_simulator.total_cycles << \"\\\\n\";\n    std::cout << \"Elapsed: \"\n              << std::chrono::duration_cast<std::chrono::seconds>(end - start).count()\n              << \" s\\\\n\";\n\n")

(define (emit-verification-block vtx-name field-name field-init app)
  (define verification (application-decl-verification app))
  (if verification
      (string-append
       "    if (verify) {\n"
       "        std::cout << \"\\\\nBFS Verification:\\\\n\";\n"
       (format "        std::string verification_file = input_graph_path + \".bfs\";\n")
       "        std::ifstream file(verification_file);\n"
       "        if (!file.is_open()) {\n"
       "            std::cout << \"Failed to open: \" << verification_file << \"\\\\n\";\n"
       "        } else {\n"
       "            std::string line;\n"
       "            std::getline(file, line); // header\n"
       "            u_int32_t root_in_file = 0;\n"
       "            std::getline(file, line);\n"
       "            std::istringstream(line) >> root_in_file;\n"
       "            if (root_in_file != root_vertex) {\n"
       "                std::cerr << \"Root mismatch in verification file!\\\\n\";\n"
       "                return EXIT_FAILURE;\n"
       "            }\n\n"
       "            std::vector<u_int32_t> control;\n"
       "            u_int32_t nid, bval;\n"
       "            while (std::getline(file, line)) {\n"
       "                std::istringstream iss(line);\n"
       "                if (iss >> nid >> bval) {\n"
       (format "                    while (nid != control.size()) { control.emplace_back(~a); }\n" field-init)
       "                    control.emplace_back(bval);\n"
       "                }\n"
       "            }\n"
       "            file.close();\n\n"
       "            u_int32_t errors = 0;\n"
       "            for (u_int32_t i = 0; i < control.size(); i++) {\n"
       "                Address addr_i = input_graph.get_vertex_address_in_cca(i);\n"
       (format "                auto* vi = static_cast<~a<ghost_type_level_1>*>(cca_simulator.get_object(addr_i));\n" vtx-name)
       (format "                if (control[i] != vi->~a) {\n" field-name)
       (format "                    std::cout << \"Vertex \" << i << \": computed=\" << vi->~a\n" field-name)
       "                              << \" expected=\" << control[i] << \"\\\\n\";\n"
       "                    errors++;\n"
       "                }\n"
       "            }\n"
       "            if (errors > 0) {\n"
       "                std::cout << \"FAILED: \" << errors << \" errors\\\\n\";\n"
       "                return EXIT_FAILURE;\n"
       "            } else {\n"
       "                std::cout << \"PASSED\\\\n\";\n"
       "            }\n"
       "        }\n"
       "    }\n\n")
      ""))

;; ─── Generate CMakeLists.txt ──────────────────────────────────────────────────
(define (generate-cmake binary-name cpp-filename)
  (format "cca_add_application(NAME ~a SOURCE ~a)\n" binary-name cpp-filename))

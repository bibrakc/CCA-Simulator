#lang racket/base

;; BSD 3-Clause License
;;
;; Copyright (c) 2026, Bibrak Qamar

(require racket/match
         racket/list
         racket/string
         racket/format
         racket/port
         racket/file
         "../ast.rkt"
         "../frontend/resolve.rkt")

(provide emit-cpp-pass)

;; ─── Main entry: emit all generated files ─────────────────────────────────────
;; Input: (list resolved-program output-directory)
;; Output: writes .hpp, .cpp, and CMakeLists.txt to output-directory

(define (emit-cpp-pass resolved+output)
  (match-define (list resolved output-dir) resolved+output)
  (define vtx (resolved-program-vertex resolved))
  (define actions (resolved-program-actions resolved))
  (define app (resolved-program-application resolved))
  (define constants (resolved-program-constants resolved))
  (define symbols (resolved-program-symbols resolved))

  ;; We support exactly one action for now (BFS)
  (define act (car actions))

  ;; Derive names
  (define app-name-lower (string-downcase (symbol->string (application-decl-name app))))
  (define hpp-filename (format "cca_~a_generated.hpp" app-name-lower))
  (define cpp-filename (format "cca_~a_generated.cpp" app-name-lower))
  (define binary-name (application-decl-binary-name app))

  ;; Generate content
  (define hpp-content (generate-hpp vtx act app constants symbols))
  (define cpp-content (generate-cpp vtx act app constants symbols hpp-filename))
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

;; ─── Helper: C++ type from CCA type ──────────────────────────────────────────
(define (cpp-type t)
  (match t
    [(t-u32) "u_int32_t"]
    [(t-bool) "bool"]
    [(t-address) "Address"]
    [_ "u_int32_t"]))

;; ─── Helper: C++ expression from AST expr ────────────────────────────────────
(define (cpp-expr e target-var args-var)
  (match e
    [(literal-expr _ val _) (format "~a" val)]
    [(var-expr _ name)
     (cond
       [(equal? name target-var) "v"]
       [else (format "~a" (mangle-cpp name))])]
    [(prim-expr _ op args)
     (cond
       [(= (length args) 1)
        (format "(~a~a)" (cpp-op op) (cpp-expr (car args) target-var args-var))]
       [(= (length args) 2)
        (format "(~a ~a ~a)"
                (cpp-expr (car args) target-var args-var)
                (cpp-op op)
                (cpp-expr (cadr args) target-var args-var))]
       [else (error 'cpp-expr "unsupported arity for ~a" op)])]
    [(vertex-read-expr _ field target)
     (format "v->~a" (mangle-cpp field))]
    [(edge-read-expr _ 'address _) "v->edges[i].edge"]
    [(edge-read-expr _ 'weight _) "v->edges[i].weight"]
    [_ (error 'cpp-expr "unsupported expr: ~s" e)]))

(define (cpp-op op)
  (case op
    [(+) "+"] [(-) "-"] [(*) "*"]
    [(>) ">"] [(>=) ">="] [(<) "<"] [(<=) "<="]
    [(=) "=="] [(and) "&&"] [(or) "||"] [(not) "!"]
    [else (error 'cpp-op "unknown op: ~a" op)]))

(define (mangle-cpp sym)
  (string-replace (string-replace (symbol->string sym) "-" "_") "!" ""))

;; cpp-expr-diffuse: in the diffuse handler, payload params are accessed via current_<field>
(define (cpp-expr-diffuse e target-var payload-field field-name)
  (match e
    [(literal-expr _ val _) (format "~a" val)]
    [(var-expr _ name)
     (cond
       [(equal? (mangle-cpp name) payload-field)
        (format "current_~a" field-name)]
       [(equal? name target-var) "v"]
       [else (format "~a" (mangle-cpp name))])]
    [(prim-expr _ op args)
     (cond
       [(= (length args) 1)
        (format "(~a~a)" (cpp-op op) (cpp-expr-diffuse (car args) target-var payload-field field-name))]
       [(= (length args) 2)
        (format "(~a ~a ~a)"
                (cpp-expr-diffuse (car args) target-var payload-field field-name)
                (cpp-op op)
                (cpp-expr-diffuse (cadr args) target-var payload-field field-name))]
       [else (error 'cpp-expr-diffuse "unsupported arity for ~a" op)])]
    [_ (error 'cpp-expr-diffuse "unsupported expr in diffuse: ~s" e)]))

;; ─── Generate .hpp ────────────────────────────────────────────────────────────
(define (generate-hpp vtx act app constants symbols)
  (define vtx-name (symbol->string (vertex-decl-name vtx)))
  (define act-name (mangle-cpp (action-decl-name act)))
  (define target-param-name (param-decl-name (action-decl-target act)))
  (define payload-params (action-decl-params act))

  ;; Build the payload struct field name
  (define payload-field (mangle-cpp (param-decl-name (car payload-params))))

  ;; Constant values
  (define const-defs
    (string-join
     (for/list ([c (in-list constants)])
       (format "inline static constexpr ~a ~a = ~a;"
               (cpp-type (constant-decl-type c))
               (mangle-cpp (constant-decl-name c))
               (constant-decl-value c)))
     "\n"))

  ;; Vertex field
  (define field (car (vertex-decl-fields vtx)))
  (define field-name (mangle-cpp (field-decl-name field)))
  (define field-type (cpp-type (field-decl-type field)))
  (define field-init
    (if (field-decl-initial field)
        (format "~a" (mangle-cpp (field-decl-initial field)))
        "0"))

  ;; Predicate expression
  (define pred-expr-cpp
    (cpp-expr (action-decl-predicate act) target-param-name payload-field))
  ;; Diffuse predicate expression
  (define dpred-expr-cpp
    (cpp-expr (action-decl-diffuse-predicate act) target-param-name payload-field))

  ;; Work assignment
  (define work-stmt (car (action-decl-work act)))
  (define work-field (mangle-cpp (set-field-stmt-field work-stmt)))
  (define work-value (cpp-expr (set-field-stmt-value work-stmt) target-param-name payload-field))

  ;; Diffuse: propagate expression for the level to send
  (define diff-body (car (action-decl-diffuse act)))  ; for-edges-stmt
  (define prop-stmt (car (for-edges-stmt-body diff-body)))  ; propagate-stmt
  (define prop-arg (car (propagate-stmt-args prop-stmt)))  ; the level expression
  ;; In diffuse context, payload fields are accessed via current_<field>
  (define prop-arg-cpp (cpp-expr-diffuse prop-arg target-param-name payload-field field-name))

  (string-append
   (generated-header)
   #<<HPP
#ifndef CCA_BFS_GENERATED_HPP
#define CCA_BFS_GENERATED_HPP

#include "CCASimulator.hpp"
#include "Enums.hpp"
#include "Graph.hpp"
#include "RecursiveParallelVertex.hpp"
#include "cmdparser.hpp"

#include <cstring>
#include <fstream>

HPP
   "\n"
   const-defs
   "\n\n"
   (format #<<VERTEX
template<typename Vertex_T>
struct ~a : Vertex_T
{
    ~a ~a{~a};

    ~a(u_int32_t id_in, u_int32_t total_number_of_vertices_in)
        : ~a(~a)
    {
        this->id = id_in;
        this->number_of_edges = 0;
        this->total_number_of_vertices = total_number_of_vertices_in;
    }

    void configure_derived_class_LCOs() {}

    ~a() {}
    ~~~a() {}
};

VERTEX
           vtx-name
           field-type field-name field-init
           vtx-name
           field-name field-init
           vtx-name
           vtx-name)
   "\n"
   ;; Function event declarations
   (format #<<EVENTS
extern CCAFunctionEvent ~a_predicate;
extern CCAFunctionEvent ~a_work;
extern CCAFunctionEvent ~a_diffuse_predicate;
extern CCAFunctionEvent ~a_diffuse;

EVENTS
           act-name act-name act-name act-name)
   "\n"
   ;; Payload struct
   (format #<<PAYLOAD
struct ~aArguments
{
    ~a ~a;
    u_int32_t src_vertex_id;
};

PAYLOAD
           vtx-name
           (cpp-type (param-decl-type (car payload-params)))
           payload-field)
   "\n"
   ;; Predicate handler
   (format #<<PRED
template<typename ghost_type>
auto
~a_predicate_T(ComputeCell& cc,
               const Address addr,
               const ActionArgumentType args) -> Closure
{
    cc.apply_CPI(1);

    auto* parent_recursive_parralel_vertex = static_cast<ghost_type*>(cc.get_object(addr));

    if (parent_recursive_parralel_vertex->is_ghost_vertex) {
        return Closure(cc.null_true_event, nullptr);
    }

    auto* v = static_cast<~a<ghost_type>*>(cc.get_object(addr));
    ~aArguments const action_args = cca_get_action_argument<~aArguments>(args);

    ~a const ~a = action_args.~a;

    if (~a) {
        return Closure(cc.null_true_event, nullptr);
    }
    return Closure(cc.null_false_event, nullptr);
}

inline auto
~a_predicate_func(ComputeCell& cc,
                  const Address addr,
                  actionType,
                  const ActionArgumentType args) -> Closure
{
    INVOKE_HANDLER_3(~a_predicate_T, cc, addr, args);
}

PRED
           act-name
           vtx-name
           vtx-name vtx-name
           field-type payload-field payload-field
           pred-expr-cpp
           act-name
           act-name)
   "\n"
   ;; Work handler
   (format #<<WORK
template<typename ghost_type>
auto
~a_work_T(ComputeCell& cc,
           const Address addr,
           const ActionArgumentType args) -> Closure
{
    cc.apply_CPI(1);

    auto* parent_recursive_parralel_vertex = static_cast<ghost_type*>(cc.get_object(addr));

    if (parent_recursive_parralel_vertex->is_ghost_vertex) {
        return Closure(cc.null_true_event, nullptr);
    }

    auto* v = static_cast<~a<ghost_type>*>(cc.get_object(addr));
    ~aArguments const action_args = cca_get_action_argument<~aArguments>(args);

    ~a const ~a = action_args.~a;

    v->~a = ~a;
    return Closure(cc.null_true_event, nullptr);
}

inline auto
~a_work_func(ComputeCell& cc,
             const Address addr,
             actionType,
             const ActionArgumentType args) -> Closure
{
    INVOKE_HANDLER_3(~a_work_T, cc, addr, args);
}

WORK
           act-name
           vtx-name
           vtx-name vtx-name
           field-type payload-field payload-field
           work-field work-value
           act-name
           act-name)
   "\n"
   ;; Diffuse predicate handler
   (format #<<DPRED
template<typename ghost_type>
auto
~a_diffuse_predicate_T(ComputeCell& cc,
                       const Address addr,
                       const ActionArgumentType args) -> Closure
{
    cc.apply_CPI(1);

    auto* parent_recursive_parralel_vertex = static_cast<ghost_type*>(cc.get_object(addr));

    if (parent_recursive_parralel_vertex->is_ghost_vertex) {
        return Closure(cc.null_true_event, nullptr);
    }

    auto* v = static_cast<~a<ghost_type>*>(cc.get_object(addr));
    ~aArguments const action_args = cca_get_action_argument<~aArguments>(args);

    ~a const ~a = action_args.~a;

    if (~a) {
        return Closure(cc.null_true_event, nullptr);
    }
    return Closure(cc.null_false_event, nullptr);
}

inline auto
~a_diffuse_predicate_func(ComputeCell& cc,
                          const Address addr,
                          actionType,
                          const ActionArgumentType args) -> Closure
{
    INVOKE_HANDLER_3(~a_diffuse_predicate_T, cc, addr, args);
}

DPRED
           act-name
           vtx-name
           vtx-name vtx-name
           field-type payload-field payload-field
           dpred-expr-cpp
           act-name
           act-name)
   "\n"
   ;; Diffuse handler
   (format #<<DIFFUSE
template<typename ghost_type>
auto
~a_diffuse_T(ComputeCell& cc,
             const Address addr,
             const ActionArgumentType args) -> Closure
{
    auto* parent_recursive_parralel_vertex = static_cast<ghost_type*>(cc.get_object(addr));
    bool this_is_ghost_vertex = parent_recursive_parralel_vertex->is_ghost_vertex;

    auto* v = static_cast<~a<ghost_type>*>(cc.get_object(addr));

    ~a current_~a = ~a;
    if (this_is_ghost_vertex) {
        ~aArguments const action_args = cca_get_action_argument<~aArguments>(args);
        current_~a = action_args.~a;
    } else {
        current_~a = v->~a;
    }

    ~aArguments level_to_send;
    level_to_send.~a = current_~a;
    level_to_send.src_vertex_id = v->id;

    ActionArgumentType const args_for_ghost_vertices =
        cca_create_action_argument<~aArguments>(level_to_send);

    for (u_int32_t ghosts_iterator = 0; ghosts_iterator < ghost_type::ghost_vertices_max_degree;
         ghosts_iterator++) {
        if (v->ghost_vertices[ghosts_iterator].has_value()) {
            cc.diffuse(Action(v->ghost_vertices[ghosts_iterator].value(),
                              addr,
                              actionType::application_action,
                              true,
                              args_for_ghost_vertices,
                              ~a_predicate,
                              ~a_work,
                              ~a_diffuse_predicate,
                              ~a_diffuse));
        }
    }

    for (int i = 0; i < v->number_of_edges; i++) {
        level_to_send.~a = ~a;
        ActionArgumentType const args_x =
            cca_create_action_argument<~aArguments>(level_to_send);

        cc.diffuse(Action(v->edges[i].edge,
                          addr,
                          actionType::application_action,
                          true,
                          args_x,
                          ~a_predicate,
                          ~a_work,
                          ~a_diffuse_predicate,
                          ~a_diffuse));
    }

    return Closure(cc.null_false_event, nullptr);
}

inline auto
~a_diffuse_func(ComputeCell& cc,
                const Address addr,
                actionType,
                const ActionArgumentType args) -> Closure
{
    INVOKE_HANDLER_3(~a_diffuse_T, cc, addr, args);
}

DIFFUSE
           act-name
           vtx-name
           field-type field-name field-init
           vtx-name vtx-name
           field-name payload-field
           field-name field-name
           vtx-name
           payload-field field-name
           vtx-name
           act-name act-name act-name act-name
           payload-field prop-arg-cpp
           vtx-name
           act-name act-name act-name act-name
           act-name
           act-name)
   "\n"
   "#endif // CCA_BFS_GENERATED_HPP\n"))

;; ─── Generate .cpp ────────────────────────────────────────────────────────────
(define (generate-cpp vtx act app constants symbols hpp-filename)
  (define vtx-name (symbol->string (vertex-decl-name vtx)))
  (define act-name (mangle-cpp (action-decl-name act)))
  (define binary-name (application-decl-binary-name app))
  (define payload-field (mangle-cpp (param-decl-name (car (action-decl-params act)))))
  (define field-name (mangle-cpp (field-decl-name (car (vertex-decl-fields vtx)))))
  (define field-init
    (if (field-decl-initial (car (vertex-decl-fields vtx)))
        (format "~a" (mangle-cpp (field-decl-initial (car (vertex-decl-fields vtx)))))
        "0"))

  (string-append
   (generated-header)
   (format #<<CPP
#include "~a"

#include "CyclicMemoryAllocator.hpp"
#include "Graph.hpp"

#include <chrono>
#include <fstream>

CCAFunctionEvent ~a_predicate;
CCAFunctionEvent ~a_work;
CCAFunctionEvent ~a_diffuse_predicate;
CCAFunctionEvent ~a_diffuse;

auto
main(int argc, char** argv) -> int
{
    cli::Parser parser(argc, argv);
    parser.set_required<std::string>("f", "graphfile", "Path to the input data graph file");
    parser.set_required<std::string>("g", "graphname", "Name of the input graph");
    parser.set_required<std::string>("s", "shape", "Shape of the compute cell");
    parser.set_required<u_int32_t>("root", "bfsroot", "Root vertex for BFS");
    parser.set_optional<bool>("verify", "verification", 0, "Enable verification");
    parser.set_optional<u_int32_t>("m", "memory_per_cc", 512 * 1024, "Memory per CC in bytes");
    parser.set_optional<std::string>("od", "outputdirectory", "./", "Output directory");
    parser.set_optional<u_int32_t>("hx", "htree_x", 3, "Htree X");
    parser.set_optional<u_int32_t>("hy", "htree_y", 5, "Htree Y");
    parser.set_optional<u_int32_t>("hdepth", "htree_depth", 0, "Htree depth");
    parser.set_optional<u_int32_t>("hb", "hbandwidth_max", 64, "Htree max bandwidth");
    parser.set_optional<u_int32_t>("mesh", "mesh_type", 0, "Mesh type: 0=Regular, 1=Torus");
    parser.set_optional<u_int32_t>("route", "routing_policy", 0, "Routing algorithm");
    parser.set_optional<bool>("shuffle", "shuffle_vertices", 0, "Shuffle vertex list");
    parser.set_optional<u_int32_t>("trail", "trail_number", 0, "Trail number");
    parser.run_and_exit_if_error();

    auto input_graph_path = parser.get<std::string>("f");
    auto graph_name = parser.get<std::string>("g");
    auto shape_arg = parser.get<std::string>("s");
    auto root_vertex = parser.get<u_int32_t>("root");
    auto verify = parser.get<bool>("verify");
    auto memory_per_cc = parser.get<u_int32_t>("m");
    auto output_dir = parser.get<std::string>("od");
    auto hdepth = parser.get<u_int32_t>("hdepth");
    auto hx = parser.get<u_int32_t>("hx");
    auto hy = parser.get<u_int32_t>("hy");
    auto hbandwidth_max = parser.get<u_int32_t>("hb");
    auto mesh_type = parser.get<u_int32_t>("mesh");
    auto routing_policy = parser.get<u_int32_t>("route");
    auto shuffle = parser.get<bool>("shuffle");

    if (hdepth == 0) { hbandwidth_max = 0; }

    computeCellShape shape_of_compute_cells = computeCellShape::computeCellShape_invalid;
    if (shape_arg == "square") {
        shape_of_compute_cells = computeCellShape::square;
    } else {
        std::cerr << "Error: shape " << shape_arg << " not supported.\\n";
        return EXIT_FAILURE;
    }

    CCASimulator cca_simulator(shape_of_compute_cells,
                               hx, hy, hdepth, hbandwidth_max,
                               memory_per_cc, mesh_type, routing_policy);

    cca_simulator.print_discription(std::cout);

    Graph<~a<SimpleVertex<host_edge_type, edges_min>>> input_graph(input_graph_path, false);

    u_int32_t center_of_chip =
        (cca_simulator.dim_x * (cca_simulator.dim_y / 2)) + (cca_simulator.dim_y / 2);
    CyclicMemoryAllocator allocator(center_of_chip, cca_simulator.total_compute_cells);

    input_graph.transfer_graph_host_to_cca<~a<ghost_type_level_1>>(
        cca_simulator, allocator, std::optional<u_int32_t>(root_vertex), shuffle);

    auto vertex_addr = input_graph.get_vertex_address_in_cca(root_vertex);

    ~a_predicate = cca_simulator.register_function_event(~a_predicate_func);
    ~a_work = cca_simulator.register_function_event(~a_work_func);
    ~a_diffuse_predicate = cca_simulator.register_function_event(~a_diffuse_predicate_func);
    ~a_diffuse = cca_simulator.register_function_event(~a_diffuse_func);

    ~aArguments root_args;
    root_args.~a = 0;
    root_args.src_vertex_id = 99999;

    ActionArgumentType const args_x = cca_create_action_argument<~aArguments>(root_args);

    std::optional<Address> terminator = cca_simulator.create_terminator();
    if (!terminator) {
        std::cerr << "Error! Memory not allocated for terminator\\n";
        return EXIT_FAILURE;
    }

    cca_simulator.germinate_action(Action(vertex_addr,
                                          terminator.value(),
                                          actionType::germinate_action,
                                          true,
                                          args_x,
                                          ~a_predicate,
                                          ~a_work,
                                          ~a_diffuse_predicate,
                                          ~a_diffuse));

    std::cout << "\\nStarting Execution:\\n\\n";
    auto start = std::chrono::steady_clock::now();
    cca_simulator.run_simulation(terminator.value());
    auto end = std::chrono::steady_clock::now();

    std::cout << "Total Cycles: " << cca_simulator.total_cycles << "\\n";
    std::cout << "Elapsed: "
              << std::chrono::duration_cast<std::chrono::seconds>(end - start).count()
              << " s\\n";

    if (verify) {
        std::cout << "\\nBFS Verification:\\n";
        std::string verification_file = input_graph_path + ".bfs";
        std::ifstream file(verification_file);
        if (!file.is_open()) {
            std::cout << "Failed to open: " << verification_file << "\\n";
        } else {
            std::string line;
            std::getline(file, line); // header
            u_int32_t root_in_file = 0;
            std::getline(file, line);
            std::istringstream(line) >> root_in_file;
            if (root_in_file != root_vertex) {
                std::cerr << "Root mismatch in verification file!\\n";
                return EXIT_FAILURE;
            }

            std::vector<u_int32_t> control;
            u_int32_t nid, bval;
            while (std::getline(file, line)) {
                std::istringstream iss(line);
                if (iss >> nid >> bval) {
                    while (nid != control.size()) { control.emplace_back(~a); }
                    control.emplace_back(bval);
                }
            }
            file.close();

            u_int32_t errors = 0;
            for (u_int32_t i = 0; i < control.size(); i++) {
                Address addr_i = input_graph.get_vertex_address_in_cca(i);
                auto* vi = static_cast<~a<ghost_type_level_1>*>(cca_simulator.get_object(addr_i));
                if (control[i] != vi->~a) {
                    std::cout << "Vertex " << i << ": computed=" << vi->~a
                              << " expected=" << control[i] << "\\n";
                    errors++;
                }
            }
            if (errors > 0) {
                std::cout << "FAILED: " << errors << " errors\\n";
                return EXIT_FAILURE;
            } else {
                std::cout << "PASSED\\n";
            }
        }
    }

    return 0;
}

CPP
           hpp-filename
           act-name act-name act-name act-name
           vtx-name
           vtx-name
           act-name act-name
           act-name act-name
           act-name act-name
           act-name act-name
           vtx-name payload-field
           vtx-name
           act-name act-name act-name act-name
           field-init
           vtx-name field-name
           field-name)))

;; ─── Generate CMakeLists.txt ──────────────────────────────────────────────────
(define (generate-cmake binary-name cpp-filename)
  (format "cca_add_application(NAME ~a SOURCE ~a)\n" binary-name cpp-filename))

;; ─── Generated file header ────────────────────────────────────────────────────
(define (generated-header)
  #<<HEADER
// Auto-generated by CCA Compiler v0.1.0
// Do not edit — regenerate from .cca source.

HEADER
  )

(define (string-downcase s)
  (string-foldcase s))

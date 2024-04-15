/*
BSD 3-Clause License

Copyright (c) 2023-2024, Bibrak Qamar

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef CCA_SSSP_Rhizome_HPP
#define CCA_SSSP_Rhizome_HPP

#include "CCASimulator.hpp"
#include "Enums.hpp"

// Datastructures
#include "Graph.hpp"
#include "RhizomeRecursiveParallelVertex.hpp"

#include "cmdparser.hpp"

// For memcpy()
#include <cstring>

#include <fstream>

inline static constexpr u_int32_t undefined_distance = 999999;

template<typename Vertex_T>
struct SSSPVertex : Vertex_T
{
    inline static constexpr u_int32_t max_distance = undefined_distance;
    u_int32_t sssp_distance;

    SSSPVertex(u_int32_t id_in, u_int32_t total_number_of_vertices_in)
        : sssp_distance(SSSPVertex::max_distance)
    {
        this->id = id_in;
        this->number_of_edges = 0;
        this->total_number_of_vertices = total_number_of_vertices_in;
    }

    // Nothing to do.
    void configure_derived_class_LCOs() {}

    SSSPVertex() = default;
    ~SSSPVertex() = default;
};

// CCAFunctionEvent ids for the SSSP action: predicate, work, and diffuse.
// In the main register the functions with the CCASimulator chip and get their ids.
extern CCAFunctionEvent sssp_predicate;
extern CCAFunctionEvent sssp_work;
extern CCAFunctionEvent sssp_diffuse_predicate;
extern CCAFunctionEvent sssp_diffuse;

// This is what the action carries as payload.
struct SSSPArguments
{
    u_int32_t distance;
    u_int32_t src_vertex_id;
};

template<typename ghost_type>
inline auto
sssp_predicate_T(ComputeCell& cc, const Address addr, const ActionArgumentType args) -> Closure
{

    // First check whether this is a ghost vertex.If it is then always predicate true.
    // parent word is used in the sense that `RhizomeRecursiveParallelVertex` is the parent class.
    auto* parent_recursive_parralel_vertex = static_cast<ghost_type*>(cc.get_object(addr));

    if (parent_recursive_parralel_vertex->is_ghost_vertex) {
        return Closure(cc.null_true_event, nullptr);
    }

    auto* v = static_cast<SSSPVertex<ghost_type>*>(cc.get_object(addr));

    SSSPArguments const sssp_args = cca_get_action_argument<SSSPArguments>(args);
    u_int32_t const incoming_distance = sssp_args.distance;

    if (v->sssp_distance > incoming_distance) {
        return Closure(cc.null_true_event, nullptr);
    }
    return Closure(cc.null_false_event, nullptr);
}

inline auto
sssp_predicate_func(ComputeCell& cc,
                    const Address addr,
                    actionType /* action_type_in */,
                    const ActionArgumentType args) -> Closure
{
    INVOKE_HANDLER_3(sssp_predicate_T, cc, addr, args);
}

template<typename ghost_type>
inline auto
sssp_work_T(ComputeCell& cc, const Address addr, const ActionArgumentType args) -> Closure
{

    // First check whether this is a ghost vertex. If it is then don't perform any work.
    // parent word is used in the sense that `RhizomeRecursiveParallelVertex` is the parent class.
    auto* parent_recursive_parralel_vertex = static_cast<ghost_type*>(cc.get_object(addr));

    if (parent_recursive_parralel_vertex->is_ghost_vertex) {
        return Closure(cc.null_true_event, nullptr);
    }

    auto* v = static_cast<SSSPVertex<ghost_type>*>(cc.get_object(addr));

    SSSPArguments const sssp_args = cca_get_action_argument<SSSPArguments>(args);
    u_int32_t const incoming_distance = sssp_args.distance;

    // Update distance with the new distance
    v->sssp_distance = incoming_distance;
    return Closure(cc.null_true_event, nullptr);
}

inline auto
sssp_work_func(ComputeCell& cc,
               const Address addr,
               actionType /* action_type_in */,
               const ActionArgumentType args) -> Closure
{
    INVOKE_HANDLER_3(sssp_work_T, cc, addr, args);
}

template<typename ghost_type>
inline auto
sssp_diffuse_predicate_T(ComputeCell& cc, const Address addr, const ActionArgumentType args)
    -> Closure
{

    // First check whether this is a ghost vertex.If it is then always predicate true.
    // parent word is used in the sense that `RhizomeRecursiveParallelVertex` is the parent class.
    auto* parent_recursive_parallel_vertex = static_cast<ghost_type*>(cc.get_object(addr));

    if (parent_recursive_parallel_vertex->is_ghost_vertex) {
        return Closure(cc.null_true_event, nullptr);
    }

    auto* v = static_cast<SSSPVertex<ghost_type>*>(cc.get_object(addr));

    SSSPArguments const sssp_args = cca_get_action_argument<SSSPArguments>(args);
    u_int32_t const incoming_distance = sssp_args.distance;

    if (v->sssp_distance == incoming_distance) {
        return Closure(cc.null_true_event, nullptr);
    }
    return Closure(cc.null_false_event, nullptr);
}

inline auto
sssp_diffuse_predicate_func(ComputeCell& cc,
                            const Address addr,
                            actionType /* action_type_in */,
                            const ActionArgumentType args) -> Closure
{
    INVOKE_HANDLER_3(sssp_diffuse_predicate_T, cc, addr, args);
}

template<typename ghost_type>
inline auto
sssp_diffuse_T(ComputeCell& cc, const Address addr, const ActionArgumentType args) -> Closure
{

    // Get the hold of the parent ghost vertex. If it is ghost then simply perform diffusion.
    auto* parent_recursive_parallel_vertex = static_cast<ghost_type*>(cc.get_object(addr));
    bool this_is_ghost_vertex = parent_recursive_parallel_vertex->is_ghost_vertex;
    bool const this_is_rhizome_vertex = parent_recursive_parallel_vertex->is_rhizome_vertex;

    auto* v = static_cast<SSSPVertex<ghost_type>*>(cc.get_object(addr));

    u_int32_t current_distance = SSSPVertex<ghost_type>::max_distance;
    if (this_is_ghost_vertex) {
        SSSPArguments const sssp_args = cca_get_action_argument<SSSPArguments>(args);
        current_distance = sssp_args.distance;
    } else {
        current_distance = v->sssp_distance;
    }

    SSSPArguments distance_to_send;
    // To be potentially sent to ghost vertices.
    distance_to_send.distance = current_distance;
    distance_to_send.src_vertex_id = v->id;

    ActionArgumentType const args_for_ghost_vertices =
        cca_create_action_argument<SSSPArguments>(distance_to_send);

    // Relay to the Rhizome link
    for (u_int32_t rhizome_iterator = 0;
         rhizome_iterator < SSSPVertex<ghost_type>::rhizome_vertices_max_degree;
         rhizome_iterator++) {

        if (v->rhizome_vertices[rhizome_iterator].has_value()) {
            cc.diffuse(Action(v->rhizome_vertices[rhizome_iterator].value(),
                              addr,
                              actionType::application_action,
                              true,
                              args_for_ghost_vertices, // same as if relaying to ghosts
                              sssp_predicate,
                              sssp_work,
                              sssp_diffuse_predicate,
                              sssp_diffuse));
        }
    }

    // Note: The application vertex type is derived from the parent `RhizomeRecursiveParallelVertex`
    // therefore using the derived pointer. It works for both. First diffuse to the ghost vertices.
    for (u_int32_t ghosts_iterator = 0; ghosts_iterator < ghost_type::ghost_vertices_max_degree;
         ghosts_iterator++) {
        if (v->ghost_vertices[ghosts_iterator].has_value()) {
            cc.diffuse(Action(v->ghost_vertices[ghosts_iterator].value(),
                              addr,
                              actionType::application_action,
                              true,
                              args_for_ghost_vertices,
                              sssp_predicate,
                              sssp_work,
                              sssp_diffuse_predicate,
                              sssp_diffuse));
        }
    }

    for (int i = 0; i < v->number_of_edges; i++) {

        distance_to_send.distance = current_distance + v->edges[i].weight;
        ActionArgumentType const args_x =
            cca_create_action_argument<SSSPArguments>(distance_to_send);

        cc.diffuse(Action(v->edges[i].edge,
                          addr,
                          actionType::application_action,
                          true,
                          args_x,
                          sssp_predicate,
                          sssp_work,
                          sssp_diffuse_predicate,
                          sssp_diffuse));
    }

    return Closure(cc.null_false_event, nullptr);
}

inline auto
sssp_diffuse_func(ComputeCell& cc,
                  const Address addr,
                  actionType /* action_type_in */,
                  const ActionArgumentType args) -> Closure
{
    INVOKE_HANDLER_3(sssp_diffuse_T, cc, addr, args);
}

inline void
configure_parser(cli::Parser& parser)
{
    parser.set_required<std::string>("f", "graphfile", "Path to the input data graph file");
    parser.set_required<std::string>("g",
                                     "graphname",
                                     "Name of the input graph used to set the name of the output "
                                     "file. Example: Erdos or anything");
    parser.set_required<std::string>("s", "shape", "Shape of the compute cell");
    parser.set_optional<bool>("verify",
                              "verification",
                              0,
                              "Enable verification of the calculated paths with the paths provided "
                              "in the accompanying .sssp file");
    parser.set_required<u_int32_t>(
        "root", "sssproot", "Root vertex for Single Source Shortest Path (SSSP)");
    parser.set_optional<u_int32_t>("m",
                                   "memory_per_cc",
                                   1 * 512 * 1024,
                                   "Memory per compute cell in bytes. Default is 0.5 MB");
    parser.set_optional<std::string>(
        "od", "outputdirectory", "./", "Path to the output file directory. Default: ./");

    parser.set_optional<u_int32_t>(
        "hx",
        "htree_x",
        3,
        "Rows of Cells that are served by a single end Htree node. hx must be an odd value");
    parser.set_optional<u_int32_t>(
        "hy",
        "htree_y",
        5,
        "Columns of Cells that are served by a single end Htree node. hy must be an odd value");

    parser.set_optional<u_int32_t>("hdepth",
                                   "htree_depth",
                                   0,
                                   "Depth of the Htree. This is the size of the Htree. \n\t0: No "
                                   "Htree\n\t1: 1 Htree\n\t2: 5 Htrees "
                                   "as it recurssively constructs more...");

    parser.set_optional<u_int32_t>(
        "hb",
        "hbandwidth_max",
        64,
        "Max possible lanes in the htree joints. The htree is recursively constructred with end "
        "nodes having 4 lanes (for sqaure cells) to the joint. Then in the next joint there are 8, "
        "then 16 and so on. There needs to be a max value to avoid exponential growth.");

    parser.set_optional<u_int32_t>(
        "mesh", "mesh_type", 0, "Type of the mesh:\n\t0: Regular\n\t1: Torus.");

    parser.set_optional<u_int32_t>("route", "routing_policy", 0, "Routing algorithm to use.");

    parser.set_optional<bool>(
        "shuffle",
        "shuffle_vertices",
        0,
        "Randomly shuffle the vertex list so as to avoid any pattern in the graph based on vertex "
        "IDs. This appears to be the case for certain RMAT graphs.");
}

struct SSSPCommandLineArguments
{
    // SSSP root vertex
    u_int32_t root_vertex{};
    // Cross check the calculated sssp lengths from root with the lengths provided in .sssp file
    bool verify_results{};
    // Configuration related to the input data graph
    std::string input_graph_path;
    std::string graph_name;
    // Optional output directory path
    std::string output_file_directory;
    // Get the depth of Htree
    u_int32_t hdepth{};

    // Get the rows and columbs of cells that are served by a single end Htree node. This will
    // help in construction of the CCA chip, Htree, and routing
    u_int32_t hx{};
    u_int32_t hy{};

    // Get the max bandwidth of Htree
    u_int32_t hbandwidth_max{};

    // Configuration related to the CCA Chip
    std::string shape_arg;
    computeCellShape shape_of_compute_cells;

    // Get the memory per cc or use the default
    u_int32_t memory_per_cc{};

    // Mesh type.
    u_int32_t mesh_type{};

    // Get the routing policy to use
    u_int32_t routing_policy{};

    // To shuffle or to not shuffle the vertex ID list.
    bool shuffle_switch{};

    SSSPCommandLineArguments(cli::Parser& parser)
        : root_vertex(parser.get<u_int32_t>("root"))
        , verify_results(parser.get<bool>("verify"))
        , input_graph_path(parser.get<std::string>("f"))
        , graph_name(parser.get<std::string>("g"))
        , output_file_directory(parser.get<std::string>("od"))
        , hdepth(parser.get<u_int32_t>("hdepth"))
        , hx(parser.get<u_int32_t>("hx"))
        , hy(parser.get<u_int32_t>("hy"))
        , hbandwidth_max(parser.get<u_int32_t>("hb"))
        , shape_arg(parser.get<std::string>("s"))
        , shape_of_compute_cells(computeCellShape::computeCellShape_invalid)
        , memory_per_cc(parser.get<u_int32_t>("m"))
        , mesh_type(parser.get<u_int32_t>("mesh"))
        , routing_policy(parser.get<u_int32_t>("route"))
        , shuffle_switch(parser.get<bool>("shuffle"))
    {

        if (hdepth != 0) {
            if (!(hx % 2)) {
                std::cerr << "Invalid Input: hx must be odd! Provided value: " << hx << "\n";
                exit(0);
            }
            if (!(hy % 2)) {
                std::cerr << "Invalid Input: hy must be odd! Provided value: " << hy << "\n";
                exit(0);
            }
        }

        if (hdepth == 0) {
            hbandwidth_max = 0;
        }

        if (shape_arg == "square") {
            shape_of_compute_cells = computeCellShape::square;
        } else {
            std::cerr << "Error: Compute cell shape type " << shape_arg << " not supported.\n";
            exit(0);
        }
    }
};

template<typename NodeType>
inline void
verify_results(const SSSPCommandLineArguments& cmd_args,
               Graph<NodeType>& input_graph,
               const CCASimulator& cca_simulator)
{
    std::cout << "\nSingle Source Shortest Path Verification: \n";

    // Open the file containing sssp results for verification.
    std::string verfication_file_path = cmd_args.input_graph_path + ".sssp";
    std::ifstream file(verfication_file_path);

    if (!file.is_open()) {
        std::cout << "Failed to open the verification file: " << verfication_file_path << "\n";

    } else {

        std::vector<u_int32_t> control_results;
        std::string line;
        // Read the header.
        std::getline(file, line);
        // Read the root (source) of sssp that was used for results in the .sssp file.
        // Initialize it to an invalid value first.
        u_int32_t root_in_file = input_graph.total_vertices + 1;
        std::getline(file, line);
        if (!(std::istringstream(line) >> root_in_file)) {
            std::cerr << "Invalid root (source) value.\n";
            exit(0);
        }

        if (root_in_file != cmd_args.root_vertex) {
            std::cerr << "root vertex in file and root vertex used to run the program miss match. "
                         "Please use the same root in both for verification. Failed!\n";
            exit(0);
        }

        u_int32_t node_id;
        // u_int32_t sssp_value;
        double sssp_value;
        while (std::getline(file, line)) {

            std::istringstream iss(line);

            if (iss >> node_id >> sssp_value) {
                // When there are vertices with in-degree zero then they are not present in the
                // .sssp file. Therefore, we have to substitute its value with the undefined of
                // `max_distance` for the verification to work.
                while (node_id != control_results.size()) {
                    control_results.emplace_back(undefined_distance);
                }
                control_results.emplace_back(static_cast<u_int32_t>(sssp_value));
            } else {
                // Parsing failed.
                std::cerr << "Error parsing line: " << line
                          << ", in file: " << verfication_file_path << std::endl;
            }
        }

        file.close();

        u_int32_t total_errors = 0;
        for (u_int32_t i = 0; i < control_results.size(); i++) {

            // Check for correctness. Print the distance to a target test vertex. test_vertex

            Address const test_vertex_addr = input_graph.get_vertex_address_in_cca_rhizome(i);

            auto* v_test = static_cast<SSSPVertex<ghost_type_level_1>*>(
                cca_simulator.get_object(test_vertex_addr));

            // Assumes the result .sssp file is sorted.
            bool equal = control_results[i] == v_test->sssp_distance;
            if (!equal) {
                std::cout << "Vertex: " << i << ", Computed SSSP: " << v_test->sssp_distance
                          << ", Control Value: " << control_results[i] << ", Not equal! Error\n";
                total_errors++;
            }
        }

        if (total_errors > 0) {
            std::cout << ANSI_COLOR_RED << "Total number values error: " << total_errors
                      << ", Verification Failed\n"
                      << ANSI_COLOR_RESET;
        } else {
            std::cout << ANSI_COLOR_GREEN << "All values were correct. Verification Successful.\n"
                      << ANSI_COLOR_RESET;
        }
    }
}

// Write simulation statistics to a file
template<typename NodeType>
inline void
write_results(const SSSPCommandLineArguments& cmd_args,
              Graph<NodeType>& input_graph,
              CCASimulator& cca_simulator)
{

    std::string const output_file_name =
        "sssp_graph_" + cmd_args.graph_name + "_v_" + std::to_string(input_graph.total_vertices) +
        "_e_" + std::to_string(input_graph.total_edges) + "_rhizomes_" +
        std::to_string(rhizome_size) + "_rhizomecutoff_" +
        std::to_string(rhizome_inbound_degree_cutoff) + cca_simulator.key_configurations_string();

    std::string const output_file_path = cmd_args.output_file_directory + "/" + output_file_name;
    std::cout << "\nWriting results to output file: " << output_file_path << "\n";

    std::ofstream output_file(output_file_path);
    if (!output_file) {
        std::cerr << "Error! Output file not created\n";
    }

    // Output input graph details in the header of the statistics for us to know which input graph
    // it operated on.
    output_file << "graph_file\tvertices\tedges\troot_vertex\n"
                << cmd_args.input_graph_path << "\t" << input_graph.total_vertices << "\t"
                << input_graph.total_edges << "\t" << cmd_args.root_vertex << "\n";

    // Ask the simulator to print its statistics to the `output_file`.
    cca_simulator.print_statistics(output_file);

    // Close the output file
    output_file.close();

    // Print the animation of active status of each CC per cycle.
    cca_simulator.print_animation(output_file_path);
}

#endif // CCA_SSSP_Rhizome_HPP

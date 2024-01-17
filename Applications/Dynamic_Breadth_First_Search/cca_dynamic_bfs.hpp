/*
BSD 3-Clause License

Copyright (c) 2023, Bibrak Qamar

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

#ifndef CCA_BFS_HPP
#define CCA_BFS_HPP

#include "CCASimulator.hpp"
#include "Enums.hpp"

// Datastructures
#include "Graph.hpp"
#include "RecursiveParallelVertex.hpp"

#include "cmdparser.hpp"

// For memcpy()
#include <cstring>

#include <fstream>

inline static constexpr u_int32_t undefined_level = 999999;

// This is what the action carries as payload.
struct BFSArguments
{
    u_int32_t level;
    u_int32_t src_vertex_id;
};

// This is what the continuation for insert edge carries as payload.
struct BFSArgumentsEdgeInsertContinuation
{
    u_int32_t level;
    u_int32_t src_vertex_id; // maybe not needed.

    Address dst_vertex_addr;
    u_int32_t edge_weight;

    u_int32_t root_vertex;
};

template<typename Vertex_T>
struct BFSVertex : Vertex_T
{
    inline static constexpr u_int32_t max_level = undefined_level;
    u_int32_t bfs_level;

    BFSVertex(u_int32_t id_in, u_int32_t total_number_of_vertices_in)
        : bfs_level(BFSVertex::max_level)
    {
        this->id = id_in;
        this->number_of_edges = 0;
        this->total_number_of_vertices = total_number_of_vertices_in;
    }

    auto edge_insert_continuation_argument(Address dst_vertex_addr_in,
                                           u_int32_t edge_weight_in,
                                           u_int32_t root_vertex_in) -> ActionArgumentType
    {
        BFSArgumentsEdgeInsertContinuation arg_continuation;
        arg_continuation.level = this->bfs_level;
        arg_continuation.src_vertex_id = this->id;

        arg_continuation.dst_vertex_addr = dst_vertex_addr_in;
        arg_continuation.edge_weight = edge_weight_in;

        arg_continuation.root_vertex = root_vertex_in;

        /* std::cout << "edge_insert_continuation_argument: dst_vertex_addr_in: " <<
           dst_vertex_addr_in
                  << "\n"; */

        return cca_create_action_argument<BFSArgumentsEdgeInsertContinuation>(arg_continuation);
    }

    BFSVertex() {}
    ~BFSVertex() {}
};

// CCAFunctionEvent ids for the BFS action: predicate, work, and diffuse.
// In the main register the functions with the CCASimulator chip and get their ids.
extern CCAFunctionEvent dynamic_bfs_predicate;
extern CCAFunctionEvent dynamic_bfs_work;
extern CCAFunctionEvent dynamic_bfs_diffuse_predicate;
extern CCAFunctionEvent dynamic_bfs_diffuse;

extern CCAFunctionEvent dynamic_bfs_edge_insert_continuation;

inline auto
dynamic_bfs_predicate_func(ComputeCell& cc,
                           const Address& addr,
                           actionType /* action_type_in */,
                           const ActionArgumentType& args) -> int
{

    // First check whether this is a ghost vertex.If it is then always predicate true.
    // parent word is used in the sense that `RecursiveParallelVertex` is the parent class.
    auto* parent_recursive_parralel_vertex =
        static_cast<RecursiveParallelVertex<Address>*>(cc.get_object(addr));

    if (parent_recursive_parralel_vertex->is_ghost_vertex) {
        return 1;
    }

    auto* v = static_cast<BFSVertex<RecursiveParallelVertex<Address>>*>(cc.get_object(addr));
    BFSArguments const bfs_args = cca_get_action_argument<BFSArguments>(args);

    u_int32_t const incoming_level = bfs_args.level;

    if (v->bfs_level > incoming_level) {
        return 1;
    }
    return 0;
}

inline auto
dynamic_bfs_work_func(ComputeCell& cc,
                      const Address& addr,
                      actionType /* action_type_in */,
                      const ActionArgumentType& args) -> int
{
    // First check whether this is a ghost vertex. If it is then don't perform any work.
    // parent word is used in the sense that `RecursiveParallelVertex` is the parent class.
    auto* parent_recursive_parralel_vertex =
        static_cast<RecursiveParallelVertex<Address>*>(cc.get_object(addr));

    if (parent_recursive_parralel_vertex->is_ghost_vertex) {
        return 0;
    }

    auto* v = static_cast<BFSVertex<RecursiveParallelVertex<Address>>*>(cc.get_object(addr));
    BFSArguments const bfs_args = cca_get_action_argument<BFSArguments>(args);

    u_int32_t const incoming_level = bfs_args.level;

    /* if (v->id == 402) {
        std::cout << "dynamic_bfs_work_func: incoming_level: " << incoming_level
                  << ", src: " << bfs_args.src_vertex_id << ", old v->bfs_level: " << v->bfs_level
                  << "\n";
    } */

    // Update level with the new level
    v->bfs_level = incoming_level;
    return 0;
}

inline auto
dynamic_bfs_diffuse_predicate_func(ComputeCell& cc,
                                   const Address& addr,
                                   actionType /* action_type_in */,
                                   const ActionArgumentType& args) -> int
{
    // First check whether this is a ghost vertex. If it is then always predicate true.
    // parent word is used in the sense that `RecursiveParallelVertex` is the parent class.
    auto* parent_recursive_parralel_vertex =
        static_cast<RecursiveParallelVertex<Address>*>(cc.get_object(addr));

    if (parent_recursive_parralel_vertex->is_ghost_vertex) {
        return 1;
    }

    auto* v = static_cast<BFSVertex<RecursiveParallelVertex<Address>>*>(cc.get_object(addr));
    BFSArguments const bfs_args = cca_get_action_argument<BFSArguments>(args);

    u_int32_t const incoming_level = bfs_args.level;

    if (v->bfs_level == incoming_level) {
        return 1;
    }
    return 0;
}

inline auto
dynamic_bfs_diffuse_func(ComputeCell& cc,
                         const Address& addr,
                         actionType /* action_type_in */,
                         const ActionArgumentType& args) -> int
{

    // Get the hold of the parent ghost vertex. If it is ghost then simply perform diffusion.
    auto* parent_recursive_parralel_vertex =
        static_cast<RecursiveParallelVertex<Address>*>(cc.get_object(addr));
    bool this_is_ghost_vertex = parent_recursive_parralel_vertex->is_ghost_vertex;

    auto* v = static_cast<BFSVertex<RecursiveParallelVertex<Address>>*>(cc.get_object(addr));

    u_int32_t current_level = BFSVertex<RecursiveParallelVertex<Address>>::max_level;
    if (this_is_ghost_vertex) {
        BFSArguments const bfs_args = cca_get_action_argument<BFSArguments>(args);
        current_level = bfs_args.level;
    } else {
        current_level = v->bfs_level;
    }

    BFSArguments level_to_send;
    level_to_send.level = current_level;
    level_to_send.src_vertex_id = v->id;

    ActionArgumentType const args_for_ghost_vertices =
        cca_create_action_argument<BFSArguments>(level_to_send);

    // Note: The application vertex type is derived from the parent `RecursiveParallelVertex`
    // therefore using the derived pointer. It works for both. First diffuse to the ghost vertices.
    for (u_int32_t ghosts_iterator = 0;
         ghosts_iterator < RecursiveParallelVertex<Address>::ghost_vertices_max_degree;
         ghosts_iterator++) {
        if (v->ghost_vertices[ghosts_iterator].has_value()) {

            cc.diffuse(Action(v->ghost_vertices[ghosts_iterator].value(),
                              addr,
                              actionType::application_action,
                              true,
                              args_for_ghost_vertices,
                              dynamic_bfs_predicate,
                              dynamic_bfs_work,
                              dynamic_bfs_diffuse_predicate,
                              dynamic_bfs_diffuse));
        }
    }

    for (int i = 0; i < v->number_of_edges; i++) {

        level_to_send.level = current_level + 1;
        ActionArgumentType const args_x = cca_create_action_argument<BFSArguments>(level_to_send);

        cc.diffuse(Action(v->edges[i].edge,
                          addr,
                          actionType::application_action,
                          true,
                          args_x,
                          dynamic_bfs_predicate,
                          dynamic_bfs_work,
                          dynamic_bfs_diffuse_predicate,
                          dynamic_bfs_diffuse));
    }

    return 0;
}

inline auto
dynamic_bfs_edge_insert_continuation_func(ComputeCell& cc,
                                          const Address& addr,
                                          actionType /* action_type_in */,
                                          const ActionArgumentType& args) -> int
{

    BFSArgumentsEdgeInsertContinuation const bfs_args =
        cca_get_action_argument<BFSArgumentsEdgeInsertContinuation>(args);

    u_int32_t current_level = bfs_args.level;

    /* if (current_level == BFSVertex<RecursiveParallelVertex<Address>>::max_level) {
        std::cout << "undefined level Not diffusing the continuation\n";
        return 0;
    } */

    BFSArguments level_to_send;
    level_to_send.level = current_level + 1;
    level_to_send.src_vertex_id = bfs_args.src_vertex_id;

    ActionArgumentType const args_x = cca_create_action_argument<BFSArguments>(level_to_send);

    /* std::cout << "dynamic_bfs_edge_insert_continuation_func || vertex: "
              << level_to_send.src_vertex_id << ", level to send: " << level_to_send.level
              << ", bfs_args.dst_vertex_addr: " << bfs_args.dst_vertex_addr << "\n";
 */
    cc.diffuse(Action(bfs_args.dst_vertex_addr,
                      addr,
                      actionType::application_action,
                      true,
                      args_x,
                      dynamic_bfs_predicate,
                      dynamic_bfs_work,
                      dynamic_bfs_diffuse_predicate,
                      dynamic_bfs_diffuse));

    return 0;
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
    parser.set_required<u_int32_t>("root", "bfsroot", "Root vertex for Breadth First Search (BFS)");
    parser.set_optional<bool>(
        "verify",
        "verification",
        0,
        "Enable verification of the calculated levels with the levels provided "
        "in the accompanying .bfs file");
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

    parser.set_optional<u_int32_t>(
        "increments", "dynamic_increments", 0, "How many times does that graph evolve.");

    parser.set_optional<bool>(
        "shuffle",
        "shuffle_vertices",
        0,
        "Randomly shuffle the vertex list so as to avoid any pattern in the graph based on vertex "
        "IDs. This appears to be the case for certain RMAT graphs.");
}

struct BFSCommandLineArguments
{
    // BFS root vertex
    u_int32_t root_vertex{};
    // Cross check the calculated bfs lengths from root with the lengths provided in .bfs file
    bool verify_results{};
    // Configuration related to the input data graph
    std::string input_graph_path;
    std::string graph_name;
    // Optional output directory path
    std::string output_file_directory;
    // Get the depth of Htree.
    u_int32_t hdepth{};

    // Get the rows and columbs of cells that are served by a single end Htree node. This will
    // help in construction of the CCA chip, Htree, and routing.
    u_int32_t hx{};
    u_int32_t hy{};

    // Get the max bandwidth of Htree.
    u_int32_t hbandwidth_max{};

    // Configuration related to the CCA Chip.
    std::string shape_arg;
    computeCellShape shape_of_compute_cells;

    // Get the memory per cc or use the default.
    u_int32_t memory_per_cc{};

    // Mesh type.
    u_int32_t mesh_type{};

    // Get the routing policy to use.
    u_int32_t routing_policy{};

    // Increments to the graph for dynamic graphs
    u_int32_t increments{};

    // To shuffle or to not shuffle the vertex ID list.
    bool shuffle_switch{};

    BFSCommandLineArguments(cli::Parser& parser)
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
        , increments(parser.get<u_int32_t>("increments"))
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
verify_results(const BFSCommandLineArguments& cmd_args,
               Graph<NodeType>& input_graph,
               const CCASimulator& cca_simulator,
               const u_int32_t increment)
{
    std::cout << "\nDynamic Breadth First Search Verification: \n";

    // Open the file containing bfs results for verification.
    std::string input_graph_inc_path =
        cmd_args.input_graph_path + ".edgelist_" + std::to_string(increment);
    std::string verfication_file_path = input_graph_inc_path + ".bfs";
    std::ifstream file(verfication_file_path);

    if (!file.is_open()) {
        std::cout << "Failed to open the verification file: " << verfication_file_path << "\n";

    } else {

        std::vector<u_int32_t> control_results;
        std::string line;
        // Read the header.
        std::getline(file, line);
        // Read the root (source) of bfs that was used for results in the .bfs file.
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
        u_int32_t bfs_value;
        while (std::getline(file, line)) {

            std::istringstream iss(line);

            if (iss >> node_id >> bfs_value) {
                // When there are vertices with in-degree zero then they are not present in the .bfs
                // file. Therefore, we have to substitute its value with the undefined of
                // `max_level` for the verification to work.
                while (node_id != control_results.size()) {
                    control_results.emplace_back(undefined_level);
                }
                control_results.emplace_back(bfs_value);
            } else {
                // Parsing failed.
                std::cerr << "Error parsing line: " << line
                          << ", in file: " << verfication_file_path << std::endl;
            }
        }

        file.close();

        u_int32_t total_errors = 0;
        for (u_int32_t i = 0; i < control_results.size(); i++) {

            // Check for correctness. Print the level to a target test vertex. test_vertex

            Address const test_vertex_addr = input_graph.get_vertex_address_in_cca(i);

            auto* v_test = static_cast<BFSVertex<RecursiveParallelVertex<Address>>*>(
                cca_simulator.get_object(test_vertex_addr));

            // Assumes the result .bfs file is sorted.
            bool equal = control_results[i] == v_test->bfs_level;
            if (!equal) {
                std::cout << "Vertex: " << i << ", Computed BFS: " << v_test->bfs_level
                          << ", Control Value: " << control_results[i] << ", Not equal! Error\n";
                total_errors++;
            }
        }

        if (total_errors > 0) {
            std::cout << "Total number values error: " << total_errors << ", Verification Failed\n";
        } else {
            std::cout << "All values were correct. Verification Successful.\n";
        }
    }
}

template<typename NodeType>
inline void
write_results(const BFSCommandLineArguments& cmd_args,
              Graph<NodeType>& input_graph,
              CCASimulator& cca_simulator)
{
    // Write simulation statistics to a file
    std::string throttle_text = "OFF";
    if constexpr (throttling_switch) {
        throttle_text = "ON";
    }
    std::string termination_text = "OFF";
    if constexpr (termination_switch) {
        termination_text = "ON";
    }
    std::string network_text = "MESH";
    if (cmd_args.mesh_type == 1) {
        network_text = "TORUS";
    }

    std::string const output_file_name =
        "dynamic_bfs_square_x_" + std::to_string(cca_simulator.dim_x) + "_y_" +
        std::to_string(cca_simulator.dim_y) + "_graph_" + cmd_args.graph_name + "_v_" +
        std::to_string(input_graph.total_vertices) + "_e_" +
        std::to_string(input_graph.total_edges) + "_hb_" + std::to_string(cmd_args.hbandwidth_max) +
        "_th_" + throttle_text + "_recvbuff_" + std::to_string(RECVBUFFSIZE) + "_vicinity_" +
        std::to_string(vicinity_radius) + "_edges_max_" + std::to_string(edges_max) +
        "_termimation_" + termination_text + "_network_" + network_text;

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

    if constexpr (animation_switch) {
        // Write the active status animation data in a separate file.
        std::string const output_file_path_animation = output_file_path + "_active_animation";
        std::cout << "\nWriting active status animation data to output file: "
                  << output_file_path_animation << "\n";

        std::ofstream output_file_animation(output_file_path_animation);
        if (!output_file_animation) {
            std::cerr << "Error! Output file not created\n";
        }

        // Ask the simulator to print cell active status information per cycle to the
        // `output_file_animation`. This will be used mostly for animation purposes.
        cca_simulator.output_CCA_active_status_per_cell_cycle(output_file_animation);

        // Close the output file
        output_file_animation.close();
    }
}

#endif // CCA_BFS_HPP

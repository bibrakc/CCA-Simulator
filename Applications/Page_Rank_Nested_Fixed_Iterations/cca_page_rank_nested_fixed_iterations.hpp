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

#ifndef CCA_Page_Rank_Fixed_Iterations_HPP
#define CCA_Page_Rank_Fixed_Iterations_HPP

#include "CCASimulator.hpp"

// Datastructures
#include "Graph.hpp"
#include "RecursiveParallelVertex.hpp"

#include "cmdparser.hpp"

#include <algorithm>
#include <fstream>

// For memcpy()
#include <cstring>

inline constexpr double damping_factor = 0.85;
inline constexpr u_int32_t nested_iterations = NESTEDITERATIONS; // 10;

/* inline constexpr u_int32_t DEBUG_VERTEX = 17; */

// This is what the action carries as payload.
struct PageRankNestedFixedIterationsArguments
{
    double score;
    u_int32_t nested_iteration;
    u_int32_t src_vertex_id;
};

/* #define VERTEX_TYPE RecursiveParallelVertex */

// VERTEX_TYPE comes from the compiler -DVERTEX_TYPE argument.
template<typename Address_T>
using Vertex_Type = VERTEX_TYPE<Address_T>;

template<typename Vertex_T>
struct PageRankNestedFixedIterationsVertex : Vertex_T
{
    // This is the global iteration
    u_int32_t page_rank_current_iteration{};

    // This is the nested iteration count
    u_int32_t page_rank_current_nested_iteration{};

    struct Iteration
    {
        // When a diffusion occurs for a single iteration this is set to true so as to not diffuse
        // the same results everytime the vertex gets activated for a given iteration.
        bool has_current_iteration_diffused{};
        bool is_current_iteration_diffusion_ready{};

        // Score count from the inbound vertices.
        u_int32_t messages_received_count{};

        // The page rank score for this vertex for this iteration.
        double iteration_page_rank_score{};
        bool score_is_valid{};

        void reset()
        {
            this->has_current_iteration_diffused = false;
            this->is_current_iteration_diffusion_ready = false;
            this->messages_received_count = 0;
            this->iteration_page_rank_score = 0.0;
            this->score_is_valid = false;
        }
    };

    Iteration iterations[nested_iterations]{};

    // Counter to hold how many iterations this vertex has received completely.
    u_int32_t iterations_received_this_epoch{};

    // Use to check if the vertex goes into the next iteration by germinating an action in the
    // diffuse phase.
    bool start_next_iteration{};

    // Check to see if finished with all the nested iterations for this epoch.
    bool nested_epoch_completed{};

    // Final page rank score of this vertex for the iterative epoch.
    double page_rank_score{};
    u_int32_t page_rank_score_which_iteration_set_it{};

    // The page rank score used for diffusion. This act as a temporary since the score can be
    // updated but then for diffusion we need to sent the score of the previous iteration. So, it
    // acts to insure consistency.
    PageRankNestedFixedIterationsArguments args_for_diffusion{};

    // The page rank score for this vertex. This acts as a temporary to compute the new score.
    // double current_iteration_rank_score[nested_iterations]{};

    // Count to see if this vertex has recieved all messages (scores) so as to then compute the
    // final score and move to the next iteration.
    // current_iteration_incoming_count == inbound_degree
    // u_int32_t current_iteration_incoming_count[nested_iterations]{};

    PageRankNestedFixedIterationsVertex(u_int32_t id_in, u_int32_t total_number_of_vertices_in)
    {
        this->id = id_in;
        this->number_of_edges = 0;
        this->total_number_of_vertices = total_number_of_vertices_in;

        this->page_rank_score = 1.0 / this->total_number_of_vertices;
    }

    // Can be used to custom initialize the page rank score other than the default of (1.0 / N).
    void initialize_page_rank_score(double initial_page_rank_score)
    {
        this->page_rank_score = initial_page_rank_score;
    }

    PageRankNestedFixedIterationsVertex() = default;
    ~PageRankNestedFixedIterationsVertex() = default;
};

// CCAFunctionEvent ids for the Page Rank Fixed Iterations action: predicate, work, and diffuse.
// In the main register the functions with the CCASimulator chip and get their ids.
extern CCAFunctionEvent page_rank_nested_fixed_iterations_predicate;
extern CCAFunctionEvent page_rank_nested_fixed_iterations_work;
extern CCAFunctionEvent page_rank_nested_fixed_iterations_diffuse;

inline auto
page_rank_nested_fixed_iterations_predicate_func(ComputeCell& cc,
                                                 const Address& addr,
                                                 actionType /* action_type_in */,
                                                 const ActionArgumentType& /* args */) -> int
{
    // Set to always true. Since the idea is to accumulate the scores per iteration from all inbound
    // vertices.
    return 1;
}

inline auto
page_rank_nested_fixed_iterations_work_func(ComputeCell& cc,
                                            const Address& addr,
                                            actionType action_type_in,
                                            const ActionArgumentType& args) -> int
{

    // First check whether this is a ghost vertex. If it is then don't perform any work.
    auto* parent_recursive_parralel_vertex =
        static_cast<Vertex_Type<Address>*>(cc.get_object(addr));

    if (parent_recursive_parralel_vertex->is_ghost_vertex) {
        return 0;
    }

    auto* v = static_cast<PageRankNestedFixedIterationsVertex<Vertex_Type<Address>>*>(
        cc.get_object(addr));
    PageRankNestedFixedIterationsArguments const page_rank_args =
        cca_get_action_argument<PageRankNestedFixedIterationsArguments>(args);

    u_int32_t const iteration = page_rank_args.nested_iteration;

    // std::cout << "v->id: " << v->id << ", start work\n";

    bool const is_germinate = action_type_in == actionType::germinate_action;

    bool const is_first_message_of_the_epoch =
        ((v->iterations[iteration].messages_received_count == 0) && (iteration == 0));

    assert(iteration < nested_iterations && "Bug! Incoming Exceeds nested_iterations");

    /*     assert(v->iterations[iteration].messages_received_count < v->inbound_degree ||
               is_germinate && "Bug! current_iteration_incoming_count Exceeds v->inbound_degree");
     */

    if (v->iterations[iteration].messages_received_count >= v->inbound_degree && !is_germinate) {

        std::cout << "v->id: " << v->id << ", Bug! "
                  << ", messages_received_count[" << iteration
                  << "]: " << v->iterations[iteration].messages_received_count << " >  inbound "
                  << v->inbound_degree << "\n";

        /*   if (v->id == DEBUG_VERTEX) {
              std::cout << "\nstate of counters, v->page_rank_current_nested_iteration: "
                        << v->page_rank_current_nested_iteration
                        << ", page_rank_args.nested_iteration: " << iteration << std::endl;
              // Print values before setting to zero
              for (int i = 0; i < nested_iterations; i++) {

                  std::cout << "v->id: " << v->id << ", messages_received_count[" << i
                            << "]: " << v->iterations[i].messages_received_count
                            << ", iteration_page_rank_score[" << i
                            << "]: " << v->iterations[i].iteration_page_rank_score
                            << ", v->iterations_received_this_epoch: "
                            << v->iterations_received_this_epoch
                            << ", v->inbound_degree: " << v->inbound_degree << "\n";
              }
              std::cout << std::endl;
          } */
        exit(0);
    }

    if (is_germinate || is_first_message_of_the_epoch) {

        // Store the score of this iteration, which maybe used for diffusion.
        if (is_first_message_of_the_epoch) {
            v->nested_epoch_completed = false;
            v->args_for_diffusion.score =
                v->page_rank_score / static_cast<double>(v->outbound_degree);
        } else {

            u_int32_t const previous_iteration = iteration - 1;
            v->args_for_diffusion.score =
                v->iterations[previous_iteration].iteration_page_rank_score /
                static_cast<double>(v->outbound_degree);
        }

        v->args_for_diffusion.src_vertex_id = v->id;
        v->args_for_diffusion.nested_iteration = iteration;

        v->iterations[iteration].is_current_iteration_diffusion_ready = true;
    }

    // If the action comes from the host and is germinate action then don't update scores and
    // just treat it as a purely diffusive action.
    if (action_type_in == actionType::application_action) {

        // Update partial new score with the new incoming score.
        v->iterations[iteration].iteration_page_rank_score += page_rank_args.score;
        v->iterations[iteration].messages_received_count++;
    }

    // If this is the last message for this iteration then update the score. And set the stage for
    // the germination of the next iteration.
    if ((v->iterations[iteration].messages_received_count == v->inbound_degree) &&
        (!is_germinate || (v->inbound_degree == 0))) {

        // Update the page rank score.
        v->iterations[iteration].iteration_page_rank_score =
            ((1.0 - damping_factor) / static_cast<double>(v->total_number_of_vertices)) +
            (damping_factor * v->iterations[iteration].iteration_page_rank_score);

        assert(iteration >= v->page_rank_score_which_iteration_set_it &&
               "BUG! Lower iteration setting score but a higher iteration already set it!");
        // Update the score to the current iteration score.

        v->page_rank_score = v->iterations[iteration].iteration_page_rank_score;
        v->page_rank_score_which_iteration_set_it = iteration;
        v->iterations[iteration].score_is_valid = true;
        if (v->inbound_degree == 0) {
            v->page_rank_score =
                ((1.0 - damping_factor) / static_cast<double>(v->total_number_of_vertices));
        }

        // Update the count for how many total iterations have been received and scores set for this
        // iterative epoch.
        v->iterations_received_this_epoch++;
        // Increament the global iteration count.
        v->page_rank_current_iteration++;

        // TODO: Such a logic can be used to store the state of how many msg have been received in
        // the next iteration. Therefore action overlap or iterative overlap due to asynchrony.

        /*   if (v->id == DEBUG_VERTEX) {
              std::cout << "\nstate of counters, v->page_rank_current_nested_iteration: "
                        << v->page_rank_current_nested_iteration
                        << ", page_rank_args.nested_iteration: " << iteration << std::endl;
              // Print values before setting to zero
              for (int i = 0; i < nested_iterations; i++) {

                  std::cout << "v->id: " << v->id << ", messages_received_count[" << i
                            << "]: " << v->iterations[i].messages_received_count
                            << ", iteration_page_rank_score[" << i
                            << "]: " << v->iterations[i].iteration_page_rank_score
                            << ", v->iterations_received_this_epoch: "
                            << v->iterations_received_this_epoch
                            << ", v->inbound_degree: " << v->inbound_degree
                            << ", number_of_edges: " << v->number_of_edges
                            << ", number_of_edges_in_this_recurssive_tree: "
                            << v->number_of_edges_in_this_recurssive_tree << "\n";
              }
              std::cout << std::endl;
          } */

        // Go to the next nested iteration.
        v->page_rank_current_nested_iteration++;

        v->start_next_iteration = true;
    }

    // std::cout << "v->id: " << v->id << ", end work\n";
    return 0;
}

inline auto
page_rank_nested_fixed_iterations_diffuse_func(ComputeCell& cc,
                                               const Address& addr,
                                               actionType /* action_type_in */,
                                               const ActionArgumentType& args) -> int
{
    auto* v = static_cast<PageRankNestedFixedIterationsVertex<Vertex_Type<Address>>*>(
        cc.get_object(addr));

    PageRankNestedFixedIterationsArguments const page_rank_args =
        cca_get_action_argument<PageRankNestedFixedIterationsArguments>(args);

    u_int32_t const iteration = page_rank_args.nested_iteration;
    // v->args_for_diffusion.nested_iteration; //;

    // std::cout << "v->id: " << v->id << ", start diffuse\n";

    // If the diffusion has not occured for the current iteration then diffuse.
    bool should_diffuse = !v->iterations[iteration].has_current_iteration_diffused &&
                          v->iterations[iteration].is_current_iteration_diffusion_ready &&
                          !v->nested_epoch_completed;

    // Get the hold of the parent ghost vertex. If it is ghost then simply perform diffusion.
    auto* parent_recursive_parralel_vertex =
        static_cast<Vertex_Type<Address>*>(cc.get_object(addr));

    bool this_is_ghost_vertex = parent_recursive_parralel_vertex->is_ghost_vertex;

    ActionArgumentType args_x = nullptr;
    if (this_is_ghost_vertex) {
        // Just relay what parent gave.
        args_x = args;
    } else {
        // This is the parent non-ghost vertex so it makes its arguments.
        args_x = cca_create_action_argument<PageRankNestedFixedIterationsArguments>(
            v->args_for_diffusion);
    }

    if (should_diffuse || this_is_ghost_vertex) {

        // Note: The application vertex type is derived from the parent `Vertex_Type` therefore
        // using the derived pointer. It works for both.
        // First diffuse to the ghost vertices.
        for (u_int32_t ghosts_iterator = 0;
             ghosts_iterator < Vertex_Type<Address>::ghost_vertices_max_degree;
             ghosts_iterator++) {
            if (v->ghost_vertices[ghosts_iterator].has_value()) {

                cc.diffuse(Action(v->ghost_vertices[ghosts_iterator].value(),
                                  addr,
                                  actionType::application_action,
                                  true,
                                  args_x,
                                  page_rank_nested_fixed_iterations_predicate,
                                  page_rank_nested_fixed_iterations_work,
                                  page_rank_nested_fixed_iterations_diffuse));
            }
        }

        // Now diffuse along the edges.
        for (int i = 0; i < v->number_of_edges; i++) {

            /*   if (this_is_ghost_vertex) {
                  if (v->edges[i].edge.cc_id == 17)
                      std::cout << "\t" << v->id
                                << ": is ghost and is diffusing edge: " << v->edges[i].edge
                                << ", CC id: " << v->edges[i].edge.cc_id << "\n";
              } else {

                  if (v->edges[i].edge.cc_id == 17)
                      std::cout << "\t" << v->id
                                << ": is parent and is diffusing edge: " << v->edges[i].edge
                                << ", CC id: " << v->edges[i].edge.cc_id << "\n";
              } */
            // v->args_for_diffusion.nested_iteration = iteration;

            // Diffuse.
            cc.diffuse(Action(v->edges[i].edge,
                              addr,
                              actionType::application_action,
                              true,
                              args_x,
                              page_rank_nested_fixed_iterations_predicate,
                              page_rank_nested_fixed_iterations_work,
                              page_rank_nested_fixed_iterations_diffuse));
        }

        if (!this_is_ghost_vertex) {
            v->iterations[iteration].has_current_iteration_diffused = true;
        }
    }

    if (this_is_ghost_vertex) {
        return 0;
    }

    // If this is germinate then prepare and germinate the next iteration.
    u_int32_t next_iteration = iteration + 1;
    if ((next_iteration < nested_iterations) && !v->nested_epoch_completed &&
        v->start_next_iteration) {

        v->start_next_iteration = false;

        // Germinate action onto ownself.
        v->args_for_diffusion.nested_iteration = next_iteration;
        v->args_for_diffusion.score = 0;             // This wont be used.
        v->args_for_diffusion.src_vertex_id = v->id; // This won't be used.

        ActionArgumentType const args_x =
            cca_create_action_argument<PageRankNestedFixedIterationsArguments>(
                v->args_for_diffusion);
        // Diffuse.
        cc.diffuse(Action(addr, // ownself
                          addr, // from myself
                          actionType::germinate_action,
                          true,
                          args_x,
                          page_rank_nested_fixed_iterations_predicate,
                          page_rank_nested_fixed_iterations_work,
                          page_rank_nested_fixed_iterations_diffuse));
    }

    // Finish the epoch. Reset environment for the next epoch.
    if (v->iterations_received_this_epoch == nested_iterations &&
        v->iterations[nested_iterations - 1].has_current_iteration_diffused) {

        /* if (v->id == 17) {
            std::cout << "\t" << v->id << ": is terminating epoch:\n";
        } */

        v->nested_epoch_completed = true;
        for (u_int32_t i = 0; i < nested_iterations; i++) {
            v->iterations[i].reset();
        }
        // Reset
        v->page_rank_current_nested_iteration = 0;
        v->iterations_received_this_epoch = 0;
        v->page_rank_score_which_iteration_set_it = 0;
        v->start_next_iteration = false;
    }

    // std::cout << "v->id: " << v->id << ", end diffuse\n";
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
    parser.set_required<u_int32_t>("root",
                                   "page_rank_nested_fixed_iterations_root",
                                   "Root vertex where to germinate action for Page Rank (Page Rank "
                                   "Fixed Iterations). Makes no difference to the results.");
    parser.set_optional<bool>("verify",
                              "verification",
                              0,
                              "Enable verification of the calculated score with the score provided "
                              "in the accompanying .pagerank file");
    parser.set_optional<u_int32_t>(
        "iter", "iterations", 20, "Number of fixed iterations to perform");
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

struct PageRankNestedIterationCommandLineArguments
{
    // Total max depth iterations to perform.
    u_int32_t total_iterations{};
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

    PageRankNestedIterationCommandLineArguments(cli::Parser& parser)
        : total_iterations(parser.get<u_int32_t>("iter"))
        , root_vertex(parser.get<u_int32_t>("root"))
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
verify_results(const PageRankNestedIterationCommandLineArguments& cmd_args,
               Graph<NodeType>& input_graph,
               const CCASimulator& cca_simulator)
{
    std::cout << "\nPage Rank Nested Fixed Iterations Verification: \n";

    // Open the file containing pagerank results for verification.
    std::string verfication_file_path = cmd_args.input_graph_path + ".pagerank";
    std::ifstream file(verfication_file_path);

    if (!file.is_open()) {
        std::cout << "Failed to open the verification file: " << verfication_file_path << "\n";

    } else {

        std::vector<double> control_results;
        std::string line;
        int node_id;
        double pagerank_value;
        while (std::getline(file, line)) {
            
            // When there are vertices with in-degree zero then they are not present in the
            // .pagerank file. Therefore, the vertification will fail in that case.
            // TODO: Need to add the case.
            if (std::sscanf(line.c_str(), "%d\t%lf", &node_id, &pagerank_value) == 2) {
                control_results.emplace_back(pagerank_value);
            }
        }

        file.close();

        // Print the stored node and rank values
        /*  for (u_int32_t i = 0; i < control_results.size(); i++) {
             std::cout << "Node: " << i << ", Pagerank: " << control_results[i] << "\n";
         } */
        double tolerance = 0.00001;
        u_int32_t total_values_exceeding_tolerance = 0;
        for (u_int32_t i = 0; i < input_graph.total_vertices; i++) {

            // Check for correctness. Print the distance to a target test vertex. test_vertex
            Address const test_vertex_addr = input_graph.get_vertex_address_in_cca(i);

            auto* v_test = static_cast<PageRankNestedFixedIterationsVertex<Vertex_Type<Address>>*>(
                cca_simulator.get_object(test_vertex_addr));
            double difference = std::fabs(control_results[i] - v_test->page_rank_score);
            if (difference > tolerance) {
                std::cout << "Vertex: " << i << ", Computed Pagerank: " << v_test->page_rank_score
                          << ", Control Value: " << control_results[i]
                          << ", Exceeds tolerance. Difference: " << difference
                          << ", page_rank_current_iteration: "
                          << v_test->page_rank_current_iteration << "\n";

                total_values_exceeding_tolerance++;
            }
        }

        if (total_values_exceeding_tolerance > 0) {
            std::cout << "Total number values that exceeded tolerance: "
                      << total_values_exceeding_tolerance << ", Verification Failed\n";
        } else {
            std::cout << "All values were within tolerance. Verification Successful.\n";
        }
    }
}

template<typename NodeType>
inline void
write_results(const PageRankNestedIterationCommandLineArguments& cmd_args,
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

    std::string const output_file_name =
        "page_rank_nested_fixed_iterations_square_x_" + std::to_string(cca_simulator.dim_x) +
        "_y_" + std::to_string(cca_simulator.dim_y) + "_graph_" + cmd_args.graph_name + "_v_" +
        std::to_string(input_graph.total_vertices) + "_e_" +
        std::to_string(input_graph.total_edges) + "_hb_" + std::to_string(cmd_args.hbandwidth_max) +
        "_th_" + throttle_text + "_recvbuff_" + std::to_string(RECVBUFFSIZE) + "_vicinity_" +
        std::to_string(vicinity_radius) + "_edges_max_" + std::to_string(edges_max) +
        "_termimation_" + termination_text + "_nested_iter_" + std::to_string(nested_iterations);

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

#endif // CCA_Page_Rank_Fixed_Iterations_HPP

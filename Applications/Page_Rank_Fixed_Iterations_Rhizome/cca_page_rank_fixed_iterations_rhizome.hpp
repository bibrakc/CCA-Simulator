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

#ifndef CCA_Page_Rank_Fixed_Iterations_Rhizome_HPP
#define CCA_Page_Rank_Fixed_Iterations_Rhizome_HPP

#include "CCASimulator.hpp"
#include "Enums.hpp"
#include "LCO.hpp"
#include "LCO_AND.hpp"

// Datastructures
#include "Graph.hpp"
#include "RhizomeRecursiveParallelVertex.hpp"

#include "cmdparser.hpp"

// For memcpy()
#include <cstring>

inline constexpr double damping_factor = 0.85;

// This is what the action carries as payload.
struct PageRankFixedIterationsArguments
{
    double score;

    // For debuging
    u_int32_t iteration;
    u_int32_t src_vertex_id;
};

/* template<typename Address_T>
struct PageRankFixedIterationsSimpleVertex : SimpleVertex<Address_T> */
template<typename Vertex_T>
struct PageRankFixedIterationsVertex : Vertex_T
{
    u_int32_t page_rank_current_iteration{};

    // When a diffusion occurs for a single iteration this is set to true so as to not diffuse the
    // same results everytime the vertex gets activated for a given iteration.
    bool has_current_iteration_diffused{};

    // The page rank score for this vertex.
    // This becomes AND LCO. TODO: Address score_AND_LCO ?? in the purest sense.
    LCO<double, LCO_AND> page_rank_current_rank_score;

    // The page rank score for this vertex. This acts as a temporary to compute the new score.
    double current_iteration_rank_score{};

    // Count to see if this vertex has recieved all messages (scores) so as to then compute the
    // final score and move to the next iteration.
    // current_iteration_incoming_count == inbound_degree
    u_int32_t current_iteration_incoming_count{};

    PageRankFixedIterationsVertex(u_int32_t id_in, u_int32_t total_number_of_vertices_in)
        : page_rank_current_rank_score(1.0 / total_number_of_vertices_in) // LCO constructor
    {
        this->id = id_in;
        this->number_of_edges = 0;
        this->total_number_of_vertices = total_number_of_vertices_in;
    }

    // Custom initialize the page rank score other than the default of (1.0 / N).
    void initialize_page_rank_score(double initial_page_rank_score)
    {
        this->page_rank_current_rank_score.set_val(initial_page_rank_score);
    }

    // Each time this is called it increaments the N of LCO_AND. It means that it will be called
    // when the Rhizome Vertex is first created (in init()) and then everytime there is a
    // .set_rhizome() called as it is linked to other rhizomes.
    void configure_derived_class_LCOs() { this->page_rank_current_rank_score.lco.N++; }

    PageRankFixedIterationsVertex() = default;
    ~PageRankFixedIterationsVertex() = default;
};

// CCAFunctionEvent ids for the Page Rank Fixed Iterations action: predicate, work, and diffuse.
// In the main register the functions with the CCASimulator chip and get their ids.
extern CCAFunctionEvent page_rank_fixed_iterations_predicate;
extern CCAFunctionEvent page_rank_fixed_iterations_germinate_work;
extern CCAFunctionEvent page_rank_fixed_iterations_work;
extern CCAFunctionEvent page_rank_fixed_iterations_diffuse_predicate;
extern CCAFunctionEvent page_rank_fixed_iterations_diffuse;

extern CCAFunctionEvent page_rank_fixed_iterations_rhizome_collapse;
extern CCAFunctionEvent page_rank_fixed_iterations_rhizome_collapse_diffuse;

inline auto
page_rank_fixed_iterations_predicate_func(ComputeCell& cc,
                                          const Address addr,
                                          actionType /* action_type_in */,
                                          const ActionArgumentType args) -> Closure
{
    // Set to always true. Since the idea is to accumulate the scores per iteration from all inbound
    // vertices.
    return Closure(cc.null_true_event, nullptr);
}

inline auto
page_rank_fixed_iterations_germinate_work_func(ComputeCell& cc,
                                               const Address addr,
                                               actionType action_type_in,
                                               const ActionArgumentType args) -> Closure
{
    // Validity checks
    // This action must be germinate_action.
    assert(action_type_in == actionType::germinate_action);

    // Make sure this is not a ghost vertex.
    // parent word is used in the sense that `RhizomeRecursiveParallelVertex` is the parent class.
    auto* parent_recursive_parralel_vertex =
        static_cast<RhizomeRecursiveParallelVertex<Address>*>(cc.get_object(addr));
    assert(!parent_recursive_parralel_vertex->is_ghost_vertex);

    auto* v = static_cast<PageRankFixedIterationsVertex<RhizomeRecursiveParallelVertex<Address>>*>(
        cc.get_object(addr));

    PageRankFixedIterationsArguments my_score_to_send;

    my_score_to_send.score =
        v->page_rank_current_rank_score.get_val() / static_cast<double>(v->outbound_degree);
    my_score_to_send.src_vertex_id = v->id;

    // For debugging
    my_score_to_send.iteration = v->page_rank_current_iteration;

    ActionArgumentType const args_diffuse_closure =
        cca_create_action_argument<PageRankFixedIterationsArguments>(my_score_to_send);

    v->has_current_iteration_diffused = true;

    // Reset. This is a special case when the vertex has zero in-degree.
    if (0 == v->inbound_degree) {

        // Update the page rank score.
        v->page_rank_current_rank_score.set_val(
            ((1.0 - damping_factor) / static_cast<double>(v->total_number_of_vertices)));

        // Reset.
        v->current_iteration_rank_score = 0.0;
        v->current_iteration_incoming_count = 0;
        v->has_current_iteration_diffused = false;

        // Increament the global iteration count.
        v->page_rank_current_iteration++;
    }

    // Return diffuse closure
    return Closure(cc.null_true_event, args_diffuse_closure);
}

inline auto
page_rank_fixed_iterations_work_func(ComputeCell& cc,
                                     const Address addr,
                                     actionType action_type_in,
                                     const ActionArgumentType args) -> Closure
{
    // This action must be application_action.
    assert(action_type_in == actionType::application_action);

    // First check whether this is a ghost vertex. If it is then don't perform any work.
    // parent word is used in the sense that `RhizomeRecursiveParallelVertex` is the parent class.
    auto* parent_recursive_parralel_vertex =
        static_cast<RhizomeRecursiveParallelVertex<Address>*>(cc.get_object(addr));

    if (parent_recursive_parralel_vertex->is_ghost_vertex) {
        return Closure(cc.null_true_event, nullptr);
    }

    auto* v = static_cast<PageRankFixedIterationsVertex<RhizomeRecursiveParallelVertex<Address>>*>(
        cc.get_object(addr));

    PageRankFixedIterationsArguments const page_rank_args =
        cca_get_action_argument<PageRankFixedIterationsArguments>(args);

    // Update partial new score with the new incoming score.
    v->current_iteration_rank_score += page_rank_args.score;
    v->current_iteration_incoming_count++;

    Closure diffuse_closure_to_return(cc.null_false_event, nullptr);

    if (!v->has_current_iteration_diffused) {

        PageRankFixedIterationsArguments my_score_to_send;

        my_score_to_send.score =
            v->page_rank_current_rank_score.get_val() / static_cast<double>(v->outbound_degree);
        // For debugging
        my_score_to_send.iteration = v->page_rank_current_iteration;
        my_score_to_send.src_vertex_id = v->id;

        ActionArgumentType const args_diffuse_closure =
            cca_create_action_argument<PageRankFixedIterationsArguments>(my_score_to_send);

        v->has_current_iteration_diffused = true;
        // Return diffuse closure
        diffuse_closure_to_return = Closure(cc.null_true_event, args_diffuse_closure);
    }

    // Reset.
    // TODO: This needs to be all_reduced before being set.!!!
    if (v->current_iteration_incoming_count == v->inbound_degree) {

        // rhizome_collapse(CCA_PLUS_OP, v->page_rank_current_rank_score, lambda);

        PageRankFixedIterationsArguments my_partial_score_to_send;

        my_partial_score_to_send.score = v->current_iteration_rank_score;
        // For debugging
        my_partial_score_to_send.iteration = v->page_rank_current_iteration;
        my_partial_score_to_send.src_vertex_id = v->id;

        ActionArgumentType const args_rhizome_collapse =
            cca_create_action_argument<PageRankFixedIterationsArguments>(my_partial_score_to_send);

        Action rhizome_collapse_page_rank(addr,
                                          addr,
                                          actionType::application_action,
                                          true,
                                          args_rhizome_collapse,
                                          cc.null_true_event,
                                          page_rank_fixed_iterations_rhizome_collapse,
                                          cc.null_true_event,
                                          page_rank_fixed_iterations_rhizome_collapse_diffuse);
        // Not a very nice way to use system level api, of inserting action, in user code. The user
        // should return closure that is then added to the queue by the rutime. But for now we
        // will live with this instead of refactoring the code to allow two or more closures to
        // be returned from a single work function.
        if (!cc.insert_action(rhizome_collapse_page_rank, false)) {
            std::cerr << "rhizome collapse can not be inserted in action queue!" << std::endl;
            exit(0);
        }
    }

    return diffuse_closure_to_return;
}

inline auto
page_rank_fixed_iterations_rhizome_collapse_func(ComputeCell& cc,
                                                 const Address addr,
                                                 actionType action_type_in,
                                                 const ActionArgumentType args) -> Closure
{

    // This action must not be invalid.
    assert(action_type_in != actionType::invalid_action);

    // First check whether this is a ghost vertex. If it is then its a bug.
    auto* parent_recursive_parralel_vertex =
        static_cast<RhizomeRecursiveParallelVertex<Address>*>(cc.get_object(addr));

    if (parent_recursive_parralel_vertex->is_ghost_vertex) {
        std::cerr << "Bug! rhizome collapse can not happen on a ghost vertex." << std::endl;
        exit(0);
    }

    auto* v = static_cast<PageRankFixedIterationsVertex<RhizomeRecursiveParallelVertex<Address>>*>(
        cc.get_object(addr));

    PageRankFixedIterationsArguments const page_rank_args =
        cca_get_action_argument<PageRankFixedIterationsArguments>(args);

    // Accumulate the score.
    v->page_rank_current_rank_score.lco += page_rank_args.score;

    // trigger action when count == N
    // TODO: This is very hardcode way of triggering the action. In an actual setting it should be
    // diffused and inserted into this CC and then the system schedules it.
    if (v->page_rank_current_rank_score.lco.increment()) {

        // Update the page rank score.
        v->page_rank_current_rank_score.set_val(
            ((1.0 - damping_factor) / static_cast<double>(v->total_number_of_vertices)) +
            (damping_factor * v->page_rank_current_rank_score.lco.local_val));

        // Reset.
        v->page_rank_current_rank_score.lco.reset();
        v->current_iteration_rank_score = 0.0;
        v->current_iteration_incoming_count = 0;
        v->has_current_iteration_diffused = false;

        // Increament the global iteration count.
        v->page_rank_current_iteration++;
    }

    Closure diffuse_closure_to_return(cc.null_true_event, nullptr);

    return diffuse_closure_to_return;
}

inline auto
page_rank_fixed_iterations_rhizome_collapse_diffuse_func(ComputeCell& cc,
                                                         const Address addr,
                                                         actionType /* action_type_in */,
                                                         const ActionArgumentType args) -> Closure
{

    auto* v = static_cast<PageRankFixedIterationsVertex<RhizomeRecursiveParallelVertex<Address>>*>(
        cc.get_object(addr));

    /*  PageRankFixedIterationsArguments const page_rank_args =
         cca_get_action_argument<PageRankFixedIterationsArguments>(args); */

    // Relay to the Rhizome link
    for (u_int32_t rhizome_iterator = 0;
         rhizome_iterator <
         PageRankFixedIterationsVertex<
             RhizomeRecursiveParallelVertex<Address>>::rhizome_vertices_max_degree;
         rhizome_iterator++) {

        if (v->rhizome_vertices[rhizome_iterator].has_value()) {
            cc.diffuse(Action(v->rhizome_vertices[rhizome_iterator].value(),
                              addr,
                              actionType::application_action,
                              true,
                              args,
                              cc.null_true_event,
                              page_rank_fixed_iterations_rhizome_collapse,
                              cc.null_false_event,
                              cc.null_false_event));
        }
    }

    return Closure(cc.null_false_event, nullptr);
}

inline auto
page_rank_fixed_iterations_diffuse_predicate_func(ComputeCell& cc,
                                                  const Address addr,
                                                  actionType /* action_type_in */,
                                                  const ActionArgumentType /*args*/) -> Closure
{
    return Closure(cc.null_true_event, nullptr);
}

inline auto
page_rank_fixed_iterations_diffuse_func(ComputeCell& cc,
                                        const Address addr,
                                        actionType /* action_type_in */,
                                        const ActionArgumentType args) -> Closure
{

    auto* v = static_cast<PageRankFixedIterationsVertex<RhizomeRecursiveParallelVertex<Address>>*>(
        cc.get_object(addr));

    // Note: The application vertex type is derived from the parent `RhizomeRecursiveParallelVertex`
    // therefore using the derived pointer. It works for both. First diffuse to the ghost vertices.
    for (u_int32_t ghosts_iterator = 0;
         ghosts_iterator < RhizomeRecursiveParallelVertex<Address>::ghost_vertices_max_degree;
         ghosts_iterator++) {
        if (v->ghost_vertices[ghosts_iterator].has_value()) {

            cc.diffuse(Action(v->ghost_vertices[ghosts_iterator].value(),
                              addr,
                              actionType::application_action,
                              true,
                              args,
                              cc.null_true_event, // page_rank_fixed_iterations_predicate,
                              cc.null_true_event, // page_rank_fixed_iterations_work,
                              page_rank_fixed_iterations_diffuse_predicate,
                              page_rank_fixed_iterations_diffuse));
        }
    }

    for (int i = 0; i < v->number_of_edges; i++) {

        // Diffuse.
        cc.diffuse(Action(v->edges[i].edge,
                          addr,
                          actionType::application_action,
                          true,
                          args,
                          page_rank_fixed_iterations_predicate,
                          page_rank_fixed_iterations_work,
                          page_rank_fixed_iterations_diffuse_predicate,
                          page_rank_fixed_iterations_diffuse));
    }

    return Closure(cc.null_false_event, nullptr);
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
                                   "page_rank_fixed_iterations_root",
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

struct PageRankFixedIterationsCommandLineArguments
{
    // Which vertex diffuses the seed action? root vertex.
    u_int32_t root_vertex{};
    // How many iterations.
    u_int32_t iter{};
    // Whether to cross check the calculated page rank in .pagerank file.
    bool verify_results{};
    // Configuration related to the input data graph.
    std::string input_graph_path;
    std::string graph_name;
    // Optional output directory path.
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

    // To shuffle or to not shuffle the vertex ID list.
    bool shuffle_switch{};

    PageRankFixedIterationsCommandLineArguments(cli::Parser& parser)
        : root_vertex(parser.get<u_int32_t>("root"))
        , iter(parser.get<u_int32_t>("iter"))
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
verify_results(const PageRankFixedIterationsCommandLineArguments& cmd_args,
               Graph<NodeType>& input_graph,
               const CCASimulator& cca_simulator)
{

    std::cout << "\nPage Rank Fixed Iterations Verification: \n";

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
            Address const test_vertex_addr = input_graph.get_vertex_address_in_cca_rhizome(i);

            auto* v_test = static_cast<
                PageRankFixedIterationsVertex<RhizomeRecursiveParallelVertex<Address>>*>(
                cca_simulator.get_object(test_vertex_addr));
            double difference =
                std::fabs(control_results[i] - v_test->page_rank_current_rank_score.get_val());
            if (difference > tolerance) {
                std::cout << "Vertex: " << i << ", Computed Pagerank: "
                          << v_test->page_rank_current_rank_score.get_val()
                          << ", Control Value: " << control_results[i]
                          << ", Exceeds tolerance. Difference: " << difference << "\n";

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

// Write simulation statistics to a file.
template<typename NodeType>
inline void
write_results(const PageRankFixedIterationsCommandLineArguments& cmd_args,
              Graph<NodeType>& input_graph,
              CCASimulator& cca_simulator)
{

    std::string const output_file_name = "pagerank_graph_" + cmd_args.graph_name + "_v_" +
                                         std::to_string(input_graph.total_vertices) + "_e_" +
                                         std::to_string(input_graph.total_edges) + "_rhizomes_" +
                                         std::to_string(rhizome_size) + "_rhizomecutoff_" +
                                         std::to_string(rhizome_inbound_degree_cutoff) +
                                         cca_simulator.key_configurations_string();

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

#endif // CCA_Page_Rank_Fixed_Iterations_Rhizome_HPP

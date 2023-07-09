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
#include "SimpleVertex.hpp"

#include "cmdparser.hpp"

#include <algorithm>
// For memcpy()
#include <cstring>

inline constexpr double damping_factor = 0.85;
inline constexpr u_int32_t nested_iterations = 6;

inline constexpr u_int32_t DEBUG_VERTEX = 126;

// This is what the action carries as payload.
struct PageRankNestedFixedIterationsArguments
{
    double score;
    u_int32_t nested_iteration;
    u_int32_t src_vertex_id;
};

template<typename Address_T>
struct PageRankNestedFixedIterationsSimpleVertex : SimpleVertex<Address_T>
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

    PageRankNestedFixedIterationsSimpleVertex(u_int32_t id_in,
                                              u_int32_t total_number_of_vertices_in)
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

    PageRankNestedFixedIterationsSimpleVertex() = default;
    ~PageRankNestedFixedIterationsSimpleVertex() = default;
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
    auto* v = static_cast<PageRankNestedFixedIterationsSimpleVertex<Address>*>(cc.get_object(addr));

    PageRankNestedFixedIterationsArguments const page_rank_args =
        cca_get_action_argument<PageRankNestedFixedIterationsArguments>(args);

    u_int32_t const iteration = page_rank_args.nested_iteration;

    bool const is_germinate = action_type_in == actionType::germinate_action;

    bool const is_first_message_of_the_epoch =
        ((v->iterations[iteration].messages_received_count == 0) && (iteration == 0));

    assert(iteration < nested_iterations && "Bug! Incoming Exceeds nested_iterations");

    /*  assert(v->iterations[iteration].messages_received_count < v->inbound_degree && !is_germinate
       && "Bug! current_iteration_incoming_count Exceeds v->inbound_degree"); */

    /* assert(v->iterations[iteration].messages_received_count < v->inbound_degree ||
           is_germinate && "Bug! current_iteration_incoming_count Exceeds v->inbound_degree"); */

    if (v->iterations[iteration].messages_received_count >= v->inbound_degree && !is_germinate) {
        std::cout << "BUIG! v->id: " << v->id << ", messages_received_count[" << iteration
                  << "]: " << v->iterations[iteration].messages_received_count
                  << ", v->inbound_degree: " << v->inbound_degree
                  << ", is_germinate: " << is_germinate << "\n";

        std::cout << "\nstate of counters, v->page_rank_current_nested_iteration: "
                  << v->page_rank_current_nested_iteration
                  << ", page_rank_args.nested_iteration: " << iteration << std::endl;
        // Print values before setting to zero
        for (int i = 0; i < nested_iterations; i++) {

            std::cout << "v->id: " << v->id << ", messages_received_count[" << i
                      << "]: " << v->iterations[i].messages_received_count
                      << ", iteration_page_rank_score[" << i
                      << "]: " << v->iterations[i].iteration_page_rank_score
                      << ", v->inbound_degree: " << v->inbound_degree << "\n";
        }
        std::cout << std::endl;

        exit(0);
    }

    if (is_germinate || is_first_message_of_the_epoch) {

        // Store the score of this iteration, which maybe used for diffusion.
        if (is_first_message_of_the_epoch) {
            v->nested_epoch_completed = false;
            v->args_for_diffusion.score =
                v->page_rank_score / static_cast<double>(v->number_of_edges);
        } else {
            u_int32_t const previous_iteration = iteration - 1;
            v->args_for_diffusion.score =
                v->iterations[previous_iteration].iteration_page_rank_score /
                static_cast<double>(v->number_of_edges);
        }

        /* if (v->id == DEBUG_VERTEX) {
            std::cout << "is_first_message_of_the_epoch = " << is_first_message_of_the_epoch
                      << ", is_germinate: " << is_germinate
                      << ", v->page_rank_current_nested_iteration: "
                      << v->page_rank_current_nested_iteration
                      << ", page_rank_args.nested_iteration: " << page_rank_args.nested_iteration
                      << ", v->page_rank_score: " << v->page_rank_score
                      << ", my sending score: " << v->args_for_diffusion.score << "\n";
        } */

        v->args_for_diffusion.src_vertex_id = v->id;
        v->args_for_diffusion.nested_iteration = iteration;

        v->iterations[iteration].is_current_iteration_diffusion_ready = true;

        // v->is_current_iteration_diffusion_ready[iteration] = true;
        // v->start_next_iteration = false;
    }
    /* if (is_first_message_of_the_epoch) {
        v->nested_epoch_completed = false;
    } */

    // If the action comes from the host and is germinate action then don't update scores and
    // just treat it as a purely diffusive action.
    if (action_type_in == actionType::application_action) {

        /* if(v->id == 0 && iteration == nested_iterations-1){
            std::cout << page_rank_args.src_vertex_id << "\n";
        } */

        // Update partial new score with the new incoming score.
        v->iterations[iteration].iteration_page_rank_score += page_rank_args.score;
        v->iterations[iteration].messages_received_count++;

        /* if (v->id == 255) {
            std::cout << "v->id: " << v->id << ", current_iteration_incoming_count["
                      << page_rank_args.nested_iteration << "]: "
                      << v->current_iteration_incoming_count[page_rank_args.nested_iteration]
                      << ", partial score[" << page_rank_args.nested_iteration
                      << "]: " << v->current_iteration_rank_score[page_rank_args.nested_iteration]
                      << ", v->inbound_degree: " << v->inbound_degree
                      << ", from: " << page_rank_args.src_vertex_id << "\n";
        } */
    }

    // If this is the last message for this iteration then update the score. And set the stage for
    // the germination of the next iteration.
    if ((v->iterations[iteration].messages_received_count == v->inbound_degree) && !is_germinate) {

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

        // Update the count for how many total iterations have been received and scores set for this
        // iterative epoch.
        v->iterations_received_this_epoch++;
        // Increament the global iteration count.
        v->page_rank_current_iteration++;

        if (v->id == DEBUG_VERTEX) {
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
        }

        // Go to the next nested iteration.
        v->page_rank_current_nested_iteration++;

        v->start_next_iteration = true;
    }
    return 0;
}

inline auto
page_rank_nested_fixed_iterations_diffuse_func(ComputeCell& cc,
                                               const Address& addr,
                                               actionType /* action_type_in */,
                                               const ActionArgumentType& args) -> int
{
    auto* v = static_cast<PageRankNestedFixedIterationsSimpleVertex<Address>*>(cc.get_object(addr));

    PageRankNestedFixedIterationsArguments const page_rank_args =
        cca_get_action_argument<PageRankNestedFixedIterationsArguments>(args);

    u_int32_t const iteration = page_rank_args.nested_iteration;
    // v->args_for_diffusion.nested_iteration; //;

    if (v->id == 126 && iteration == 5) {
        std::cout << "DIFFUSION: v->id: " << v->id << ", iteration: " << iteration
                  << "\n, has_diff_bool: "
                  << v->iterations[iteration].has_current_iteration_diffused
                  << "\n, v->page_rank_current_nested_iteration: "
                  << v->page_rank_current_nested_iteration
                  << ", nested_iterations: " << nested_iterations
                  << "\n, v->number_of_edges: " << v->number_of_edges
                  << ", v->nested_epoch_completed: " << v->nested_epoch_completed
                  << ", v->start_next_iteration: " << v->start_next_iteration
                  << ", v->args_for_diffusion.score: " << v->args_for_diffusion.score << std::endl;

        std::cout << "v->id: " << v->id << ", messages_received_count[" << iteration
                  << "]: " << v->iterations[iteration].messages_received_count
                  << ", iteration_page_rank_score[" << iteration
                  << "]: " << v->iterations[iteration].iteration_page_rank_score
                  << ", v->iterations_received_this_epoch: " << v->iterations_received_this_epoch
                  << ", v->inbound_degree: " << v->inbound_degree << "\n";
    }

    // If the diffusion has not occured for the current iteration then diffuse.
    if (!v->iterations[iteration].has_current_iteration_diffused &&
        v->iterations[iteration].is_current_iteration_diffusion_ready &&
        !v->nested_epoch_completed) {

        if (v->id == 126 && iteration == 5) {
            std::cout << "v->id: " << v->id << "Is in iteration 5 diffusion\n";
        }

        if (v->id == 126 && iteration == 5) {
            std::cout << "DIFFUSION: v->id: " << v->id << ", iteration: " << iteration
                      << "\n, has_diff_bool: "
                      << v->iterations[iteration].has_current_iteration_diffused
                      << "\n, v->page_rank_current_nested_iteration: "
                      << v->page_rank_current_nested_iteration
                      << ", nested_iterations: " << nested_iterations
                      << "\n, v->number_of_edges: " << v->number_of_edges
                      << ", v->nested_epoch_completed: " << v->nested_epoch_completed
                      << ", v->start_next_iteration: " << v->start_next_iteration
                      << ", v->args_for_diffusion.score: " << v->args_for_diffusion.score
                      << std::endl;
        }

        for (int i = 0; i < v->number_of_edges; i++) {
            // v->args_for_diffusion.nested_iteration = iteration;
            ActionArgumentType const args_x =
                cca_create_action_argument<PageRankNestedFixedIterationsArguments>(
                    v->args_for_diffusion);

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

        v->iterations[iteration].has_current_iteration_diffused = true;
    }

    // If this is germinate then prepare and germinate the next iteration.
    u_int32_t next_iteration = iteration + 1;
    if ((next_iteration < nested_iterations) && !v->nested_epoch_completed &&
        v->start_next_iteration) {

        v->start_next_iteration = false;

        if (next_iteration == 5) {
            if (v->id == DEBUG_VERTEX) {
                std::cout << "GERMINATE: v->id: " << v->id
                          << "\n, v->page_rank_current_nested_iteration: "
                          << v->page_rank_current_nested_iteration
                          << "\n, next_iteration: " << next_iteration
                          << ", nested_iterations: " << nested_iterations
                          << ", v->iterations_received_this_epoch: "
                          << v->iterations_received_this_epoch
                          << "\n, v->nested_epoch_completed: " << v->nested_epoch_completed
                          << ", v->start_next_iteration: " << v->start_next_iteration << std::endl;
            }
            // exit(0);
        }

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
    if (v->iterations_received_this_epoch == nested_iterations && v->iterations[nested_iterations-1].has_current_iteration_diffused) {
   

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
    parser.set_required<u_int32_t>(
        "tv", "testvertex", "test vertex to print its page_rank_nested_fixed_iterations score");
    parser.set_required<u_int32_t>("root",
                                   "page_rank_nested_fixed_iterations_root",
                                   "Root vertex where to germinate action for Page Rank (Page Rank "
                                   "Fixed Iterations). Makes no difference to the results.");
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

    parser.set_optional<u_int32_t>("route", "routing_policy", 0, "Routing algorithm to use.");
}

#endif // CCA_Page_Rank_Fixed_Iterations_HPP

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

// For memcpy()
#include <cstring>

inline constexpr double damping_factor = 0.85;
inline constexpr u_int32_t nested_iterations = 2;

template<typename Address_T>
struct PageRankNestedFixedIterationsSimpleVertex : SimpleVertex<Address_T>
{
    // This is the global iteration
    u_int32_t page_rank_current_iteration{};

    // When a diffusion occurs for a single iteration this is set to true so as to not diffuse the
    // same results everytime the vertex gets activated for a given iteration.
    bool has_current_iteration_diffused{};

    // The page rank score for this vertex.
    double page_rank_current_rank_score{};

    // This is the nested iteration count
    u_int32_t page_rank_current_nested_iteration{};

    // The page rank score for this vertex. This acts as a temporary to compute the new score.
    double current_iteration_rank_score[nested_iterations]{};

    // Count to see if this vertex has recieved all messages (scores) so as to then compute the
    // final score and move to the next iteration.
    // current_iteration_incoming_count == inbound_degree
    u_int32_t current_iteration_incoming_count[nested_iterations]{};

    PageRankNestedFixedIterationsSimpleVertex(u_int32_t id_in,
                                              u_int32_t total_number_of_vertices_in)
        : page_rank_current_rank_score(1.0 / total_number_of_vertices_in)
    {
        this->id = id_in;
        this->number_of_edges = 0;
        this->total_number_of_vertices = total_number_of_vertices_in;
    }

    // Custom initialize the page rank score other than the default of (1.0 / N).
    void initialize_page_rank_score(double initial_page_rank_score)
    {
        this->page_rank_current_rank_score = initial_page_rank_score;
    }

    PageRankNestedFixedIterationsSimpleVertex() = default;
    ~PageRankNestedFixedIterationsSimpleVertex() = default;
};

// CCAFunctionEvent ids for the Page Rank Fixed Iterations action: predicate, work, and diffuse.
// In the main register the functions with the CCASimulator chip and get their ids.
extern CCAFunctionEvent page_rank_nested_fixed_iterations_predicate;
extern CCAFunctionEvent page_rank_nested_fixed_iterations_work;
extern CCAFunctionEvent page_rank_nested_fixed_iterations_diffuse;

// This is what the action carries as payload.
struct PageRankNestedFixedIterationsArguments
{
    double score;
    u_int32_t nested_iteration;
    u_int32_t src_vertex_id;
};

inline auto
page_rank_nested_fixed_iterations_predicate_func(ComputeCell& cc,
                                                 const Address& addr,
                                                 actionType /* action_type_in */,
                                                 const ActionArgumentType& args) -> int
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

    // If the action comes from the host and is germinate action then don't update scores and just
    // treat it as a purely diffusive action.
    if (action_type_in == actionType::application_action) {
        auto* v =
            static_cast<PageRankNestedFixedIterationsSimpleVertex<Address>*>(cc.get_object(addr));

        PageRankNestedFixedIterationsArguments const page_rank_args =
            cca_get_action_argument<PageRankNestedFixedIterationsArguments>(args);

        // Update partial new score with the new incoming score.
        v->current_iteration_rank_score[page_rank_args.nested_iteration] += page_rank_args.score;
        v->current_iteration_incoming_count[page_rank_args.nested_iteration]++;
    }
    return 0;
}

inline auto
page_rank_nested_fixed_iterations_diffuse_func(ComputeCell& cc,
                                               const Address& addr,
                                               actionType /* action_type_in */,
                                               const ActionArgumentType& /*args*/) -> int
{
    auto* v = static_cast<PageRankNestedFixedIterationsSimpleVertex<Address>*>(cc.get_object(addr));

    // If the diffusion has not occured for the current iteration then diffuse.
    if (!v->has_current_iteration_diffused) {
        PageRankNestedFixedIterationsArguments my_score_to_send;

        my_score_to_send.score =
            v->page_rank_current_rank_score / static_cast<double>(v->number_of_edges);
        my_score_to_send.src_vertex_id = v->id;
        my_score_to_send.nested_iteration = v->page_rank_current_nested_iteration;

        for (int i = 0; i < v->number_of_edges; i++) {

            ActionArgumentType const args_x =
                cca_create_action_argument<PageRankNestedFixedIterationsArguments>(
                    my_score_to_send);

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

        v->has_current_iteration_diffused = true;
    }

    if (v->current_iteration_incoming_count[v->page_rank_current_nested_iteration] ==
        v->inbound_degree) {
        // Update the page rank score.
        v->page_rank_current_rank_score =
            ((1.0 - damping_factor) / static_cast<double>(v->total_number_of_vertices)) +
            (damping_factor *
             v->current_iteration_rank_score[v->page_rank_current_nested_iteration]);

        // Reset.
        v->current_iteration_rank_score[v->page_rank_current_nested_iteration] = 0.0;
        v->current_iteration_incoming_count[v->page_rank_current_nested_iteration] = 0;
        v->has_current_iteration_diffused = false;

        // Increament the global iteration count.
        v->page_rank_current_iteration++;

        // Go to the next nested iteration.
        v->page_rank_current_nested_iteration++;
        if (v->page_rank_current_nested_iteration == nested_iterations) {
            // Reset and done with all the nested iterations.
            v->page_rank_current_nested_iteration = 0;
        } else {

            // Reset
            v->has_current_iteration_diffused = false;

            PageRankNestedFixedIterationsArguments dummy;
            ActionArgumentType const args_x =
                cca_create_action_argument<PageRankNestedFixedIterationsArguments>(dummy);

            // Germinate action onto ownself.
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

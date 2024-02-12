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
#include "Enums.hpp"

// Datastructures
#include "Graph.hpp"
#include "RecursiveParallelVertex.hpp"

#include "cmdparser.hpp"

// For memcpy()
#include <cstring>

inline constexpr double damping_factor = 0.85;

// This is what the action carries as payload.
struct PageRankFixedIterationsArguments
{
    double score;
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
    double page_rank_current_rank_score{};

    // The page rank score for this vertex. This acts as a temporary to compute the new score.
    double current_iteration_rank_score{};

    // Count to see if this vertex has recieved all messages (scores) so as to then compute the
    // final score and move to the next iteration.
    // current_iteration_incoming_count == inbound_degree
    u_int32_t current_iteration_incoming_count{};

    PageRankFixedIterationsVertex(u_int32_t id_in, u_int32_t total_number_of_vertices_in)
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

    PageRankFixedIterationsVertex() = default;
    ~PageRankFixedIterationsVertex() = default;
};

// CCAFunctionEvent ids for the Page Rank Fixed Iterations action: predicate, work, and diffuse.
// In the main register the functions with the CCASimulator chip and get their ids.
extern CCAFunctionEvent page_rank_fixed_iterations_predicate;
extern CCAFunctionEvent page_rank_fixed_iterations_work;
extern CCAFunctionEvent page_rank_fixed_iterations_diffuse_predicate;
extern CCAFunctionEvent page_rank_fixed_iterations_diffuse;

inline auto
page_rank_fixed_iterations_predicate_func(ComputeCell& cc,
                                          const Address& addr,
                                          actionType /* action_type_in */,
                                          const ActionArgumentType& args) -> CCAFunctionEvent
{
    // Set to always true. Since the idea is to accumulate the scores per iteration from all inbound
    // vertices.
    return static_cast<CCAFunctionEvent>(1); // TODO: provide cc.true_event
}

inline auto
page_rank_fixed_iterations_work_func(ComputeCell& cc,
                                     const Address& addr,
                                     actionType action_type_in,
                                     const ActionArgumentType& args) -> CCAFunctionEvent
{

    // First check whether this is a ghost vertex. If it is then don't perform any work.
    // parent word is used in the sense that `RecursiveParallelVertex` is the parent class.
    auto* parent_recursive_parralel_vertex =
        static_cast<RecursiveParallelVertex<Address>*>(cc.get_object(addr));

    if (parent_recursive_parralel_vertex->is_ghost_vertex) {
        return static_cast<CCAFunctionEvent>(1); // TODO: provide cc.true_event
    }

    // If the action comes from the host and is germinate action then don't update scores and just
    // treat it as a purely diffusive action.
    if (action_type_in == actionType::application_action) {

        auto* v = static_cast<PageRankFixedIterationsVertex<RecursiveParallelVertex<Address>>*>(
            cc.get_object(addr));

        PageRankFixedIterationsArguments const page_rank_args =
            cca_get_action_argument<PageRankFixedIterationsArguments>(args);

        // Update partial new score with the new incoming score.
        v->current_iteration_rank_score += page_rank_args.score;
        v->current_iteration_incoming_count++;
    }
    return 0;
}

inline auto
page_rank_fixed_iterations_diffuse_predicate_func(ComputeCell& cc,
                                                  const Address& addr,
                                                  actionType /* action_type_in */,
                                                  const ActionArgumentType& /*args*/)
    -> CCAFunctionEvent
{
    return 1;
}

inline auto
page_rank_fixed_iterations_diffuse_func(ComputeCell& cc,
                                        const Address& addr,
                                        actionType /* action_type_in */,
                                        const ActionArgumentType& /*args*/) -> CCAFunctionEvent
{

    // Get the hold of the parent ghost vertex. If it is ghost then simply perform diffusion.
    auto* parent_recursive_parralel_vertex =
        static_cast<RecursiveParallelVertex<Address>*>(cc.get_object(addr));
    bool this_is_ghost_vertex = parent_recursive_parralel_vertex->is_ghost_vertex;

    assert(this_is_ghost_vertex == false && "There is ghost");

    auto* v = static_cast<PageRankFixedIterationsVertex<RecursiveParallelVertex<Address>>*>(
        cc.get_object(addr));

    // If the diffusion has not occured for the current iteration then diffuse.
    if (!v->has_current_iteration_diffused) {
        PageRankFixedIterationsArguments my_score_to_send;
        my_score_to_send.score =
            v->page_rank_current_rank_score / static_cast<double>(v->outbound_degree);

        my_score_to_send.src_vertex_id = v->id;

        for (int i = 0; i < v->number_of_edges; i++) {

            ActionArgumentType const args_x =
                cca_create_action_argument<PageRankFixedIterationsArguments>(my_score_to_send);

            // Diffuse.
            cc.diffuse(Action(v->edges[i].edge,
                              addr,
                              actionType::application_action,
                              true,
                              args_x,
                              page_rank_fixed_iterations_predicate,
                              page_rank_fixed_iterations_work,
                              page_rank_fixed_iterations_diffuse_predicate,
                              page_rank_fixed_iterations_diffuse));
        }
        if (v->id == 1)
            std::cout << "vertex: " << v->id << ", inbound: " << v->inbound_degree
                      << ", page_rank_current_iteration: " << v->page_rank_current_iteration
                      << ", has_current_iteration_diffused: " << v->has_current_iteration_diffused
                      << std::endl;
        v->has_current_iteration_diffused = true;
    }

    if (v->current_iteration_incoming_count == v->inbound_degree) {

        // Update the page rank score.
        v->page_rank_current_rank_score =
            ((1.0 - damping_factor) / static_cast<double>(v->total_number_of_vertices)) +
            (damping_factor * v->current_iteration_rank_score);

        // Reset.
        v->current_iteration_rank_score = 0.0;
        v->current_iteration_incoming_count = 0;
        v->has_current_iteration_diffused = false;

        // Increament the global iteration count.
        v->page_rank_current_iteration++;
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

    parser.set_optional<u_int32_t>("route", "routing_policy", 0, "Routing algorithm to use.");
}

#endif // CCA_Page_Rank_Fixed_Iterations_HPP

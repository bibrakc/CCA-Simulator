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

inline constexpr double damping_factor = 0.85;

template<typename Address_T>
struct PageRankFixedIterationsSimpleVertex : SimpleVertex<Address_T>
{
    u_int32_t page_rank_current_iteration{};

    // When a diffusion occurs for a single iteration this is set to true so as to not diffuse the
    // same results everytime the vertex gets activated for a given iteration.
    bool has_current_iteration_diffused{};

    // Used in the calculation with the damping factor.
    u_int32_t total_number_of_vertices;

    // Total inbound edges to this vertex.
    u_int32_t inbound_degree;

    // The page rank score for this vertex.
    double page_rank_current_rank_score{};

    // The page rank score for this vertex. This acts as a temporary to compute the new score.
    double current_iteration_rank_score{};

    // Count to see if this vertex has recieved all messages (scores) so as to then compute the
    // final score and move to the next iteration.
    // current_iteration_incoming_count == inbound_degree
    u_int32_t current_iteration_incoming_count{};

    PageRankFixedIterationsSimpleVertex(u_int32_t id_in, u_int32_t total_number_of_vertices_in)
        : total_number_of_vertices(total_number_of_vertices_in)
        , page_rank_current_rank_score(1.0 / total_number_of_vertices_in)
    {
        this->id = id_in;
        this->number_of_edges = 0;
    }

    // Custom initialize the page rank score other than the default of (1.0 / N).
    void initialize_page_rank_score(double initial_page_rank_score)
    {
        this->page_rank_current_rank_score = initial_page_rank_score;
    }

    PageRankFixedIterationsSimpleVertex() = default;
    ~PageRankFixedIterationsSimpleVertex() = default;
};

// CCAFunctionEvent ids for the Page Rank Fixed Iterations action: predicate, work, and diffuse.
// In the main register the functions with the CCASimulator chip and get their ids.
extern CCAFunctionEvent page_rank_fixed_iterations_predicate;
extern CCAFunctionEvent page_rank_fixed_iterations_work;
extern CCAFunctionEvent page_rank_fixed_iterations_diffuse;

// This is what the action carries as payload.
struct PageRankFixedIterationsArguments
{
    double score;
};

// Action for the Page Rank Fixed Iterations program.
class PageRankFixedIterationsAction : public Action
{
  public:
    PageRankFixedIterationsAction(const Address destination_vertex_addr_in,
                                  const Address origin_vertex_addr_in,
                                  actionType type,
                                  const bool ready,
                                  const int nargs_in,
                                  const std::shared_ptr<int[]>& args_in,
                                  CCAFunctionEvent predicate_in,
                                  CCAFunctionEvent work_in,
                                  CCAFunctionEvent diffuse_in)
    {
        this->obj_addr = destination_vertex_addr_in;
        this->origin_addr = origin_vertex_addr_in;

        this->action_type = type;
        this->is_ready = ready;

        this->nargs = nargs_in;
        this->args = args_in;

        this->predicate = predicate_in;
        this->work = work_in;
        this->diffuse = diffuse_in;
    }

    ~PageRankFixedIterationsAction() override = default;
};

inline auto
page_rank_fixed_iterations_predicate_func(ComputeCell& cc,
                                          const Address& addr,
                                          int /*nargs*/,
                                          const std::shared_ptr<int[]>& args) -> int
{
    // Set to always true. Since the idea is to accumulate the scores per iteration from all inbound
    // vertices.
    return 1;
}

inline auto
page_rank_fixed_iterations_work_func(ComputeCell& cc,
                                     const Address& addr,
                                     int /*nargs*/,
                                     const std::shared_ptr<int[]>& args) -> int
{
    auto* v = static_cast<PageRankFixedIterationsSimpleVertex<Address>*>(cc.get_object(addr));
    // TODO: memcpy size of double. Change the func to be generic.
    double const incoming_score = args[0];

    // Update partial new score with the new incoming score.
    v->current_iteration_rank_score += incoming_score;
    v->current_iteration_incoming_count++;

    if (v->current_iteration_incoming_count == v->inbound_degree) {
        // Update the page rank score.
        v->page_rank_current_rank_score = ((1 - damping_factor) / v->total_number_of_vertices) +
                                          (damping_factor * v->current_iteration_rank_score);
    }
    return 0;
}

inline auto
page_rank_fixed_iterations_diffuse_func(ComputeCell& cc,
                                        const Address& addr,
                                        int /*nargs*/,
                                        const std::shared_ptr<int[]>& /*args*/) -> int
{
    auto* v = static_cast<PageRankFixedIterationsSimpleVertex<Address>*>(cc.get_object(addr));

    // If the diffusion has not occured for the current iteration then diffuse.
    if (!v->has_current_iteration_diffused) {
        double my_score_to_send =
            v->page_rank_current_rank_score / static_cast<double>(v->number_of_edges);
        for (int i = 0; i < v->number_of_edges; i++) {

            // std::shared_ptr<int[]> args_x = std::make_shared<int[]>(2);
            std::shared_ptr<int[]> const args_x(new int[2], std::default_delete<int[]>());

            args_x[0] = static_cast<int>(my_score_to_send);
            args_x[1] = static_cast<int>(v->id);

            cc.diffuse(PageRankFixedIterationsAction(v->edges[i].edge,
                                                     addr,
                                                     actionType::application_action,
                                                     true,
                                                     2,
                                                     args_x,
                                                     page_rank_fixed_iterations_predicate,
                                                     page_rank_fixed_iterations_work,
                                                     page_rank_fixed_iterations_diffuse));
        }

        v->has_current_iteration_diffused = true;
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
        "tv", "testvertex", "test vertex to print its page_rank_fixed_iterations distance");
    parser.set_required<u_int32_t>(
        "root",
        "page_rank_fixed_iterationsroot",
        "Root vertex for Single Source Shortest Path (Page Rank Fixed Iterations)");
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

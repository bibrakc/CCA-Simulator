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

#ifndef CCA_SSSP_HPP
#define CCA_SSSP_HPP

#include "CCASimulator.hpp"
#include "Enums.hpp"
#include "RecursiveParallelVertex.hpp"

#include "cmdparser.hpp"

// For memcpy()
#include <cstring>

template<typename Vertex_T>
struct SSSPVertex : Vertex_T
{
    inline static constexpr u_int32_t max_distance = 999999;

    // The distance from root.
    u_int32_t sssp_distance;

    // The vertex from which my current ditance comes from.
    Address parent;
    u_int32_t depth{};

    SSSPVertex(u_int32_t id_in, u_int32_t total_number_of_vertices_in)
        : sssp_distance(SSSPVertex::max_distance)
        , parent(0, 0, adressType::invalid_address)
    {
        this->id = id_in;
        this->number_of_edges = 0;
        this->total_number_of_vertices = total_number_of_vertices_in;
    }

    SSSPVertex() = default;
    ~SSSPVertex() = default;
};

// CCAFunctionEvent ids for the SSSP action: predicate, work, and diffuse.
// In the main register the functions with the CCASimulator chip and get their ids.
extern CCAFunctionEvent sssp_iterative_predicate;
extern CCAFunctionEvent sssp_iterative_work;
extern CCAFunctionEvent sssp_iterative_diffuse;

// This is what the action carries as payload.
struct SSSPIterativeArguments
{
    u_int32_t distance;
    u_int32_t src_vertex_id;
    Address src_vertex_addr;
    u_int32_t depth_max;
    u_int32_t depth_current;
};

inline auto
sssp_iterative_predicate_func(ComputeCell& cc,
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

    auto* v = static_cast<SSSPVertex<RecursiveParallelVertex<Address>>*>(cc.get_object(addr));

    SSSPIterativeArguments const sssp_args = cca_get_action_argument<SSSPIterativeArguments>(args);
    u_int32_t const incoming_distance = sssp_args.distance;

    // This is the first time that this vertex has received an action. Predicate is true and update
    // the distance for the first time.
    if (v->parent.type == adressType::invalid_address) {
        return 1;
    }

    const bool same_parent = v->parent == sssp_args.src_vertex_addr;
    const bool same_distance = v->sssp_distance == sssp_args.distance;
    const bool less_than_max_depth = sssp_args.depth_current < sssp_args.depth_max;
    if (same_parent && same_distance && less_than_max_depth) {
        return 1;
    }

    // TODO: check this < or <= for depth??
    const bool greater_than_incoming_distance = v->sssp_distance > incoming_distance;
    if (greater_than_incoming_distance && less_than_max_depth) {
        return 1;
    }
    return 0;
}

inline auto
sssp_iterative_work_func(ComputeCell& cc,
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

    auto* v = static_cast<SSSPVertex<RecursiveParallelVertex<Address>>*>(cc.get_object(addr));

    SSSPIterativeArguments const sssp_args = cca_get_action_argument<SSSPIterativeArguments>(args);
    u_int32_t const incoming_distance = sssp_args.distance;

    // Update distance with the new distance.
    v->sssp_distance = incoming_distance;

    // Update the parent.
    v->parent = sssp_args.src_vertex_addr;

    // To be used when diffusing.
    v->depth = sssp_args.depth_current;
    return 0;
}

inline auto
sssp_iterative_diffuse_func(ComputeCell& cc,
                            const Address& addr,
                            actionType /* action_type_in */,
                            const ActionArgumentType& args) -> int
{

    // Get the hold of the parent ghost vertex. If it is ghost then simply perform diffusion.
    auto* parent_recursive_parralel_vertex =
        static_cast<RecursiveParallelVertex<Address>*>(cc.get_object(addr));
    bool this_is_ghost_vertex = parent_recursive_parralel_vertex->is_ghost_vertex;

    auto* v = static_cast<SSSPVertex<RecursiveParallelVertex<Address>>*>(cc.get_object(addr));

    SSSPIterativeArguments const sssp_args = cca_get_action_argument<SSSPIterativeArguments>(args);

    u_int32_t current_distance = SSSPVertex<RecursiveParallelVertex<Address>>::max_distance;
    u_int32_t const current_depth = sssp_args.depth_current;
    u_int32_t const depth_max = sssp_args.depth_max;

    if (this_is_ghost_vertex) {
        current_distance = sssp_args.distance;
    } else {
        current_distance = v->sssp_distance;
    }

    SSSPIterativeArguments distance_to_send;

    // To be potentially sent to ghost vertices.
    distance_to_send.distance = current_distance;
    distance_to_send.src_vertex_id = v->id;
    distance_to_send.depth_max = depth_max;
    distance_to_send.depth_current = v->depth + 1;
    // My address.
    distance_to_send.src_vertex_addr = addr;

    ActionArgumentType const args_for_ghost_vertices =
        cca_create_action_argument<SSSPIterativeArguments>(distance_to_send);

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
                              sssp_iterative_predicate,
                              sssp_iterative_work,
                              sssp_iterative_diffuse));
        }
    }

    for (int i = 0; i < v->number_of_edges; i++) {

        distance_to_send.distance = current_distance + v->edges[i].weight;
        ActionArgumentType const args_x =
            cca_create_action_argument<SSSPIterativeArguments>(distance_to_send);

        cc.diffuse(Action(v->edges[i].edge,
                          addr,
                          actionType::application_action,
                          true,
                          args_x,
                          sssp_iterative_predicate,
                          sssp_iterative_work,
                          sssp_iterative_diffuse));
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
        "root", "sssproot", "Root vertex for Single Source Shortest Path (SSSP)");
    parser.set_optional<bool>("verify",
                              "verification",
                              0,
                              "Enable verification of the calculated paths with the paths provided "
                              "in the accompanying .sssp file");
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

#endif // CCA_SSSP_HPP

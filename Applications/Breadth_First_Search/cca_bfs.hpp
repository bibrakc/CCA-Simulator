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
#include "SimpleVertex.hpp"

#include "cmdparser.hpp"

// For memcpy()
#include <cstring>

inline constexpr u_int32_t max_level = 999999;

template<typename Address_T>
struct BFSSimpleVertex : SimpleVertex<Address_T>
{
    u_int32_t bfs_level;

    BFSSimpleVertex(u_int32_t id_in, u_int32_t total_number_of_vertices_in)
        : bfs_level(max_level)
    {
        this->id = id_in;
        this->number_of_edges = 0;
        this->total_number_of_vertices = total_number_of_vertices_in;
    }

    BFSSimpleVertex() {}
    ~BFSSimpleVertex() {}
};

// CCAFunctionEvent ids for the BFS action: predicate, work, and diffuse.
// In the main register the functions with the CCASimulator chip and get their ids.
extern CCAFunctionEvent bfs_predicate;
extern CCAFunctionEvent bfs_work;
extern CCAFunctionEvent bfs_diffuse;

// This is what the action carries as payload.
struct BFSArguments
{
    u_int32_t level;
    u_int32_t src_vertex_id;
};

inline auto
bfs_predicate_func(ComputeCell& cc,
                   const Address& addr,
                   actionType /* action_type_in */,
                   const ActionArgumentType& args) -> int
{
    auto* v = static_cast<BFSSimpleVertex<Address>*>(cc.get_object(addr));
    BFSArguments const bfs_args = cca_get_action_argument<BFSArguments>(args);

    u_int32_t const incoming_level = bfs_args.level;

    if (v->bfs_level > incoming_level) {
        return 1;
    }
    return 0;
}

inline auto
bfs_work_func(ComputeCell& cc,
              const Address& addr,
              actionType /* action_type_in */,
              const ActionArgumentType& args) -> int
{
    auto* v = static_cast<BFSSimpleVertex<Address>*>(cc.get_object(addr));
    BFSArguments const bfs_args = cca_get_action_argument<BFSArguments>(args);

    u_int32_t const incoming_level = bfs_args.level;

    // Update level with the new level
    v->bfs_level = incoming_level;
    return 0;
}

inline auto
bfs_diffuse_func(ComputeCell& cc,
                 const Address& addr,
                 actionType /* action_type_in */,
                 const ActionArgumentType& args) -> int
{
    auto* v = static_cast<BFSSimpleVertex<Address>*>(cc.get_object(addr));

    BFSArguments level_to_send;
    level_to_send.src_vertex_id = v->id;

    for (int i = 0; i < v->number_of_edges; i++) {

        level_to_send.level = v->bfs_level + 1;
        ActionArgumentType const args_x = cca_create_action_argument<BFSArguments>(level_to_send);

        cc.diffuse(Action(v->edges[i].edge,
                          addr,
                          actionType::application_action,
                          true,
                          args_x,
                          bfs_predicate,
                          bfs_work,
                          bfs_diffuse));
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
    parser.set_required<u_int32_t>("tv", "testvertex", "test vertex to print its bfs level");
    parser.set_required<u_int32_t>("root", "bfsroot", "Root vertex for Breadth First Search (BFS)");
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

#endif // CCA_BFS_HPP

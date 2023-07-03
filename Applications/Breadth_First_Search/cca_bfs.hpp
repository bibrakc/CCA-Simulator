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
#include "SimpleVertex.hpp"

#include "cmdparser.hpp"

inline constexpr u_int32_t max_level = 999999;

template<typename Address_T>
struct BFSSimpleVertex : SimpleVertex<Address_T>
{
    u_int32_t bfs_level;

    BFSSimpleVertex(u_int32_t id_in)
        : bfs_level(max_level)
    {
        this->id = id_in;
        this->number_of_edges = 0;
    }

    BFSSimpleVertex() {}
    ~BFSSimpleVertex() {}
};

// CCAFunctionEvent ids for the BFS action: predicate, work, and diffuse.
// In the main register the functions with the CCASimulator chip and get their ids.
extern CCAFunctionEvent bfs_predicate;
extern CCAFunctionEvent bfs_work;
extern CCAFunctionEvent bfs_diffuse;

// Action for the BFS program.
class BFSAction : public Action
{
  public:
    BFSAction(const Address destination_vertex_addr_in,
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

    ~BFSAction() override {}
};

auto
bfs_predicate_func(ComputeCell& cc,
                   const Address& addr,
                   int nargs,
                   const std::shared_ptr<int[]>& args) -> int
{
    BFSSimpleVertex<Address>* v = static_cast<BFSSimpleVertex<Address>*>(cc.get_object(addr));
    int incoming_level = args[0];
    int origin_vertex = args[1];

    if (v->bfs_level > incoming_level) {
        return 1;
    }
    return 0;
}

auto
bfs_work_func(ComputeCell& cc, const Address& addr, int nargs, const std::shared_ptr<int[]>& args) -> int
{
    BFSSimpleVertex<Address>* v = static_cast<BFSSimpleVertex<Address>*>(cc.get_object(addr));
    int incoming_level = args[0];

    // Update level with the new level
    v->bfs_level = incoming_level;
    return 0;
}

auto
bfs_diffuse_func(ComputeCell& cc,
                 const Address& addr,
                 int nargs,
                 const std::shared_ptr<int[]>& args) -> int
{
    BFSSimpleVertex<Address>* v = static_cast<BFSSimpleVertex<Address>*>(cc.get_object(addr));
    for (int i = 0; i < v->number_of_edges; i++) {

        // std::shared_ptr<int[]> args_x = std::make_shared<int[]>(2);
        std::shared_ptr<int[]> args_x(new int[2], std::default_delete<int[]>());
        args_x[0] = static_cast<int>(v->bfs_level + 1);
        args_x[1] = static_cast<int>(v->id);

        cc.diffuse(BFSAction(v->edges[i].edge,
                             addr,
                             actionType::application_action,
                             true,
                             2,
                             args_x,
                             bfs_predicate,
                             bfs_work,
                             bfs_diffuse));
    }

    return 0;
}

void
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

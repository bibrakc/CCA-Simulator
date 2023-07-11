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

#ifndef RECURSIVE_PARALLEL_Vertex_HPP
#define RECURSIVE_PARALLEL_Vertex_HPP

#include "SimpleVertex.hpp"

template<typename Address_T>
struct RecursiveParallelVertex : SimpleVertex<Address_T>
{

    // Checking to see whether the user has mistakenly used this for host side allocation. If they
    // give `Edge.edge` type other than `Address` then it means they are intenting to allocate on
    // the host. This vertex object is not supposed to be allocated on the host. For that purpose
    // use the `SimpleVertex`.
    static_assert(std::is_same_v<Address_T, Address>);

    // Insert an edge with weight
    auto insert_edge(Address_T dst_vertex_addr, u_int32_t edge_weight) -> bool
    {
        std::cout << "RecursiveParallelVertex insert_edge\n";

        if (this->number_of_edges >= edges_max) {
            std::cerr << "this->number_of_edges: " << this->number_of_edges << "\n";
            return false;
        }

        this->edges[this->number_of_edges].edge = dst_vertex_addr;
        this->edges[this->number_of_edges].weight = edge_weight;
        this->number_of_edges++;

        return true;
    }

    RecursiveParallelVertex() = default;
    ~RecursiveParallelVertex() = default;
};

// Print the SimpleVertex vertex
inline void
print_RecursiveParallelVertex(const RecursiveParallelVertex<Address>* vertex,
                              const Address& vertex_addr)
{
    std::cout << "Vertex ID: " << vertex->id << ", Addr: "
              << vertex_addr
              //<< " sssp_distance: " << vertex->sssp_distance
              << " deficit: " << vertex->terminator.deficit << "\n";

    for (u_int32_t i = 0; i < vertex->number_of_edges; i++) {
        std::cout << "\t\t[" << vertex->edges[i].edge << ", {w: " << vertex->edges[i].weight
                  << "} ]";
    }
    std::cout << std::endl;
}

#endif // RECURSIVE_PARALLEL_Vertex_HPP

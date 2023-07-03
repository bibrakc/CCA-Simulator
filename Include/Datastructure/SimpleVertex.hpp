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

#ifndef SimpleVertex_HPP
#define SimpleVertex_HPP

#include "Address.hpp"
#include "Constants.hpp"
#include "Object.hpp"

template<typename Address_T>
struct Edge
{
    Address_T edge;
    u_int32_t weight;
};

inline constexpr u_int32_t edges_max = 60;

template<typename Address_T>
struct SimpleVertex : Object
{
    u_int32_t id{};
    Edge<Address_T> edges[edges_max]{};
    u_int32_t number_of_edges{};

    // Insert an edge with weight
    auto insert_edge(Address_T dst_vertex_addr, u_int32_t edge_weight) -> bool
    {
        if (this->number_of_edges >= edges_max)
            return false;

        this->edges[this->number_of_edges].edge = dst_vertex_addr;
        this->edges[this->number_of_edges].weight = edge_weight;
        this->number_of_edges++;

        return true;
    }

    SimpleVertex(u_int32_t id_in)
        : id(id_in)
        , number_of_edges(0)
    {
    }

    SimpleVertex() = default;
    ~SimpleVertex() = default;
};

// Print the SimpleVertex vertex
void
print_SimpleVertex(const SimpleVertex<Address>* vertex, const Address& vertex_addr)
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

#endif // SimpleVertex_HPP

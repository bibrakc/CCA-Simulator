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

using host_edge_type = u_int32_t;

/* template<typename Address_T>
struct Edge
{
    Address_T edge;
    u_int32_t weight;
}; */

// Simple edge that only contains information about the other vertex it is pointing/connected to.
template<typename Address_T, bool weighted>
struct EdgeBase
{
    Address_T edge;

    EdgeBase() = default;
    // For compatability with the EdgeBase that has weight.
    EdgeBase(Address_T edge_in, uint32_t weight_in) { this->edge = edge_in; }
};

// An edge with weight. Useful for app such as SSSP.
template<typename Address_T>
struct EdgeBase<Address_T, true>
{
    Address_T edge;
    uint32_t weight;
};

template<typename Address_T>
using Edge = EdgeBase<Address_T, weighted_edge>;

// Used when the vertex is allocated on the CCA device. There we just create an edge list of size
// `edgelist_size`.
template<typename Address_T, u_int32_t edgelist_size>
struct SimpleVertex : Object
{
    u_int32_t id{};
    u_int32_t local_edgelist_size{};

    // If the graph is located on the host then simply store the edges as a `std::vector` but if the
    // graph is stored on the CCA then store it as a smaller edges[] array.
    static bool constexpr is_vertex_allocated_on_cca_device = std::is_same_v<Address_T, Address>;
    using Edges_t = std::conditional_t<is_vertex_allocated_on_cca_device,
                                       Edge<Address>[edgelist_size],
                                       std::vector<Edge<Address_T>>>;
    Edges_t edges{};

    // Note: For the SimpleVertex both `number_of_edges` and `outbound_degree` mean the same.
    // Edges in the edge list.
    u_int32_t number_of_edges{};
    // Outbound degree. Number of vertices that this vertex points to.
    u_int32_t outbound_degree{};

    // Total inbound edges to this vertex. Used for Page Rank like applications.
    u_int32_t inbound_degree{};

    // Used in the calculation with the damping factor in page rank. Or can be used in other
    // algorithms. Right now putting this here in the parent class since this might be used for many
    // algorithms. TODO: Think of ways how this changes in dynamic graphs
    u_int32_t total_number_of_vertices;

    // Insert an edge with weight on the device.
    auto insert_edge(CCASimulator& /* cca_simulator */,
                     u_int32_t /* source_vertex_cc_id */,
                     Address_T dst_vertex_addr,
                     u_int32_t edge_weight) -> bool
    {
        // std::cout << "SimpleVertex insert_edge\n";
        if constexpr (this->is_vertex_allocated_on_cca_device) {

            if (this->number_of_edges >= local_edgelist_size) {
                std::cerr << "this->number_of_edges: " << this->number_of_edges << "\n";
                return false;
            }

            this->edges[this->number_of_edges].edge = dst_vertex_addr;
            if constexpr (weighted_edge) {
                this->edges[this->number_of_edges].weight = edge_weight;
            }
            this->number_of_edges++;
            this->outbound_degree++;
        } else {
            this->edges.emplace_back(dst_vertex_addr, edge_weight);

            this->number_of_edges++;
            this->outbound_degree++;
        }

        return true;
    }

    // Insert an edge with weight on the host.
    auto insert_edge(host_edge_type dst_vertex_addr, u_int32_t edge_weight) -> bool
    {
        // std::cout << "SimpleVertex insert_edge\n";
        static_assert(!this->is_vertex_allocated_on_cca_device);

        this->edges.emplace_back(dst_vertex_addr, edge_weight);
        this->number_of_edges++;
        this->outbound_degree++;

        return true;
    }

    auto init(CCASimulator& cca_simulator, u_int32_t source_cc_id) -> bool
    {
        // Do nothing. Here just to make it compatible with the RecurssiveParallelVertex since it
        // uses the `init` to initialize `ghost_vertex_allocator`.
        return true;
    }

    SimpleVertex()
        : local_edgelist_size(edgelist_size)
    {
        /* std::cout << "SimpleVertex " << this->id << ", called local_edgelist_size = " <<
        local_edgelist_size
        << std::endl; */
    }
    ~SimpleVertex() = default;
};

// Print the SimpleVertex vertex
/* inline void
print_SimpleVertex(const SimpleVertex<Address, edges_max>* vertex, const Address& vertex_addr)
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
} */

#endif // SimpleVertex_HPP

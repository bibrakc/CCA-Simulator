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

inline constexpr u_int32_t ghost_vertices_max = 1;

template<typename Address_T>
struct RecursiveParallelVertex : SimpleVertex<Address_T>
{

    // Checking to see whether the user has mistakenly used this for host side allocation. If they
    // give `Edge.edge` type other than `Address` then it means they are intenting to allocate on
    // the host. This vertex object is not supposed to be allocated on the host. For that purpose
    // use the `SimpleVertex`.
    static_assert(std::is_same_v<Address_T, Address>);

    // If this vertex is ghost vertex? Default is `false` meaning that it is the root/main vertex
    // not a ghost.
    bool is_ghost_vertex{};
    // Addresses of any ghost vertices that this vertex might have.
    std::optional<Address_T> ghost_vertices[ghost_vertices_max]{};
    // Balance adding into the ghost vertices by having this iterator that goes in round-robin.
    u_int32_t next_insertion_in_ghost_iterator{};

    // Recurssively add edge into the ghost vertex
    // Insert edge by `Address` type src and dst
    inline auto insert_edge_in_ghost_vertex(CCASimulator& cca_simulator,
                                            std::unique_ptr<MemoryAllocator>& allocator,
                                            Address src_vertex_addr,
                                            Address dst_vertex_addr,
                                            u_int32_t edge_weight) -> bool
    {

        auto* vertex = static_cast<VertexTypeOfAddress*>(cca_simulator.get_object(src_vertex_addr));
        bool success = vertex->insert_edge(cca_simulator, allocator, dst_vertex_addr, edge_weight);

        // Increament the `inbound_degree` of the destination vertex
        if (success) {
            auto* vertex =
                static_cast<VertexTypeOfAddress*>(cca_simulator.get_object(dst_vertex_addr));

            vertex->inbound_degree++;
        }
        // Check if edges are not full
        // TODO: Later implement the hierarical parallel vertex object
        return success;
    }

    // Insert an edge with weight
    auto insert_edge(CCASimulator& cca_simulator,
                     std::unique_ptr<MemoryAllocator>& allocator,
                     Address_T dst_vertex_addr,
                     u_int32_t edge_weight) -> bool
    {
        std::cout << "RecursiveParallelVertex insert_edge\n";

        if (this->number_of_edges == edges_max) {
            std::cerr << "this->number_of_edges: " << this->number_of_edges
                      << " max edge list limit reached. Using ghost vertex now.\n";

            if (!this->ghost_vertices[0].has_value()) {
                std::cout << "No ghost vertex. Need to allocate one!\n";

                // Knowingly using the basic class RecursiveParallelVertex<> and not the application
                // specialized class that derives from this class. Since the ghost vertices are only
                // there to store edges and be able to diffuse in parallel. They donot contain any
                // appplication specific information. Also, use a different diffuse function that
                // specializes only in diffusing and nothing about application in it.
                RecursiveParallelVertex<Address_T> new_ghost_vertex;
                new_ghost_vertex.id = this->id;
                new_ghost_vertex.total_number_of_vertices = this->total_number_of_vertices;
                new_ghost_vertex.is_ghost_vertex = true;

                std::optional<Address> ghost_vertex_addr =
                    cca_simulator.allocate_and_insert_object_on_cc(
                        allocator, &new_ghost_vertex, sizeof(RecursiveParallelVertex<Address_T>));
                if (ghost_vertex_addr == std::nullopt) {
                    std::cerr << "Error: Not able to allocate ghost vertex\n";
                    exit(0);
                }
            }
            std::cout << "There is ghost vertex. Now sending there!\n";
            return ghost_vertex_addr.insert_edgeXXXX(cca_simulator, allocator, );

        } else {

            this->edges[this->number_of_edges].edge = dst_vertex_addr;
            this->edges[this->number_of_edges].weight = edge_weight;
            this->number_of_edges++;
        }
        return true;
    }

    // Insert an edge with weight on the host.
    auto insert_edge(host_edge_type dst_vertex_addr, u_int32_t edge_weight) -> bool
    {
        std::cerr << "Not a valid use of " << typeid(*this).name() << " type of Vertex!\n";
        exit(0);

        return false;
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

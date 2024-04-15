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

#include "Constants.hpp"
#include "SimpleVertex.hpp"
#include "VicinityMemoryAllocator.hpp"
using Allocator_T = VicinityMemoryAllocator;

#include "CyclicMemoryAllocator.hpp"
// using Allocator_T = CyclicMemoryAllocator;

// Forward declaration
template<typename Address_T, int edgelist_size>
struct RecursiveParallelVertex;

using ghost_type_level_1 = RecursiveParallelVertex<Address, edges_min>;
using ghost_type_level_greater_than_1 = RecursiveParallelVertex<Address, edges_max>;

// A convient wrapper for deciding how to invoke the function on vertex objects. It checks for the
// size of the local edgelist and sees if it was allocating using `edge_min` or `edge_max` and then
// uses that type to invoke the function. Note: This is not really necessary but is used to save
// space as most vertices are not big and have samll number of edges so don't have to allocate all
// of them with larger local edgelist size space and waste it. In future when something like
// std::vector is implemented in the runtime then we might not need that.
#define INVOKE_HANDLER_3(func, cc, addr, args)                                                     \
    do {                                                                                           \
        auto* parent_simple_vertex = static_cast<ghost_type_level_1*>(cc.get_object(addr));        \
        if (parent_simple_vertex->local_edgelist_size == edges_min) {                              \
            return func<ghost_type_level_1>(cc, addr, args);                                       \
        }                                                                                          \
        return func<ghost_type_level_greater_than_1>(cc, addr, args);                              \
    } while (0)

#define INVOKE_HANDLER_4(func, cc, addr, action_type, args)                                        \
    do {                                                                                           \
        auto* parent_simple_vertex = static_cast<ghost_type_level_1*>(cc.get_object(addr));        \
        if (parent_simple_vertex->local_edgelist_size == edges_min) {                              \
            return func<ghost_type_level_1>(cc, addr, action_type, args);                          \
        }                                                                                          \
        return func<ghost_type_level_greater_than_1>(cc, addr, action_type, args);                 \
    } while (0)

template<typename Address_T, int edgelist_size>
struct RecursiveParallelVertex : SimpleVertex<Address_T, edgelist_size>
{

    // Checking to see whether the user has mistakenly used this for host side allocation. If they
    // give `Edge.edge` type other than `Address` then it means they are intenting to allocate on
    // the host. This vertex object is not supposed to be allocated on the host. For that purpose
    // use the `SimpleVertex`.
    static_assert(std::is_same_v<Address_T, Address>);

    // For now only supported for 1, 2, and 3. See artibrate logic in the construction...
    // static_assert(ghost_children_max < 4);
    static_assert(GHOST_CHILDREN < 4);

    // ghost_children_max;
    inline static constexpr u_int32_t ghost_vertices_max_degree = GHOST_CHILDREN;

    // If this vertex is ghost vertex? Default is `false` meaning that it is the root/main vertex
    // not a ghost.
    bool is_ghost_vertex{};
    // Addresses of any ghost vertices that this vertex might have.
    std::optional<Address_T> ghost_vertices[RecursiveParallelVertex::ghost_vertices_max_degree]{};
    // Balance adding into the ghost vertices by having this iterator that goes in round-robin.
    u_int8_t next_insertion_in_ghost_iterator{};

    // Used to allocate the ghost vertices.
    Allocator_T ghost_vertex_allocator;

    // Private: allocate ghost
    template<typename ghost_type>
    void allocate_ghost(CCASimulator& cca_simulator, u_int32_t RPVO_level)
    {
        // Knowingly using the basic class RecursiveParallelVertex<> and not the application
        // specialized class that derives from this class. Since the ghost vertices are only
        // there to store edges and be able to diffuse in parallel. They donot contain any
        // appplication specific information. Also, in the future perhaps refactor to use a
        // different diffuse function that specializes only in diffusing and nothing about
        // application in it.
        // RecursiveParallelVertex<Address_T, edgelist_size> new_ghost_vertex;
        ghost_type new_ghost_vertex;

        // This won't really be used just giving the ghost the same id as parent.
        new_ghost_vertex.id = this->id;
        // Let is know about the graph size. Not sure whether this will be ever used...
        new_ghost_vertex.total_number_of_vertices = this->total_number_of_vertices;
        // Important to make sure to mark this as the ghost.
        new_ghost_vertex.is_ghost_vertex = true;

        std::optional<Address> ghost_vertex_addr = cca_simulator.allocate_and_insert_object_on_cc(
            this->ghost_vertex_allocator, &new_ghost_vertex, sizeof(ghost_type));
        if (ghost_vertex_addr == std::nullopt) {
            std::cerr << "Error: Not able to allocate ghost vertex dst: << " //<< dst_vertex_addr
                      << "\n";
            exit(0);
        }

        this->ghost_vertices[this->next_insertion_in_ghost_iterator] = ghost_vertex_addr;

        auto* ghost_vertex_accessor = static_cast<ghost_type*>(cca_simulator.get_object(
            this->ghost_vertices[this->next_insertion_in_ghost_iterator].value()));

        // This defines the center of vicinity for allocation. Currently, using the
        // ghost_vertex itself as the center of vicinity. That makes more sense than to have
        // the root non-ghost vertex as the center of vicinity. The `src_vertex_cc_id` is
        // currently not used but keeping it for future use for benchmarking.
        const u_int32_t source_vertex_cc_id_to_use =
            this->ghost_vertices[this->next_insertion_in_ghost_iterator].value().cc_id;

        ghost_vertex_accessor->init(cca_simulator, source_vertex_cc_id_to_use, RPVO_level);
    }

    // Recurssively add edge into the ghost vertex
    // Insert an edge with weight
    auto insert_edge_recurssively(CCASimulator& cca_simulator,
                                  u_int32_t src_vertex_cc_id,
                                  Address_T dst_vertex_addr,
                                  u_int32_t edge_weight,
                                  u_int32_t RPVO_level) -> bool
    {

        if (this->number_of_edges == edgelist_size) {

            if (!this->ghost_vertices[this->next_insertion_in_ghost_iterator].has_value()) {
                // Allocate ghost vertex since it does not exist.
                if (RPVO_level == 0) {
                    this->allocate_ghost<ghost_type_level_1>(cca_simulator, RPVO_level);
                } else {
                    this->allocate_ghost<ghost_type_level_greater_than_1>(cca_simulator,
                                                                          RPVO_level);
                }
            }

            // Note: This whole RPVO_level == 0 if else is here so that we can have level 0 and 1
            // with local edgelist size of `edges_min` and the rest of the higher levels in RPVO
            // hierarchy to have `edges_max` local edgelist size. It kind of mimics a progressively
            // growing list such as std::vector without having to do runtime memory management. It
            // can actually be done by having the local edgelist size be implemented like
            // std::vector but that is work for future.
            bool success = false;
            if (RPVO_level == 0) {
                auto* ghost_vertex_accessor =
                    static_cast<ghost_type_level_1*>(cca_simulator.get_object(
                        this->ghost_vertices[this->next_insertion_in_ghost_iterator].value()));

                success = ghost_vertex_accessor->insert_edge_recurssively(
                    cca_simulator, src_vertex_cc_id, dst_vertex_addr, edge_weight, RPVO_level + 1);
            } else {
                auto* ghost_vertex_accessor =
                    static_cast<ghost_type_level_greater_than_1*>(cca_simulator.get_object(
                        this->ghost_vertices[this->next_insertion_in_ghost_iterator].value()));

                success = ghost_vertex_accessor->insert_edge_recurssively(
                    cca_simulator, src_vertex_cc_id, dst_vertex_addr, edge_weight, RPVO_level + 1);
            }

            if (success) {
                // Increment the global edges count for this vertex.
                this->outbound_degree++;

                bool arbitrate_ghost = this->outbound_degree % edges_max == 0;

                // Only will happen once when this RPVO's first ghost is full. It will then create a
                // new ghost and add edges_min edges to it. It helps in saving space and not
                // creating too many ghosts.
                if (RPVO_level == 0 && (this->outbound_degree == 2 * edges_min)) {
                    arbitrate_ghost = true;
                }

                // Only will happen once when this RPVO's second (if exists) ghost is full. It will
                // then create a new ghost and add edges_min edges to it. It helps in saving space
                // and not creating too many ghosts.
                if constexpr (RecursiveParallelVertex::ghost_vertices_max_degree == 3) {
                    if (RPVO_level == 0 && (this->outbound_degree == 3 * edges_min)) {
                        arbitrate_ghost = true;
                    }
                }

                if (arbitrate_ghost) {
                    // In order to balance the ghost tree iterate over them when adding. This will
                    // make sure a balanced tree.
                    this->next_insertion_in_ghost_iterator =
                        (this->next_insertion_in_ghost_iterator + 1) %
                        RecursiveParallelVertex::ghost_vertices_max_degree;
                }
                return true;
            } else {
                return false; // insertion failed.
            }

        } else {

            this->edges[this->number_of_edges].edge = dst_vertex_addr;
            if constexpr (weighted_edge) {
                this->edges[this->number_of_edges].weight = edge_weight;
            }
            // Only increments the currect ghost/root vertex edges.
            this->number_of_edges++;
            // Increment the global edges count for this vertex. For a ghost vertex this is all the
            // edges contains in itself in its edge list and all in its child ghost vertices.
            this->outbound_degree++;
        }
        return true;
    }

    // Recurssively add edge into the ghost vertex
    // Insert an edge with weight
    auto insert_edge_recurssively(CCASimulator& cca_simulator,
                                  u_int32_t src_vertex_cc_id,
                                  Address_T this_vertex_addr_in,
                                  Address_T dst_vertex_addr,
                                  u_int32_t edge_weight,
                                  ActionArgumentType args_for_continuation,
                                  Address terminator,
                                  CCAFunctionEvent continuation,
                                  u_int32_t RPVO_level) -> bool
    {
        /* std::cout << "insert_edge_recurssively: this_vertex_addr_in: " << this_vertex_addr_in
                  << ", dst_vertex_addr: " << dst_vertex_addr << "\n"; */

        if (this->number_of_edges == edgelist_size) {

            if (!this->ghost_vertices[this->next_insertion_in_ghost_iterator].has_value()) {
                // Allocate ghost vertex since it does not exist.
                if (RPVO_level == 0) {
                    this->allocate_ghost<ghost_type_level_1>(cca_simulator, RPVO_level);
                } else {
                    this->allocate_ghost<ghost_type_level_greater_than_1>(cca_simulator,
                                                                          RPVO_level);
                }
            }

            Address ghost_vertex_addr =
                this->ghost_vertices[next_insertion_in_ghost_iterator].value();

            // This defines the center of vicinity for allocation. Currently, using the ghost_vertex
            // itself as the center of vicinity. That makes more sense than to have the root
            // non-ghost vertex as the center of vicinity. The `src_vertex_cc_id` is to be used for
            // germinating the continuation action.
            const u_int32_t source_vertex_cc_id_to_use = this_vertex_addr_in.cc_id;

            // Note : This whole RPVO_level == 0 if else is here so that we can have level 0 and 1
            // with local edgelist size of `edges_min` and the rest of the higher levels in RPVO
            // hierarchy to have `edges_max` local edgelist size. It kind of mimics a
            // progressively growing list such as std::vector without having to do runtime
            // memory management. It can actually be done by having the local edgelist size be
            // implemented like std::vector but that is work for future.
            bool success = false;
            if (RPVO_level == 0) {
                auto* ghost_vertex_accessor =
                    static_cast<ghost_type_level_1*>(cca_simulator.get_object(ghost_vertex_addr));

                success =
                    ghost_vertex_accessor->insert_edge_recurssively(cca_simulator,
                                                                    source_vertex_cc_id_to_use,
                                                                    ghost_vertex_addr,
                                                                    dst_vertex_addr,
                                                                    edge_weight,
                                                                    args_for_continuation,
                                                                    terminator,
                                                                    continuation,
                                                                    RPVO_level + 1);
            } else {
                auto* ghost_vertex_accessor = static_cast<ghost_type_level_greater_than_1*>(
                    cca_simulator.get_object(ghost_vertex_addr));

                success =
                    ghost_vertex_accessor->insert_edge_recurssively(cca_simulator,
                                                                    source_vertex_cc_id_to_use,
                                                                    ghost_vertex_addr,
                                                                    dst_vertex_addr,
                                                                    edge_weight,
                                                                    args_for_continuation,
                                                                    terminator,
                                                                    continuation,
                                                                    RPVO_level + 1);
            }

            if (success) {
                // Increment the global edges count for this vertex.
                this->outbound_degree++;

                bool arbitrate_ghost = this->outbound_degree % edges_max == 0;
                if (RPVO_level == 0 && (this->outbound_degree == 2 * edges_min)) {
                    arbitrate_ghost = true;
                }

                if (arbitrate_ghost) {
                    // Inorder to balance the ghost tree iterate over them when adding. This will
                    // make sure a balanced tree.
                    this->next_insertion_in_ghost_iterator =
                        (this->next_insertion_in_ghost_iterator + 1) %
                        RecursiveParallelVertex::ghost_vertices_max_degree;
                }
                return true;
            } else {
                return false; // insertion failed.
            }

        } else {

            this->edges[this->number_of_edges].edge = dst_vertex_addr;
            if constexpr (weighted_edge) {
                this->edges[this->number_of_edges].weight = edge_weight;
            }
            // Only increments the currect ghost/root vertex edges.
            this->number_of_edges++;
            // Increment the global edges count for this vertex. For a ghost vertex this is all the
            // edges contains in itself in its edge list and all in its child ghost vertices.
            this->outbound_degree++;

            cca_simulator.germinate_action(Action(this_vertex_addr_in,
                                                  terminator,
                                                  actionType::germinate_action,
                                                  true,
                                                  args_for_continuation,
                                                  cca_simulator.function_events.null_event_true_id,
                                                  cca_simulator.function_events.null_event_true_id,
                                                  cca_simulator.function_events.null_event_true_id,
                                                  continuation));
        }
        return true;
    }

    // Insert an edge with weight
    auto insert_edge(CCASimulator& cca_simulator,
                     u_int32_t src_vertex_cc_id,
                     Address_T dst_vertex_addr,
                     u_int32_t edge_weight) -> bool
    {
        assert(!this->is_ghost_vertex);

        return this->insert_edge_recurssively(
            cca_simulator, src_vertex_cc_id, dst_vertex_addr, edge_weight, 0);
    }

    // Insert an edge with weight with continuation
    auto insert_edge(CCASimulator& cca_simulator,
                     u_int32_t src_vertex_cc_id,
                     Address_T src_vertex_addr,
                     Address_T dst_vertex_addr,
                     u_int32_t edge_weight,
                     ActionArgumentType args_for_continuation,
                     Address terminator,
                     CCAFunctionEvent continuation) -> bool
    {
        assert(!this->is_ghost_vertex);

        return this->insert_edge_recurssively(cca_simulator,
                                              src_vertex_cc_id,
                                              src_vertex_addr,
                                              dst_vertex_addr,
                                              edge_weight,
                                              args_for_continuation,
                                              terminator,
                                              continuation,
                                              0);
    }

    // Insert an edge with weight on the host.
    auto insert_edge(host_edge_type dst_vertex_addr, u_int32_t edge_weight) -> bool
    {
        std::cerr << "Not a valid use of " << typeid(*this).name() << " type of Vertex!\n";
        exit(0);

        return false;
    }

    auto init(CCASimulator& cca_simulator, u_int32_t source_cc_id, u_int32_t RPVO_level) -> bool
    {
        if constexpr (std::is_same_v<Allocator_T, CyclicMemoryAllocator>) {
            this->ghost_vertex_allocator =
                CyclicMemoryAllocator(source_cc_id, cca_simulator.total_compute_cells);
            return true;
        } else if constexpr (std::is_same_v<Allocator_T, VicinityMemoryAllocator>) {
            // Right now defining the vicinity boundary as constant but later this can be made
            // sophisticated by using some measure like the outbound edges and then for each
            // vertex spread its vicinity of allocation such that large vertices have a larger
            // vicinity. 2 and 2 = 5x5 actually.
            u_int32_t vicinity_rows = vicinity_radius; // 2;
            u_int32_t vicinity_cols = vicinity_radius; // 2;
            if (RPVO_level < 1) {
                vicinity_rows = 1;
                vicinity_cols = 1;
            }

            this->ghost_vertex_allocator = VicinityMemoryAllocator(
                Cell::cc_id_to_cooridinate(
                    source_cc_id, cca_simulator.shape_of_compute_cells, cca_simulator.dim_y),
                vicinity_rows,
                vicinity_cols,
                cca_simulator.dim_x,
                cca_simulator.dim_y,
                cca_simulator.shape_of_compute_cells);

            return true;
        } else {
            std::cerr << "Allocator type not supported: " << typeid(Allocator_T).name() << "\n";
            exit(0);
        }
    }
    RecursiveParallelVertex() = default;
    ~RecursiveParallelVertex() = default;
};

// Print the RecursiveParallelVertex vertex
template<u_int32_t edgelist_size>
inline void
print_RecursiveParallelVertex(const RecursiveParallelVertex<Address, edgelist_size>* vertex,
                              const Address& vertex_addr)
{
    std::cout << "Vertex ID: " << vertex->id << ", Addr: "
              << vertex_addr
              //<< " sssp_distance: " << vertex->sssp_distance
              << " deficit: " << vertex->terminator.deficit << "\n";

    for (u_int32_t i = 0; i < vertex->number_of_edges; i++) {

        if constexpr (weighted_edge) {
            std::cout << "\t\t[" << vertex->edges[i].edge << ", {w: " << vertex->edges[i].weight
                      << "} ]";
        } else {
            std::cout << "\t\t[" << vertex->edges[i].edge << "]";
        }
    }
    std::cout << std::endl;
}

#endif // RECURSIVE_PARALLEL_Vertex_HPP

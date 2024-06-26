/*
BSD 3-Clause License

Copyright (c) 2024, Bibrak Qamar

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

// Datastructures
#include "Graph.hpp"
#include "StreamingRecursiveParallelVertex.hpp"

#include "cmdparser.hpp"

// For memcpy()
#include <cstring>

#include <fstream>

inline static constexpr u_int32_t undefined_level = 999999;

// This is what the action carries as payload.
struct BFSArguments
{
    u_int32_t level;
    u_int32_t src_vertex_id;
};

struct InsertEdgeArgumentsBFS : public InsertEdgeArguments
{
    u_int32_t level{};
};

template<typename Vertex_T>
struct BFSVertex : Vertex_T
{
    inline static constexpr u_int32_t max_level = undefined_level;
    u_int32_t bfs_level;

    BFSVertex(u_int32_t id_in, u_int32_t total_number_of_vertices_in)
        : bfs_level(BFSVertex::max_level)
    {
        this->id = id_in;
        this->number_of_edges = 0;
        this->total_number_of_vertices = total_number_of_vertices_in;
    }

    // Nothing to do.
    void configure_derived_class_LCOs() {}

    BFSVertex() {}
    ~BFSVertex() {}
};

// CCAFunctionEvent ids for the BFS action: predicate, work, and diffuse.
// In the main register the functions with the CCASimulator chip and get their ids.
extern CCAFunctionEvent dynamic_bfs_predicate;
extern CCAFunctionEvent dynamic_bfs_work;
extern CCAFunctionEvent dynamic_bfs_diffuse_predicate;
extern CCAFunctionEvent dynamic_bfs_diffuse;

extern CCAFunctionEvent dynamic_bfs_insert_edge_predicate;
extern CCAFunctionEvent dynamic_bfs_insert_edge_work;
extern CCAFunctionEvent dynamic_bfs_insert_edge_diffuse_predicate;
extern CCAFunctionEvent dynamic_bfs_insert_edge_diffuse;

extern CCAFunctionEvent allocate;
extern CCAFunctionEvent dynamic_bfs_insert_edge_continuation_ghost_allocate_return;

template<typename ghost_type>
inline auto
dynamic_bfs_predicate_T(ComputeCell& cc,
                        const Address addr,
                        const ActionArgumentType args) -> Closure
{
    cc.apply_CPI(1);

    // First check whether this is a ghost vertex.If it is then always predicate true.
    // parent word is used in the sense that `RecursiveParallelVertex` is the parent class.
    auto* parent_recursive_parralel_vertex = static_cast<ghost_type*>(cc.get_object(addr));

    if (parent_recursive_parralel_vertex->is_ghost_vertex) {
        return Closure(cc.null_true_event, nullptr);
    }

    auto* v = static_cast<BFSVertex<ghost_type>*>(cc.get_object(addr));
    BFSArguments const bfs_args = cca_get_action_argument<BFSArguments>(args);

    u_int32_t const incoming_level = bfs_args.level;

    if (v->bfs_level > incoming_level) {
        return Closure(cc.null_true_event, nullptr);
    }
    return Closure(cc.null_false_event, nullptr);
}

inline auto
dynamic_bfs_predicate_func(ComputeCell& cc,
                           const Address addr,
                           actionType /* action_type_in */,
                           const ActionArgumentType args) -> Closure
{
    INVOKE_HANDLER_3(dynamic_bfs_predicate_T, cc, addr, args);
}

template<typename ghost_type>
inline auto
dynamic_bfs_work_T(ComputeCell& cc, const Address addr, const ActionArgumentType args) -> Closure
{
    cc.apply_CPI(1);

    // First check whether this is a ghost vertex. If it is then don't perform any work.
    // parent word is used in the sense that `RecursiveParallelVertex` is the parent class.
    auto* parent_recursive_parralel_vertex = static_cast<ghost_type*>(cc.get_object(addr));

    if (parent_recursive_parralel_vertex->is_ghost_vertex) {
        return Closure(cc.null_true_event, nullptr);
    }

    auto* v = static_cast<BFSVertex<ghost_type>*>(cc.get_object(addr));
    BFSArguments const bfs_args = cca_get_action_argument<BFSArguments>(args);

    u_int32_t const incoming_level = bfs_args.level;

    // Update level with the new level
    v->bfs_level = incoming_level;
    return Closure(cc.null_true_event, nullptr);
}

inline auto
dynamic_bfs_work_func(ComputeCell& cc,
                      const Address addr,
                      actionType /* action_type_in */,
                      const ActionArgumentType args) -> Closure
{
    INVOKE_HANDLER_3(dynamic_bfs_work_T, cc, addr, args);
}

template<typename ghost_type>
inline auto
dynamic_bfs_diffuse_predicate_T(ComputeCell& cc,
                                const Address addr,
                                const ActionArgumentType args) -> Closure
{
    cc.apply_CPI(1);

    // First check whether this is a ghost vertex. If it is then always predicate true.
    // parent word is used in the sense that `RecursiveParallelVertex` is the parent class.
    auto* parent_recursive_parralel_vertex = static_cast<ghost_type*>(cc.get_object(addr));

    if (parent_recursive_parralel_vertex->is_ghost_vertex) {
        return Closure(cc.null_true_event, nullptr);
    }

    auto* v = static_cast<BFSVertex<ghost_type>*>(cc.get_object(addr));
    BFSArguments const bfs_args = cca_get_action_argument<BFSArguments>(args);

    u_int32_t const incoming_level = bfs_args.level;

    if (v->bfs_level == incoming_level) {
        return Closure(cc.null_true_event, nullptr);
    }
    return Closure(cc.null_false_event, nullptr);
}

inline auto
dynamic_bfs_diffuse_predicate_func(ComputeCell& cc,
                                   const Address addr,
                                   actionType /* action_type_in */,
                                   const ActionArgumentType args) -> Closure
{
    INVOKE_HANDLER_3(dynamic_bfs_diffuse_predicate_T, cc, addr, args);
}

template<typename ghost_type>
inline auto
dynamic_bfs_diffuse_T(ComputeCell& cc, const Address addr, const ActionArgumentType args) -> Closure
{

    // Get the hold of the parent ghost vertex. If it is ghost then simply perform diffusion.
    auto* parent_recursive_parralel_vertex = static_cast<ghost_type*>(cc.get_object(addr));
    bool this_is_ghost_vertex = parent_recursive_parralel_vertex->is_ghost_vertex;

    auto* v = static_cast<BFSVertex<ghost_type>*>(cc.get_object(addr));

    u_int32_t current_level = BFSVertex<ghost_type>::max_level;
    if (this_is_ghost_vertex) {
        BFSArguments const bfs_args = cca_get_action_argument<BFSArguments>(args);
        current_level = bfs_args.level;
    } else {
        current_level = v->bfs_level;
    }

    BFSArguments level_to_send;
    level_to_send.level = current_level;
    level_to_send.src_vertex_id = v->id;

    ActionArgumentType const args_for_ghost_vertices =
        cca_create_action_argument<BFSArguments>(level_to_send);

    // Note: The application vertex type is derived from the parent `RecursiveParallelVertex`
    // therefore using the derived pointer. It works for both. First diffuse to the ghost vertices.
    for (u_int32_t ghosts_iterator = 0; ghosts_iterator < ghost_type::ghost_vertices_max_degree;
         ghosts_iterator++) {
        if (v->ghost_vertices[ghosts_iterator].is_fulfilled()) {
            // if (v->ghost_vertices[ghosts_iterator].has_value()) {

            cc.diffuse(Action(v->ghost_vertices[ghosts_iterator].get(),
                              addr,
                              actionType::application_action,
                              true,
                              args_for_ghost_vertices,
                              dynamic_bfs_predicate,
                              dynamic_bfs_work,
                              dynamic_bfs_diffuse_predicate,
                              dynamic_bfs_diffuse));
        }
    }

    for (int i = 0; i < v->number_of_edges; i++) {

        level_to_send.level = current_level + 1;
        ActionArgumentType const args_x = cca_create_action_argument<BFSArguments>(level_to_send);

        cc.diffuse(Action(v->edges[i].edge,
                          addr,
                          actionType::application_action,
                          true,
                          args_x,
                          dynamic_bfs_predicate,
                          dynamic_bfs_work,
                          dynamic_bfs_diffuse_predicate,
                          dynamic_bfs_diffuse));
    }

    return Closure(cc.null_false_event, nullptr);
}

inline auto
dynamic_bfs_diffuse_func(ComputeCell& cc,
                         const Address addr,
                         actionType /* action_type_in */,
                         const ActionArgumentType args) -> Closure
{
    INVOKE_HANDLER_3(dynamic_bfs_diffuse_T, cc, addr, args);
}

// TODO: This is not the purist way. Will have to implement a way in which we call the system
// allocate and then have it initialize the ghost vertex there. But right now putting both function
// in a single function here.
template<typename ghost_type>
inline auto
allocate_T(ComputeCell& cc, const Address addr, const ActionArgumentType args) -> Closure
{

    AllocateArguments const allocate_request_args =
        cca_get_action_argument<AllocateArguments>(args);

    ghost_type new_ghost_vertex;

    // This won't really be used just giving the ghost the same id as parent.
    new_ghost_vertex.id = allocate_request_args.vertex_id;
    // Let is know about the graph size. Not sure whether this will be ever used...
    new_ghost_vertex.total_number_of_vertices = allocate_request_args.total_number_of_vertices;
    // Important to make sure to mark this as the ghost.
    new_ghost_vertex.is_ghost_vertex = true;

    // This defines the center of vicinity for allocation. Currently, using the
    // ghost_vertex itself as the center of vicinity. That makes more sense than to have
    // the root non-ghost vertex as the center of vicinity.
    new_ghost_vertex.ghost_vertex_allocator =
        VicinityMemoryAllocator(cc.cooridates, 2, 2, cc.dim_x, cc.dim_y, cc.shape);

    std::optional<Address> ghost_vertex_addr =
        cc.create_object_in_memory(&new_ghost_vertex, allocate_request_args.size_in_bytes);

    if (ghost_vertex_addr == std::nullopt) {
        std::cerr << "Error: Not able to allocate ghost vertex dst: << " //<< dst_vertex_addr
                  << "\n";
        exit(0);
    }

    // cc.apply_CPI(1);
    AllocateReturnArguments allocated_memory;
    allocated_memory.new_memory_addrs = ghost_vertex_addr.value();
    allocated_memory.ghost_vertices_future_lco_index =
        allocate_request_args.ghost_vertices_future_lco_index;

    ActionArgumentType allocate_reply_back_args =
        cca_create_action_argument<AllocateReturnArguments>(allocated_memory);

    cc.diffuse(Action(allocate_request_args.src_vertex_addrs,
                      addr,
                      actionType::application_action, // TODO: later have runtime_action
                      true,
                      allocate_reply_back_args,
                      cc.null_true_event,
                      allocate_request_args.continuation,
                      cc.null_false_event,
                      cc.null_false_event));

    return Closure(cc.null_false_event, nullptr);
}

inline auto
allocate_bfs_func(ComputeCell& cc,
                  const Address addr,
                  actionType /* action_type_in */,
                  const ActionArgumentType args) -> Closure
{
    INVOKE_HANDLER_3(allocate_T, cc, addr, args);
}

// For the streaming edge insertion.
inline auto
dynamic_bfs_insert_edge_predicate_func(ComputeCell& cc,
                                       const Address addr,
                                       actionType /* action_type_in */,
                                       const ActionArgumentType args) -> Closure
{
    // Set to always true because the edge must always be inserted and its work needs to be
    // performed.
    return Closure(cc.null_true_event, nullptr);
}

// This is the continuation anonymous action.
template<typename ghost_type>
inline auto
dynamic_bfs_insert_edge_continuation_ghost_allocate_return_T(ComputeCell& cc,
                                                             const Address addr,
                                                             const ActionArgumentType args)
    -> Closure
{

    auto* v = static_cast<ghost_type*>(cc.get_object(addr));

    AllocateReturnArguments const allocate_return_args =
        cca_get_action_argument<AllocateReturnArguments>(args);

    const u_int8_t ghost_future_lco_index = allocate_return_args.ghost_vertices_future_lco_index;
    v->ghost_vertices[ghost_future_lco_index] = allocate_return_args.new_memory_addrs;

    // for loop in Future LCO's queue and send actions along to the ghost.
    while (auto closure = v->ghost_vertices[ghost_future_lco_index].dequeue()) {
        // for (int i = 0; i < v->ghost_vertices[ghost_future_lco_index].queue_size; i++) {

        /* std::cout << "v id: " << v->id << ", dequeueing, ghost_future_lco_index: "
                  << static_cast<u_int8_t>(ghost_future_lco_index) << "\n"; */

        ActionArgumentType args_for_ghost = closure.value().second;
        if (!v->is_ghost_vertex) {
            InsertEdgeArguments const insert_edge_args =
                cca_get_action_argument<InsertEdgeArguments>(args_for_ghost);

            auto* root_rpvo_bfs_vertex = static_cast<BFSVertex<ghost_type>*>(cc.get_object(addr));

            InsertEdgeArgumentsBFS send_to_ghost;
            send_to_ghost.dst_vertex_addrs = insert_edge_args.dst_vertex_addrs;
            send_to_ghost.edge_weight = insert_edge_args.edge_weight;
            send_to_ghost.level = root_rpvo_bfs_vertex->bfs_level;

            args_for_ghost = cca_create_action_argument<InsertEdgeArgumentsBFS>(send_to_ghost);
        }

        cc.diffuse(Action(v->ghost_vertices[ghost_future_lco_index].get(),
                          addr,
                          actionType::application_action,
                          true,
                          args_for_ghost,
                          dynamic_bfs_insert_edge_predicate,
                          dynamic_bfs_insert_edge_work,
                          dynamic_bfs_insert_edge_diffuse_predicate,
                          dynamic_bfs_insert_edge_diffuse));
    }
    return Closure(cc.null_false_event, nullptr);
}

inline auto
dynamic_bfs_insert_edge_continuation_ghost_allocate_return_func(ComputeCell& cc,
                                                                const Address addr,
                                                                actionType /* action_type_in */,
                                                                const ActionArgumentType args)
    -> Closure
{
    INVOKE_HANDLER_3(dynamic_bfs_insert_edge_continuation_ghost_allocate_return_T, cc, addr, args);
}

template<typename ghost_type>
inline auto
dynamic_bfs_insert_edge_work_T(ComputeCell& cc,
                               const Address addr,
                               const ActionArgumentType args) -> Closure
{

    // Get the vertex object.
    // auto* v = static_cast<BFSVertex<ghost_type>*>(cc.get_object(addr));

    // First check whether this is a ghost vertex. If it is then use the
    // `BFSArgumentsInsertEdgeForGhost` to get the bfs level that came from the root RPVO.
    // auto* parent_recursive_parralel_vertex = static_cast<ghost_type*>(cc.get_object(addr));

    auto* v = static_cast<ghost_type*>(cc.get_object(addr));

    // Insert the edge.
    if (v->number_of_edges == v->local_edgelist_size) {
        /* std::cerr << "cc id: " << cc.id << ", v->id:" << v->id
                  << ", v->number_of_edges: " << v->number_of_edges
                  << ", v->local_edgelist_size: " << v->local_edgelist_size << ", addr: " << addr
                  << ", dst addrs: " << insert_edge_args.dst_vertex_addrs
                  << ", w: " << insert_edge_args.edge_weight << std::endl;
        std::cout << "551 is ptr: " << static_cast<int*>(cc.get_object(addr)) << "\n";
        std::cerr << "Fatal: Cannot insert edge as the vertex is out of edgelist" << std::endl;
        exit(0); */

        if (v->ghost_vertices[v->next_insertion_in_ghost_iterator].is_empty()) {

            /* std::cout << "v id: " << v->id
                      << ", creating ghost, v->next_insertion_in_ghost_iterator: "
                      << static_cast<u_int8_t>(v->next_insertion_in_ghost_iterator) << "\n"; */

            if (!v->ghost_vertices[v->next_insertion_in_ghost_iterator].enqueue(
                    Closure(dynamic_bfs_insert_edge_continuation_ghost_allocate_return, args))) {
                std::cerr << "Error: Not able to enqueue in ghost future \n";
                exit(0);
            }

            v->ghost_vertices[v->next_insertion_in_ghost_iterator].set_state(
                lcoFutureState::pending);

            // Send alloc to
            const u_int32_t cc_id_that_allocates =
                v->ghost_vertex_allocator.get_next_available_cc(cc);

            AllocateArguments allocation_request;
            allocation_request.src_vertex_addrs = addr;
            allocation_request.ghost_vertices_future_lco_index =
                v->next_insertion_in_ghost_iterator;
            allocation_request.size_in_bytes = sizeof(ghost_type);

            allocation_request.continuation =
                dynamic_bfs_insert_edge_continuation_ghost_allocate_return;

            ActionArgumentType const args_to_allocate =
                cca_create_action_argument<AllocateArguments>(allocation_request);

            cc.diffuse(Action(Address(cc_id_that_allocates, 0),
                              addr,
                              actionType::application_action, // TODO: later have runtime_action
                              true,
                              args_to_allocate,
                              cc.null_true_event,
                              allocate, // cc.allocate_event,
                              cc.null_false_event,
                              cc.null_false_event));

        } else if (v->ghost_vertices[v->next_insertion_in_ghost_iterator].is_pending()) {

            /* std::cout << "v id: " << v->id
                      << ", pending state found, v->next_insertion_in_ghost_iterator: "
                      << static_cast<u_int8_t>(v->next_insertion_in_ghost_iterator) << "\n"; */

            if (!v->ghost_vertices[v->next_insertion_in_ghost_iterator].enqueue(
                    Closure(dynamic_bfs_insert_edge_continuation_ghost_allocate_return, args))) {
                std::cerr << "Error: Not able to enqueue in ghost future \n";
                exit(0);
            }

        } else { // The ghost exists

            if (v->is_ghost_vertex) {
                cc.diffuse(Action(v->ghost_vertices[v->next_insertion_in_ghost_iterator].get(),
                                  addr,
                                  actionType::application_action,
                                  true,
                                  args,
                                  dynamic_bfs_insert_edge_predicate,
                                  dynamic_bfs_insert_edge_work,
                                  dynamic_bfs_insert_edge_diffuse_predicate,
                                  dynamic_bfs_insert_edge_diffuse));
            } else {

                InsertEdgeArguments const insert_edge_args =
                    cca_get_action_argument<InsertEdgeArguments>(args);

                auto* root_rpvo_bfs_vertex =
                    static_cast<BFSVertex<ghost_type>*>(cc.get_object(addr));

                InsertEdgeArgumentsBFS send_to_ghost;
                send_to_ghost.dst_vertex_addrs = insert_edge_args.dst_vertex_addrs;
                send_to_ghost.edge_weight = insert_edge_args.edge_weight;
                send_to_ghost.level = root_rpvo_bfs_vertex->bfs_level;

                const ActionArgumentType args_for_ghost =
                    cca_create_action_argument<InsertEdgeArgumentsBFS>(send_to_ghost);

                cc.diffuse(Action(v->ghost_vertices[v->next_insertion_in_ghost_iterator].get(),
                                  addr,
                                  actionType::application_action,
                                  true,
                                  args_for_ghost,
                                  dynamic_bfs_insert_edge_predicate,
                                  dynamic_bfs_insert_edge_work,
                                  dynamic_bfs_insert_edge_diffuse_predicate,
                                  dynamic_bfs_insert_edge_diffuse));
            }
        }

        // Increment the global edges count for this vertex. For a ghost vertex this is all the
        // edges contained in itself in its edge list and all in its child ghost vertices.
        v->outbound_degree++;
        /*  std::cout << "\tv id: " << v->id << ", v->outbound_degree: " << v->outbound_degree
                   << ", v->next_insertion_in_ghost_iterator: "
                   << static_cast<int>(v->next_insertion_in_ghost_iterator) << "\n"; */
        bool arbitrate_ghost = v->outbound_degree % edges_min == 0;
        if (arbitrate_ghost) {
            v->next_insertion_in_ghost_iterator =
                (v->next_insertion_in_ghost_iterator + 1) % v->ghost_vertices_max_degree;
        }

        return Closure(cc.null_false_event, nullptr);
    } else {

        InsertEdgeArguments const insert_edge_args =
            cca_get_action_argument<InsertEdgeArguments>(args);

        // cc.apply_CPI(LOAD_STORE_CPI * 2);
        v->edges[v->number_of_edges].edge = insert_edge_args.dst_vertex_addrs;
        if constexpr (weighted_edge) {
            v->edges[v->number_of_edges].weight = insert_edge_args.edge_weight;
        }
        // Only increments the currect ghost/root vertex edges.
        cc.apply_CPI(ADD_CPI);
        v->number_of_edges++;
        // Increment the global edges count for this vertex. For a ghost vertex this is all the
        // edges contains in itself in its edge list and all in its child ghost vertices.
        v->outbound_degree++;

        // When we are doing batched streaming BFS, in the final increment we germinate BFS action
        // and that goes the BFS like a static bfs.
        // return Closure(cc.null_false_event, nullptr);

        // When we are doing streaming BFS
        if (v->is_ghost_vertex) {
            return Closure(cc.null_true_event, nullptr);
        } else {

            auto* root_rpvo_bfs_vertex = static_cast<BFSVertex<ghost_type>*>(cc.get_object(addr));

            InsertEdgeArgumentsBFS send_to_ghost;
            send_to_ghost.dst_vertex_addrs = insert_edge_args.dst_vertex_addrs;
            send_to_ghost.edge_weight = insert_edge_args.edge_weight;
            send_to_ghost.level = root_rpvo_bfs_vertex->bfs_level;

            const ActionArgumentType args_for_ghost =
                cca_create_action_argument<InsertEdgeArgumentsBFS>(send_to_ghost);

            return Closure(cc.null_true_event, args_for_ghost);
        }
    }
    return Closure(cc.null_false_event, nullptr);
}

inline auto
dynamic_bfs_insert_edge_work_func(ComputeCell& cc,
                                  const Address addr,
                                  actionType /* action_type_in */,
                                  const ActionArgumentType args) -> Closure
{
    INVOKE_HANDLER_3(dynamic_bfs_insert_edge_work_T, cc, addr, args);
}

template<typename ghost_type>
inline auto
dynamic_bfs_insert_edge_diffuse_predicate_T(ComputeCell& cc,
                                            const Address addr,
                                            const ActionArgumentType args) -> Closure
{
    // Get the vertex object.
    auto* v = static_cast<BFSVertex<ghost_type>*>(cc.get_object(addr));

    InsertEdgeArgumentsBFS const insert_edge_bfs_args =
        cca_get_action_argument<InsertEdgeArgumentsBFS>(args);

    if (BFSVertex<ghost_type>::max_level == insert_edge_bfs_args.level) {
        return Closure(cc.null_false_event, nullptr);
    }
    return Closure(cc.null_true_event, nullptr);
}

inline auto
dynamic_bfs_insert_edge_diffuse_predicate_func(ComputeCell& cc,
                                               const Address addr,
                                               actionType /* action_type_in */,
                                               const ActionArgumentType args) -> Closure
{
    INVOKE_HANDLER_3(dynamic_bfs_insert_edge_diffuse_predicate_T, cc, addr, args);
}

template<typename ghost_type>
inline auto
dynamic_bfs_insert_edge_diffuse_T(ComputeCell& cc,
                                  const Address addr,
                                  const ActionArgumentType args) -> Closure
{

    // Get the vertex object.
    auto* v = static_cast<BFSVertex<ghost_type>*>(cc.get_object(addr));
    InsertEdgeArgumentsBFS const insert_edge_args =
        cca_get_action_argument<InsertEdgeArgumentsBFS>(args);

    // send to insert_edge_args.dst_vertex_addrs;
    BFSArguments level_to_send;
    level_to_send.level = insert_edge_args.level + 1;
    level_to_send.src_vertex_id = v->id;

    ActionArgumentType const args_x = cca_create_action_argument<BFSArguments>(level_to_send);

    cc.diffuse(Action(insert_edge_args.dst_vertex_addrs,
                      addr,
                      actionType::application_action,
                      true,
                      args_x,
                      dynamic_bfs_predicate,
                      dynamic_bfs_work,
                      dynamic_bfs_diffuse_predicate,
                      dynamic_bfs_diffuse));

    return Closure(cc.null_false_event, nullptr);
}

inline auto
dynamic_bfs_insert_edge_diffuse_func(ComputeCell& cc,
                                     const Address addr,
                                     actionType /* action_type_in */,
                                     const ActionArgumentType args) -> Closure
{
    INVOKE_HANDLER_3(dynamic_bfs_insert_edge_diffuse_T, cc, addr, args);
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
    parser.set_required<u_int32_t>("root", "bfsroot", "Root vertex for Breadth First Search (BFS)");
    parser.set_optional<bool>(
        "verify",
        "verification",
        0,
        "Enable verification of the calculated levels with the levels provided "
        "in the accompanying .bfs file");
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

    parser.set_optional<u_int32_t>(
        "mesh", "mesh_type", 0, "Type of the mesh:\n\t0: Regular\n\t1: Torus.");

    parser.set_optional<u_int32_t>("route", "routing_policy", 0, "Routing algorithm to use.");

    parser.set_optional<u_int32_t>(
        "increments", "dynamic_increments", 0, "How many times does that graph evolve.");

    parser.set_optional<bool>(
        "shuffle",
        "shuffle_vertices",
        0,
        "Randomly shuffle the vertex list so as to avoid any pattern in the graph based on vertex "
        "IDs. This appears to be the case for certain RMAT graphs.");

    parser.set_optional<u_int32_t>("trail", "trail_number", 0, "Trail number for this experiment.");
}

struct BFSCommandLineArguments
{
    // BFS root vertex
    u_int32_t root_vertex{};
    // Cross check the calculated bfs lengths from root with the lengths provided in .bfs file
    bool verify_results{};
    // Configuration related to the input data graph
    std::string input_graph_path;
    std::string graph_name;
    // Optional output directory path
    std::string output_file_directory;
    // Get the depth of Htree.
    u_int32_t hdepth{};

    // Get the rows and columbs of cells that are served by a single end Htree node. This will
    // help in construction of the CCA chip, Htree, and routing.
    u_int32_t hx{};
    u_int32_t hy{};

    // Get the max bandwidth of Htree.
    u_int32_t hbandwidth_max{};

    // Configuration related to the CCA Chip.
    std::string shape_arg;
    computeCellShape shape_of_compute_cells;

    // Get the memory per cc or use the default.
    u_int32_t memory_per_cc{};

    // Mesh type.
    u_int32_t mesh_type{};

    // Get the routing policy to use.
    u_int32_t routing_policy{};

    // Increments to the graph for dynamic graphs
    u_int32_t increments{};

    // To shuffle or to not shuffle the vertex ID list.
    bool shuffle_switch{};

    // Trail #. Used for taking multiple samples of the same configuations. Then can be averaged
    // out.
    u_int32_t trail_number{};

    BFSCommandLineArguments(cli::Parser& parser)
        : root_vertex(parser.get<u_int32_t>("root"))
        , verify_results(parser.get<bool>("verify"))
        , input_graph_path(parser.get<std::string>("f"))
        , graph_name(parser.get<std::string>("g"))
        , output_file_directory(parser.get<std::string>("od"))
        , hdepth(parser.get<u_int32_t>("hdepth"))
        , hx(parser.get<u_int32_t>("hx"))
        , hy(parser.get<u_int32_t>("hy"))
        , hbandwidth_max(parser.get<u_int32_t>("hb"))
        , shape_arg(parser.get<std::string>("s"))
        , shape_of_compute_cells(computeCellShape::computeCellShape_invalid)
        , memory_per_cc(parser.get<u_int32_t>("m"))
        , mesh_type(parser.get<u_int32_t>("mesh"))
        , routing_policy(parser.get<u_int32_t>("route"))
        , shuffle_switch(parser.get<bool>("shuffle"))
        , increments(parser.get<u_int32_t>("increments"))
        , trail_number(parser.get<u_int32_t>("trail"))
    {

        if (hdepth != 0) {
            if (!(hx % 2)) {
                std::cerr << "Invalid Input: hx must be odd! Provided value: " << hx << "\n";
                exit(0);
            }
            if (!(hy % 2)) {
                std::cerr << "Invalid Input: hy must be odd! Provided value: " << hy << "\n";
                exit(0);
            }
        }

        if (hdepth == 0) {
            hbandwidth_max = 0;
        }

        if (shape_arg == "square") {
            shape_of_compute_cells = computeCellShape::square;
        } else {
            std::cerr << "Error: Compute cell shape type " << shape_arg << " not supported.\n";
            exit(0);
        }
    }
};

template<typename NodeType>
inline void
verify_results(const BFSCommandLineArguments& cmd_args,
               Graph<NodeType>& input_graph,
               const CCASimulator& cca_simulator,
               const u_int32_t increment)
{
    std::cout << "\nDynamic Breadth First Search Verification: \n";

    // Open the file containing bfs results for verification.
    std::string input_graph_inc_path =
        cmd_args.input_graph_path + ".edgelist_" + std::to_string(increment);
    std::string verfication_file_path = input_graph_inc_path + ".bfs";
    std::ifstream file(verfication_file_path);

    if (!file.is_open()) {
        std::cout << "Failed to open the verification file: " << verfication_file_path << "\n";

    } else {

        std::vector<u_int32_t> control_results;
        std::string line;
        // Read the header.
        std::getline(file, line);
        // Read the root (source) of bfs that was used for results in the .bfs file.
        // Initialize it to an invalid value first.
        u_int32_t root_in_file = input_graph.total_vertices + 1;
        std::getline(file, line);
        if (!(std::istringstream(line) >> root_in_file)) {
            std::cerr << "Invalid root (source) value.\n";
            exit(0);
        }

        if (root_in_file != cmd_args.root_vertex) {
            std::cerr << "root vertex in file and root vertex used to run the program miss match. "
                         "Please use the same root in both for verification. Failed!\n";
            exit(0);
        }

        u_int32_t node_id;
        u_int32_t bfs_value;
        while (std::getline(file, line)) {

            std::istringstream iss(line);

            if (iss >> node_id >> bfs_value) {
                // When there are vertices with in-degree zero then they are not present in the .bfs
                // file. Therefore, we have to substitute its value with the undefined of
                // `max_level` for the verification to work.
                while (node_id != control_results.size()) {
                    control_results.emplace_back(undefined_level);
                }
                control_results.emplace_back(bfs_value);
            } else {
                // Parsing failed.
                std::cerr << "Error parsing line: " << line
                          << ", in file: " << verfication_file_path << std::endl;
            }
        }

        file.close();

        u_int32_t total_errors = 0;
        for (u_int32_t i = 0; i < control_results.size(); i++) {

            // Check for correctness. Print the level to a target test vertex. test_vertex

            Address const test_vertex_addr = input_graph.get_vertex_address_in_cca(i);

            auto* v_test = static_cast<BFSVertex<ghost_type_level_1>*>(
                cca_simulator.get_object(test_vertex_addr));

            // Assumes the result .bfs file is sorted.
            bool equal = control_results[i] == v_test->bfs_level;
            if (!equal) {
                std::cout << "Vertex: " << i << ", Computed BFS: " << v_test->bfs_level
                          << ", Control Value: " << control_results[i] << ", Not equal! Error\n";
                total_errors++;
            }
        }

        if (total_errors > 0) {
            std::cout << ANSI_COLOR_RED << "Total number values error: " << total_errors
                      << ", Verification Failed\n"
                      << ANSI_COLOR_RESET;
        } else {
            std::cout << ANSI_COLOR_GREEN << "All values were correct. Verification Successful.\n"
                      << ANSI_COLOR_RESET;
        }
    }
}

// Write simulation statistics to a file
template<typename NodeType>
inline void
write_results(const BFSCommandLineArguments& cmd_args,
              Graph<NodeType>& input_graph,
              CCASimulator& cca_simulator)
{

    std::string const output_file_name = "dynamic_bfs_graph_" + cmd_args.graph_name + "_v_" +
                                         std::to_string(input_graph.total_vertices) + "_e_" +
                                         std::to_string(input_graph.total_edges) + "_trail_" +
                                         std::to_string(cmd_args.trail_number) +
                                         cca_simulator.key_configurations_string();

    std::string const output_file_path = cmd_args.output_file_directory + "/" + output_file_name;
    std::cout << "\nWriting results to output file: " << output_file_path << "\n";

    std::ofstream output_file(output_file_path);
    if (!output_file) {
        std::cerr << "Error! Output file not created\n";
    }

    // Output input graph details in the header of the statistics for us to know which input graph
    // it operated on.
    output_file << "graph_file\tvertices\tedges\troot_vertex\n"
                << cmd_args.input_graph_path << "\t" << input_graph.total_vertices << "\t"
                << input_graph.total_edges << "\t" << cmd_args.root_vertex << "\n";

    // Ask the simulator to print its statistics to the `output_file`.
    cca_simulator.print_statistics(output_file);

    // Close the output file
    output_file.close();

    // Print the animation of active status of each CC per cycle.
    cca_simulator.print_animation(output_file_path);
}

#endif // CCA_BFS_HPP

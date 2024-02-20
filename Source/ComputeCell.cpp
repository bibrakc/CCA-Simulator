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

#include "ComputeCell.hpp"
#include "CCAFunctionEvents.hpp"
#include "Routing.hpp"

// For memcpy()
#include <cstring>

// TODO: Remove
#include "CCASimulator.hpp"
#include "SimpleVertex.hpp"

// Recieved an acknowledgement message back. Decreament my deficit.
auto
terminator_acknowledgement_func(ComputeCell& cc,
                                const Address addr,
                                actionType action_type_in,
                                const ActionArgumentType /*args*/) -> Closure
{
    assert(action_type_in == actionType::terminator_acknowledgement_action);

    auto* obj = static_cast<Object*>(cc.get_object(addr));

    obj->terminator.acknowledgement(cc);
    return Closure(static_cast<CCAFunctionEvent>(0), nullptr); // TODO: provide cc.null_event
}

// Get the object memory location at address addr_in
auto
ComputeCell::get_object(Address addr_in) const -> void*
{
    if (addr_in.cc_id == this->host_id) {
        assert(addr_in.type == adressType::host_address);
        return (this->host_memory.get() + addr_in.addr);
    }
    return (this->memory.get() + addr_in.addr);
}

// Return the memory used in bytes
auto
ComputeCell::get_memory_used() -> u_int32_t
{
    return this->memory_curr_ptr - this->memory_raw_ptr;
}

// In bytes
auto
ComputeCell::get_memory_curr_ptr_offset() -> u_int32_t
{
    return get_memory_used();
}

// Get memory left in bytes
auto
ComputeCell::memory_available_in_bytes() -> u_int32_t
{
    return this->memory_size_in_bytes - get_memory_used();
}

// Returns the offset in memory for this newly created object. Also copies the object from host to
// Compute Cell
auto
ComputeCell::create_object_in_memory(void* obj_in, size_t size_of_obj) -> std::optional<Address>
{
    // TODO: mutex lock here for thread-safety.
    if (this->memory_available_in_bytes() < size_of_obj) {
        return std::nullopt;
    }

    auto* cca_obj = static_cast<Object*>(obj_in);

    u_int32_t const obj_memory_addr_offset = get_memory_curr_ptr_offset();
    Address obj_addr(this->id, obj_memory_addr_offset);

    cca_obj->terminator.my_object = obj_addr;

    memcpy(this->memory_curr_ptr, obj_in, size_of_obj);
    this->memory_curr_ptr += size_of_obj;

    // Increment statistics for objects allocated on this cc.
    this->statistics.objects_allocated++;

    return obj_addr;
}

// Each compute cell has a sink cell configured such that when it has to send an operon to far flung
// compute cell it routes to the Htree network and has to sink the operon into the sink cell that is
// nearby
auto
ComputeCell::get_cc_htree_sink_cell() -> std::optional<Coordinates>
{
    if (this->hdepth == 0) {
        return std::nullopt;
    }

    u_int32_t const nearby_row = (this->hx / 2) + (this->cooridates.second / this->hx) * this->hx;
    u_int32_t const nearby_col = (this->hy / 2) + (this->cooridates.first / this->hy) * this->hy;

    // We store cooridinates from top-left therefore in a row it is (0,0), (1,0), (2,0), (3,0) ....
    // That is why the row is the second in the pair/tuple and the column is the first entry
    return Coordinates(nearby_col, nearby_row);
}

bool
ComputeCell::insert_action(const Action& action, bool priority)
{
    if (this->action_queue.push(action, priority)) {
        this->statistics.actions_pushed++;
        this->statistics.action_queue_count.increment();
        return true;
    } else {
        return false;
        // std::cerr << "action_queue full. Fatal!" << std::endl;
        // exit(0);
    }
}

// Send an Operon. Create a task that when invoked on a Compute Cell it simply puts the operon on
// the `staging_operon_from_logic`
auto
ComputeCell::send_operon(const Operon& operon_in) -> Task
{

    // Increament the deficit for termination detection if actionType !=
    // terminator_acknowledgement_action
    actionType const action_type = operon_in.second.action_type;
    if (action_type == actionType::application_action ||
        action_type == actionType::germinate_action) {
        Address const addr = operon_in.second.origin_addr;
        auto* obj = static_cast<Object*>(this->get_object(addr));

        obj->terminator.deficit++;

        /* SimpleVertex<Address>* vertex = (SimpleVertex<Address>*)this->get_object(addr);
        // print_SimpleVertex(vertex, addr);
        if (vertex->id == 0) {
            //  Signal that this object is active for termination detection
            //  origin_addr is set to be parent if deficit == 0

            std::cout << "Increamented the terminator deficit: " << obj->terminator.deficit
                      << ", << vertex->terminator.deficit: " << vertex->terminator.deficit << "\n";
        } */
    }

    return std::pair<taskType, Task_func>(
        taskType::send_operon_task_type, Task_func([this, operon_in]() {
            // Bug check
            if (this->staging_operon_from_logic != std::nullopt) {
                std::cerr << "Bug! cc: " << this->id
                          << " staging_operon_from_logic buffer is full! The program shouldn't "
                             "have come "
                             "to send_operon\n";
                exit(0);
            }

            // Debug prints
            if constexpr (debug_code) {
                std::cout << "Sending operon from cc: " << this->id
                          << " to cc: " << operon_in.first.dst_cc_id << "\n";
            }

            // Actual work of sending
            this->staging_operon_from_logic = operon_in;
        }));
}

auto
ComputeCell::construct_operon(const u_int32_t src_cc_id,
                              const u_int32_t dst_cc_id,
                              const Action& action) -> Operon
{
    return std::pair<SourceDestinationPair, Action>(SourceDestinationPair(src_cc_id, dst_cc_id),
                                                    action);
}

void
ComputeCell::diffuse(const Action& action)
{
    Operon const operon_to_send = this->construct_operon(this->id, action.obj_addr.cc_id, action);
    this->task_queue.push(this->send_operon(operon_to_send));

    // A new action was created. Increment the statistics for action.
    this->statistics.actions_created++;
    this->statistics.task_queue_count.increment();
}

void
ComputeCell::execute_action(void* function_events)
{

    // Using `void* function_events` becuase there is conflict in compiler with dependencies between
    // classes.
    // TODO: later find a graceful way and then remove this `void*`

    if (!this->action_queue.empty()) {
        Action action = this->action_queue.front();
        this->action_queue.pop();
        this->statistics.action_queue_count.decrement();

        auto* function_events_manager = static_cast<FunctionEventManager*>(function_events);

        if constexpr (debug_code) {
            if (action.obj_addr.cc_id != this->id) {
                std::cout << "Invalid addr! The vertex does not exist on this CC\n";
                return;
            }
            // When needed put the inlude header for that datastructure and print it here.
            /* SimpleVertex<Address>* vertex =
                (SimpleVertex<Address>*)this->get_object(action.obj_addr);
            print_SimpleVertex(vertex, action.obj_addr); */
        }

        if (action.action_type == actionType::application_action ||
            action.action_type == actionType::germinate_action) {

            auto* obj = static_cast<Object*>(this->get_object(action.obj_addr));

            // termination_switch is set at compile time by -D TERMINATION=true/false. It is
            // here to find overhead of termination detection method in our benchmarks.
            // Normally, the termination_switch is set to true. Or false if we are using CCA as
            // strictly an accelerator for a single user application without runtime management
            // tasks.
            if constexpr (termination_switch) {

                // When needed put the inlude header for that datastructure and print it here.
                SimpleVertex<Address>* vertex =
                    (SimpleVertex<Address>*)this->get_object(action.obj_addr);

                // print_SimpleVertex(vertex, action.obj_addr);
                if (vertex->id == 0) {
                    // Signal that this object is active for termination detection
                    // origin_addr is set to be parent if deficit == 0
                    std::cout << "before term signal in execute action, deficit: "
                              << vertex->terminator.deficit
                              << ", parent: " << vertex->terminator.parent.has_value() << "\n";
                }
                obj->terminator.signal(*this, action.origin_addr);
                if (vertex->id == 0) {
                    std::cout << "after term signal in execute action, deficit: "
                              << vertex->terminator.deficit
                              << ", parent: " << vertex->terminator.parent.has_value() << "\n";
                }
            }
            // if predicate
            Closure const predicate_resolution =
                function_events_manager->get_function_event_handler(action.predicate)(
                    *this, action.obj_addr, action.action_type, action.args);

            if (function_events_manager->is_true_event(predicate_resolution.first)) {
                this->statistics.actions_performed_work++;
                // if (predicate_resolution == 1) {

                // work
                Closure const diffuse_predicate =
                    function_events_manager->get_function_event_handler(action.work)(
                        *this, action.obj_addr, action.action_type, action.args);

                // If there are two queues in the system (configured at compile time) then we put
                // the diffuse closure into the diffuse_queue otherwise just run the diffusion here.
                if constexpr (split_queues) {

                    if (function_events_manager->is_null_event(diffuse_predicate.first)) {
                        // do nothing
                    } else if (function_events_manager->is_true_event(diffuse_predicate.first) &&
                               diffuse_predicate.second == nullptr) {
                        if (!this->diffuse_queue.push(action)) {
                            std::cerr << "diffuse_queue full. Can not push. Fatal." << std::endl;
                            exit(0);
                        }
                    } else { // diffuse predicate is lazy evaluated.

                        // Get new arguments from `diffuse_predicate.second`.
                        action.args = diffuse_predicate.second;

                        if (!this->diffuse_queue.push(action)) {
                            std::cerr << "diffuse_queue full. Can not push. Fatal." << std::endl;
                            exit(0);
                        }
                    }

                } else { // Only single queue i.e. action_queue

                    if (function_events_manager->is_null_event(diffuse_predicate.first)) {
                        // do nothing
                    } else {

                        if (diffuse_predicate.second != nullptr) {
                            action.args = diffuse_predicate.second;
                        }
                        Closure const diffuse_predicate_resolution =
                            function_events_manager->get_function_event_handler(
                                diffuse_predicate.first)(
                                *this, action.obj_addr, action.action_type, action.args);

                        // diffuse
                        if (function_events_manager->is_true_event(
                                diffuse_predicate_resolution.first)) {
                            // if (diffuse_predicate_resolution == 1) {
                            function_events_manager->get_function_event_handler(action.diffuse)(
                                *this, action.obj_addr, action.action_type, action.args);
                        }
                    }
                }
            } else {
                // This action is discarded/subsumed
                this->statistics.actions_false_on_predicate++;
            }
            /* if constexpr (termination_switch) {
                obj->terminator.unsignal(*this);
            } */
        } else if (action.action_type == actionType::terminator_acknowledgement_action) {

            function_events_manager->get_acknowledgement_event_handler()(
                *this, action.obj_addr, action.action_type, action.args); // nullptr

            this->statistics.actions_acknowledgement_invoked++;
        } else {
            std::cerr << "Bug! Unsupported action type. It shouldn't be here\n";
            exit(0);
        }
        // Increament the counter for actions that were invoked
        this->statistics.actions_invoked++;
        return;
    }
    std::cerr << "Bug! Cannot execute action as the action_queue is empty! It shouldn't be here\n";
    exit(0);
}

void
ComputeCell::execute_diffusion_phase(void* function_events)
{

    // Using `void* function_events` becuase there is conflict in compiler with dependencies between
    // classes.
    // TODO: later find a graceful way and then remove this `void*`

    if (!this->diffuse_queue.empty()) {

        Action const action = this->diffuse_queue.front();
        this->diffuse_queue.pop();

        // TODO: Fix this stats!
        // this->statistics.action_queue_count.decrement();

        auto* function_events_manager = static_cast<FunctionEventManager*>(function_events);

        if constexpr (debug_code) {
            if (action.obj_addr.cc_id != this->id) {
                std::cout << "Invalid addr! The vertex does not exist on this CC\n";
                return;
            }
            // When needed put the inlude header for that datastructure and print it here.
            /* SimpleVertex<Address>* vertex =
                (SimpleVertex<Address>*)this->get_object(action.obj_addr);
            print_SimpleVertex(vertex, action.obj_addr); */
        }

        if (action.action_type == actionType::application_action ||
            action.action_type == actionType::germinate_action) {

            // if predicate
            Closure const predicate_resolution =
                function_events_manager->get_function_event_handler(action.diffuse_predicate)(
                    *this, action.obj_addr, action.action_type, action.args);
            // predicate_resolution = 1;
            if (function_events_manager->is_true_event(predicate_resolution.first)) {
                // if (predicate_resolution == 1) {
                //  diffuse
                //  std::cout << "running execute_diffusion_phase\n";
                function_events_manager->get_function_event_handler(action.diffuse)(
                    *this, action.obj_addr, action.action_type, action.args);

            } else {
                // This diffusion is discarded/subsumed.
                // TODO: Fix this stat.
                // this->statistics.actions_false_on_predicate++;
            }
            // Finally this object becomes inactive.
            if constexpr (termination_switch) {
                auto* obj = static_cast<Object*>(this->get_object(action.obj_addr));
                obj->terminator.unsignal(*this);
            }
        } else {
            std::cerr << "Bug! Unsupported action type. It shouldn't be here\n";
            exit(0);
        }
        // Increament the counter for actions that were invoked
        // this->statistics.actions_invoked++;

        return;
    }
    std::cerr << "Bug! Cannot execute diffuse phase as the diffuse_queue is empty! It shouldn't be "
                 "here\n";
    exit(0);
}

void
ComputeCell::filter_diffusion(void* function_events)
{

    // Using `void* function_events` becuase there is conflict in compiler with dependencies between
    // classes.
    // TODO: later find a graceful way and then remove this `void*`

    if (!this->diffuse_queue.empty()) {

        Action const action = this->diffuse_queue.front();
        this->diffuse_queue.pop();

        // TODO: Fix this stats!
        // this->statistics.action_queue_count.decrement();

        auto* function_events_manager = static_cast<FunctionEventManager*>(function_events);

        if constexpr (debug_code) {
            if (action.obj_addr.cc_id != this->id) {
                std::cout << "Invalid addr! The vertex does not exist on this CC\n";
                return;
            }
            // When needed put the inlude header for that datastructure and print it here.
            /* SimpleVertex<Address>* vertex =
                (SimpleVertex<Address>*)this->get_object(action.obj_addr);
            print_SimpleVertex(vertex, action.obj_addr); */
        }

        if (action.action_type == actionType::application_action ||
            action.action_type == actionType::germinate_action) {

            // if predicate
            Closure predicate_resolution = function_events_manager->get_function_event_handler(
                action.diffuse_predicate)(*this, action.obj_addr, action.action_type, action.args);
            // predicate_resolution = 1;
            if (function_events_manager->is_true_event(predicate_resolution.first)) {
                // if (predicate_resolution == 1) {
                //  put it back into the queue
                if (!this->diffuse_queue.push(action)) {
                    std::cerr << "diffuse_queue full. How is this possible? Bug!" << std::endl;
                    exit(0);
                }

            } else {
                // This diffusion is discarded/subsumed.
                // TODO: Fix this stat.
                // this->statistics.actions_false_on_predicate++;
            }
            // Finally this object becomes inactive.
            if constexpr (termination_switch) {
                auto* obj = static_cast<Object*>(this->get_object(action.obj_addr));
                obj->terminator.unsignal(*this);
            }
        } else {
            std::cerr << "Bug! Unsupported action type. It shouldn't be here\n";
            exit(0);
        }
        // Increament the counter for actions that were invoked
        // this->statistics.actions_invoked++;

        return;
    }
    std::cerr << "Bug! Cannot execute diffuse phase as the diffuse_queue is empty! It shouldn't be "
                 "here\n";
    exit(0);
}

void
ComputeCell::prepare_a_cycle(std::vector<std::shared_ptr<Cell>>& CCA_chip)
{

    if (!this->is_compute_cell_active()) {
        return;
    }

    // Move the operon from previous cycle (that was not able to be sent due to congestion) to
    // the send link of the network
    this->prepare_a_communication_cycle(CCA_chip);

    // Used for fairness. So that the channels don't get priority over others based on iterator
    u_int32_t recv_channel_index = this->current_recv_channel_to_start_a_cycle;

    // Move the operon from previous cycle recv channel to thier destination: action queue or
    // send channel of a neighbor
    for (u_int32_t i = 0; i < this->recv_channel_per_neighbor.size(); i++) {
        // Virtual Channel
        for (int j = this->recv_channel_per_neighbor[recv_channel_index].size() - 1; j >= 0; j--) {
            // for (u_int32_t j = 0; j < this->recv_channel_per_neighbor[i].size(); j++) {

            // Buffers in the recv channel
            if (this->recv_channel_per_neighbor[recv_channel_index][j].size()) {

                // If this is greater then it is a bug
                assert(j <= static_cast<int>(this->number_of_virtual_channels));

                std::vector<Operon> recv_operons;
                while (this->recv_channel_per_neighbor[recv_channel_index][j].size()) {
                    recv_operons.push_back(
                        this->recv_channel_per_neighbor[recv_channel_index][j].front());
                    this->recv_channel_per_neighbor[recv_channel_index][j].pop();
                }

                std::vector<Operon> left_over_operons;
                bool operon_was_inserted_or_sent = false;
                for (Operon operon : recv_operons) {

                    // This means that in this cycle an operon from the buffer (in recv_operons) was
                    // either inserted in this CC or sent to the neighbor CC. Therefore, just put
                    // this operon now in the left overs to be put back into the buffer.
                    if (operon_was_inserted_or_sent) {
                        left_over_operons.push_back(operon);
                        continue;
                    }

                    u_int32_t const dst_cc_id = operon.first.dst_cc_id;
                    // Bug check: Make sure the destination is not a Sink Cell
                    assert(CCA_chip[dst_cc_id]->type != CellType::sink_cell);

                    // Check if this operon is destined for this compute cell
                    if (this->id == dst_cc_id) {
                        if (this->insert_action(operon.second, false)) {

                            operon_was_inserted_or_sent = true;
                        } else {
                            // action_queue is full. Therefore, just return without doing anything.
                            // Contended.
                            // return;
                        }

                    } else {
                        // if (operon.first.src_cc_id == 43 && operon.first.dst_cc_id == 125) {
                        /*  std::cout << "\n";

                         std::cout
                             << "inside prepare_a_communication_cycle for: " << this->cooridates
                             << ", with id: " << this->id << "\n";
                         std::cout << "operon dst: "
                                   << this->cc_id_to_cooridinate(
                                          operon.first.dst_cc_id, this->shape, this->dim_y)
                                   << ", with id: " << operon.first.dst_cc_id << "\n";
                         std::cout << "operon src: "
                                   << this->cc_id_to_cooridinate(
                                          operon.first.src_cc_id, this->shape, this->dim_y)
                                   << ", with id: " << operon.first.src_cc_id << "\n"; */
                        //}

                        // Get the route using Routing 0
                        std::optional<u_int32_t> routing_cell_id =
                            Routing::get_next_move<ComputeCell>(
                                CCA_chip, operon, this->id, this->mesh_routing_policy);

                        std::vector<u_int32_t> const channels_to_send =
                            this->get_route_towards_cc_id(operon.first.src_cc_id,
                                                          routing_cell_id.value());

                        // if (operon.first.src_cc_id == 43 && operon.first.dst_cc_id == 125) {
                        /*  std::cout << "\n";
                         std::cout << "channels_to_send = " << channels_to_send[0] << "\n"; */
                        //}

                        u_int32_t const cc_column = this->cooridates.first;
                        u_int32_t const cc_row = this->cooridates.second;

                        bool const is_cell_top_border = cc_row == 0;
                        bool const is_cell_bottom_border = cc_row == this->dim_x - 1;

                        bool const is_cell_left_border = cc_column == 0;
                        bool const is_cell_right_border = cc_column == this->dim_y - 1;

                        for (auto channel_to_send : channels_to_send) {

                            u_int32_t current_virtual_channel = j;

                            bool stay_on_same_virtual_channel =
                                (recv_channel_index % 2) == (channel_to_send % 2);

                            bool is_warped = false;
                            if ((is_cell_top_border && (channel_to_send == 1)) ||
                                (is_cell_bottom_border && (channel_to_send == 3)) ||
                                (is_cell_left_border && (channel_to_send == 0)) ||
                                (is_cell_right_border && (channel_to_send == 2))) {
                                is_warped = true;
                            }

                            if (is_warped && stay_on_same_virtual_channel) {
                                stay_on_same_virtual_channel = !stay_on_same_virtual_channel;
                            }

                            u_int32_t virtual_channel_to_use = current_virtual_channel;
                            if (!stay_on_same_virtual_channel) {
                                virtual_channel_to_use = (current_virtual_channel + 1) %
                                                         this->number_of_virtual_channels;
                            }

                            if (this->send_channel_per_neighbor[channel_to_send]
                                                               [virtual_channel_to_use]
                                                                   .push(operon)) {

                                // Break out of the for loop. Discard other paths.
                                operon_was_inserted_or_sent = true;
                                break;
                            }
                        }
                    }
                    if (!operon_was_inserted_or_sent) {
                        left_over_operons.push_back(operon);
                    }
                }

                for (Operon const& operon : left_over_operons) {
                    if (!this->recv_channel_per_neighbor[recv_channel_index][j].push(operon)) {
                        std::cerr << "ComputeCell: recv_channel_per_neighbor can not push. Fatal. "
                                     "Perhaps a bug"
                                  << std::endl;
                    }
                }
            }
        }
        recv_channel_index = (recv_channel_index + 1) % this->number_of_neighbors;
    }
    // Update the index of the starting channel for the next cycle.
    this->current_recv_channel_to_start_a_cycle =
        (this->current_recv_channel_to_start_a_cycle + 1) % this->number_of_neighbors;
}

void
ComputeCell::run_a_computation_cycle(std::vector<std::shared_ptr<Cell>>& CCA_chip,
                                     void* function_events)
{

    // TODO: remove this
    if (!this->is_compute_cell_active()) {
        return;
    }

    // A single compute cell can perform work and communication in parallel in a single cycle
    // This function performs work if there is any.

    // Apply the network operations from the previous cycle and prepare this cycle for
    // computation and communication
    this->prepare_a_cycle(CCA_chip);

    // Check whether throttle needs to be applied.
    bool was_recently_congested = false;
    if constexpr (throttling_switch) {
        if (this->last_congested_cycle) {
            was_recently_congested = (this->current_cycle - this->last_congested_cycle.value()) <=
                                     curently_congested_threshold;
        }
    }

    if (!this->task_queue.empty()) {

        // If staging buffer is full then no need to perform diffusion task (`send_operon`) since it
        // will stall. Same is true when it is congested and we are throttling.
        // We can use this cycle to potentially filter out action_queue or diffuse_queue.
        if (this->staging_operon_from_logic || was_recently_congested) {

            if constexpr (split_queues) {
                // Filter action_queue or diffuse_queue by executing them.
                // When an `Action` from action_queue is executed it is checked for predicate. IF
                // predicate is true then work is performed and along with it a new diffuse Action
                // is created and put into the diffuse_queue. This process can be "seen" as
                // filtering of the action_queue. Similarly, the diffuse_queue can be filtered but
                // it is naunced as if its predicate is true then it needs to put `send_operon`
                // tasks into the task queue that is already occupied. Remember: the task queue is a
                // way of simulating the `for` loop in diffusion so it is actually a thread and not
                // a queue, and is therefore considered "context switched". So we cant have two
                // threads at the same time. Logic for arbitration between action and diffuse
                // queues.

                bool const both_queues_non_empty =
                    !this->action_queue.empty() && !this->diffuse_queue.empty();

                bool const diffuse_queue_is_getting_full =
                    this->diffuse_queue.is_percent_full(90.0);
                bool const action_queue_near_full = this->diffuse_queue.is_percent_full(95.0);

                if (both_queues_non_empty) {
                    if (action_queue_near_full && this->diffuse_queue.has_room()) {
                        this->execute_action(function_events);
                    } else if (diffuse_queue_is_getting_full) { //&& this->prefer_diffuse_queue) {
                        this->filter_diffusion(function_events);
                        // this->prefer_diffuse_queue = false;
                    } else {
                        this->execute_action(function_events);
                        // this->prefer_diffuse_queue = true;
                    }
                } else {
                    // Only one of the queues is non-empty or both are empty.
                    if (!this->diffuse_queue.empty()) {
                        this->filter_diffusion(function_events);
                    } else if (!this->action_queue.empty()) {
                        this->execute_action(function_events);
                    }
                }
            }
        } else {
            // Get a task from the task_queue
            Task const current_task = this->task_queue.front();

            // Check if the staging buffer is not full and the task type is send operon
            // In that case stall and don't do anything. Because the task can't send operon due to
            // staging buffer being full.
            if (this->staging_operon_from_logic &&
                (current_task.first == taskType::send_operon_task_type)) {
                this->staging_logic_contention_count.increment();
            } else {

                if (!was_recently_congested) {

                    // Remove the task from the queue and execute it.
                    this->task_queue.pop();

                    this->statistics.task_queue_count.decrement();
                    // Execute the task
                    current_task.second();
                    this->staging_logic_contention_count.reset();
                }
            }
        }
    } else {

        if constexpr (split_queues) {
            // Execute action_queue or diffuse_queue.
            // Logic for arbitration between action and diffuse queues.
            bool const both_queues_non_empty =
                !this->action_queue.empty() && !this->diffuse_queue.empty();

            if (both_queues_non_empty) {
                // Both queues are non-empty, decide which one to use.
                bool const diffuse_queue_is_getting_full =
                    this->diffuse_queue.is_percent_full(90.0);
                bool const action_queue_near_full = this->diffuse_queue.is_percent_full(90.0);

                if (action_queue_near_full && this->diffuse_queue.has_room()) {
                    this->execute_action(function_events);
                } else if (diffuse_queue_is_getting_full) {
                    // Execute an diffusion if the diffuse_queue is not empty.
                    this->execute_diffusion_phase(function_events);
                } else {
                    // Execute an action if the action_queue is not empty.
                    this->execute_action(function_events);
                }
            } else {
                // Only one of the queues is non-empty or both are empty.
                if (!this->action_queue.empty()) {
                    this->execute_action(function_events);
                } else if (!this->diffuse_queue.empty()) {
                    this->execute_diffusion_phase(function_events);
                }
            }
        } else { // When there is only one action_queue.
            if (!this->action_queue.empty()) {
                this->execute_action(function_events);
            }
        }
    }
}

// This acts as synchronization and needs to be called before the actual communication cycle so
// as to not cause race conditions on the communicaton buffer. It also applies to the
// prepare_a_cycle since we want to move any staging operon from logic into the communication
// network so as to not cause it to wait on network not more than one cycle
void
ComputeCell::prepare_a_communication_cycle(std::vector<std::shared_ptr<Cell>>& CCA_chip)
{

    if (this->staging_operon_from_logic) {

        Operon operon_ = this->staging_operon_from_logic.value();
        u_int32_t const dst_cc_id = operon_.first.dst_cc_id;

        // Bug check: Make sure the destination is not a Sink Cell
        assert(CCA_chip[dst_cc_id]->type != CellType::sink_cell);

        // Check if this operon is destined for this compute cell
        // Meaning both src and dst vertices are on the same compute cell?
        if (this->id == dst_cc_id) {
            if (!this->insert_action(operon_.second, true)) {
                // action_queue is full. Therefore, just return without doing anything. Contended.
                return;
            }
            // Flush the channel buffer
            this->staging_operon_from_logic = std::nullopt;
        } else {

            // Get the route using Routing 0
            std::optional<u_int32_t> routing_cell_id = Routing::get_next_move<ComputeCell>(
                CCA_chip, operon_, this->id, this->mesh_routing_policy);

            // Based on the routing algorithm and the shape of CCs it will return which neighbor
            // to pass this operon to. The returned value is the index [0...number of neighbors)
            // coresponding clockwise the channel id of the physical shape.
            std::vector<u_int32_t> const channels_to_send =
                this->get_route_towards_cc_id(operon_.first.src_cc_id, routing_cell_id.value());

            // Always use the default virtual channel 0 as this is the begining of the journey for
            // this operon. It shouldn't matter for deadlocks.
            u_int32_t virtual_channel_to_use = 0;

            for (auto channel_to_send : channels_to_send) {
                if (this->send_channel_per_neighbor[channel_to_send][virtual_channel_to_use].push(
                        this->staging_operon_from_logic.value())) {

                    // Empty the staging buffer
                    this->staging_operon_from_logic = std::nullopt;

                    // Break out of the for loop. Discard other paths.
                    break;
                }
            }
        }
    }
}

void
ComputeCell::run_a_communication_cycle(std::vector<std::shared_ptr<Cell>>& CCA_chip)
{
    if (!this->is_compute_cell_active()) {
        return;
    }

    // For shape square
    if (this->shape == computeCellShape::square) {
        int const receiving_direction[4] = { 2, 3, 0, 1 };

        for (u_int32_t i = 0; i < this->send_channel_per_neighbor.size(); i++) {

            for (u_int32_t virtual_channel_index = 0;
                 virtual_channel_index < this->number_of_virtual_channels;
                 virtual_channel_index++) {

                if (this->send_channel_per_neighbor[i][virtual_channel_index].size()) {

                    // TODO: NOTE: the size of each channel is fixed to be `1`, therfore this
                    // leftover etc is of no use for now
                    std::vector<Operon> send_operons;
                    while (this->send_channel_per_neighbor[i][virtual_channel_index].size()) {
                        send_operons.push_back(
                            this->send_channel_per_neighbor[i][virtual_channel_index].front());
                        this->send_channel_per_neighbor[i][virtual_channel_index].pop();
                    }

                    std::vector<Operon> left_over_operons;
                    for (Operon const& operon : send_operons) {
                        u_int32_t const dst_cc_id = operon.first.dst_cc_id;

                        // Check if this operon is destined for this compute cell
                        assert(this->id != dst_cc_id);
                        assert(this->neighbor_compute_cells[i] != std::nullopt);

                        u_int32_t const neighbor_id_ =
                            this->neighbor_compute_cells[i].value().first;

                        if (!CCA_chip[neighbor_id_]->recv_operon(
                                operon, receiving_direction[i], virtual_channel_index)) {

                            this->send_channel_per_neighbor_contention_count[i].increment();

                            /* std::cout
                              << "\tCC : " << this->cooridates << " Not able to send to neighbor: "
                              << this->neighbor_compute_cells[i].value().second << " i = " << i
                              << ", contention_count: max:"
                              <<
                              this->send_channel_per_neighbor_contention_count[i].get_max_count()
                              << ", contention_count: current : "
                              << this->send_channel_per_neighbor_contention_count[i].get_count()
                              << "\n";  */

                            left_over_operons.push_back(operon);
                        } else {
                            this->send_channel_per_neighbor_contention_count[i].reset();
                            this->statistics.operons_moved++;
                        }

                        for (Operon const& operon : left_over_operons) {
                            if (!this->send_channel_per_neighbor[i][virtual_channel_index].push(
                                    operon)) {
                                std::cerr << "ComputeCell: send_channel_per_neighbor can not push. "
                                             "Fatal. Perhaps a bug."
                                          << std::endl;
                            }
                        }
                    }
                }
            }
        }
    }
}

// Checks if the compute cell is active or not
auto
ComputeCell::is_compute_cell_active() -> u_int32_t
{

    bool send_channels = false;
    bool recv_channels = false;
    for (u_int32_t i = 0; i < this->number_of_neighbors; i++) {
        for (u_int32_t virtual_channel_index = 0;
             virtual_channel_index < this->number_of_virtual_channels;
             virtual_channel_index++) {

            if (this->send_channel_per_neighbor[i][virtual_channel_index].size()) {
                send_channels = true;
                break;
            }

            if (this->recv_channel_per_neighbor[i][virtual_channel_index].size()) {
                recv_channels = true;
                break;
            }
        }
    }
    bool const compute_active =
        !this->action_queue.empty() || !this->diffuse_queue.empty() || !this->task_queue.empty();

    bool const communication_active =
        (this->staging_operon_from_logic || send_channels || recv_channels);

    auto [is_congested, congestion_level_addition] = this->is_congested();

    /*
    0: Inactive
    1: Communication
    2: Computation
    3: Computation and Communication (Both)
    4: Congestion Level 1 Communication
    5: Congestion Level 2 Communication
    6: Congestion Level 3 Communication
    7: Congestion Level 4 Communication
    8: Congestion Level 1 Both
    9: Congestion Level 2 Both
    10: Congestion Level 3 Both
    11: Congestion Level 4 Both
    */
    constexpr u_int32_t inactive_status = 0;
    constexpr u_int32_t communication_status = 1;
    constexpr u_int32_t computation_status = 2;
    constexpr u_int32_t both_status = 3;
    constexpr u_int32_t communication_congested_status = 4;
    constexpr u_int32_t both_congested_status = 8;

    if (compute_active && communication_active) {
        // Both compute and communicate active
        if (is_congested) {
            return (both_congested_status + congestion_level_addition);
        }
        return both_status;

    } else if (compute_active) {
        // Only compute active
        return computation_status;
    } else if (communication_active) {
        // Only communication active
        if (is_congested) {
            return (communication_congested_status + congestion_level_addition);
        }
        return communication_status;
    }
    // Inactive
    return inactive_status;
}

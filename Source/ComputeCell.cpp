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

// Recieved an acknowledgement message back. Decreament my deficit.
auto
terminator_acknowledgement_func(ComputeCell& cc,
                                const Address& addr,
                                actionType action_type_in,
                                const ActionArgumentType& /*args*/) -> int
{
    assert(action_type_in == actionType::terminator_acknowledgement_action);

    auto* obj = static_cast<Object*>(cc.get_object(addr));

    obj->terminator.acknowledgement(cc);
    return 0;
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
    if (this->memory_available_in_bytes() < size_of_obj) {
        return std::nullopt;
    }

    auto* cca_obj = static_cast<Object*>(obj_in);

    u_int32_t const obj_memory_addr_offset = get_memory_curr_ptr_offset();
    Address obj_addr(this->id, obj_memory_addr_offset);

    cca_obj->terminator.my_object = obj_addr;

    memcpy(this->memory_curr_ptr, obj_in, size_of_obj);
    this->memory_curr_ptr += size_of_obj;

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

void
ComputeCell::insert_action(const Action& action)
{
    this->action_queue.push(action);
    this->statistics.actions_pushed++;
}

// Send an Operon. Create a task that when invoked on a Compute Cell it simply puts the operon on
// the `staging_operon_from_logic`
auto
ComputeCell::send_operon(const Operon& operon_in) -> Task
{

    // Increament the deficit for termination detection if actionType !=
    // terminator_acknowledgement_action
    actionType const action_type = operon_in.second.action_type;
    if (action_type == actionType::application_action) {
        Address const addr = operon_in.second.origin_addr;
        auto* obj = static_cast<Object*>(this->get_object(addr));

        obj->terminator.deficit++;
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
}

void
ComputeCell::execute_action(void* function_events)
{

    // Using `void* function_events` becuase there is conflict in compiler with dependencies between
    // classes.
    // TODO: later find a graceful way and then remove this `void*`

    if (!this->action_queue.empty()) {
        Action const action = this->action_queue.front();
        this->action_queue.pop();

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

            // Signal that this object is active for termination detection
            // origin_addr is set to be parent if deficit == 0
            obj->terminator.signal(*this, action.origin_addr);

            // if predicate
            int const predicate_resolution = function_events_manager->get_function_event_handler(
                action.predicate)(*this, action.obj_addr, action.action_type, action.args);

            if (predicate_resolution == 1) {

                // work
                function_events_manager->get_function_event_handler(action.work)(
                    *this, action.obj_addr, action.action_type, action.args);
                this->statistics.actions_performed_work++;

                // diffuse
                function_events_manager->get_function_event_handler(action.diffuse)(
                    *this, action.obj_addr, action.action_type, action.args);
            } else {
                // This action is discarded/subsumed
                this->statistics.actions_false_on_predicate++;
            }
            obj->terminator.unsignal(*this);
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
ComputeCell::prepare_a_cycle(std::vector<std::shared_ptr<Cell>>& CCA_chip)
{

    if (!this->is_compute_cell_active()) {
        return;
    }

    // Move the operon from previous cycle (that was not able to be sent due to congestion) to
    // the send link of the network
    this->prepare_a_communication_cycle(CCA_chip);

    // Used for fairness. So that the channels don't get priority over others based on iterator
    // u_int32_t recv_channel_index = 0; // this->current_recv_channel_to_start_a_cycle;

    // Move the operon from previous cycle recv channel to thier destination: action queue or
    // send channel of a neighbor
    for (u_int32_t i = 0; i < this->recv_channel_per_neighbor.size(); i++) {
        for (int j = this->recv_channel_per_neighbor[i].size() - 1; j >= 0; j--) {
            // for (u_int32_t j = 0; j < this->recv_channel_per_neighbor[i].size(); j++) {

            if (this->recv_channel_per_neighbor[i][j].size()) {

                // If this is greater them it is a bug
                assert(j <= static_cast<int>(this->distance_class_length));

                std::vector<Operon> recv_operons;
                while (this->recv_channel_per_neighbor[i][j].size()) {
                    recv_operons.push_back(this->recv_channel_per_neighbor[i][j].front());
                    this->recv_channel_per_neighbor[i][j].pop();
                }

                std::vector<Operon> left_over_operons;
                for (Operon operon : recv_operons) {

                    u_int32_t const dst_cc_id = operon.first.dst_cc_id;
                    // Bug check: Make sure the destination is not a Sink Cell
                    assert(CCA_chip[dst_cc_id]->type != CellType::sink_cell);

                    // Check if this operon is destined for this compute cell
                    if (this->id == dst_cc_id) {
                        this->insert_action(operon.second);
                    } else {

                        // Get the route using Routing 0
                        std::optional<u_int32_t> routing_cell_id =
                            Routing::get_next_move<ComputeCell>(
                                CCA_chip, operon, this->id, this->mesh_routing_policy);

                        std::vector<u_int32_t> const channels_to_send =
                            this->get_route_towards_cc_id(operon.first.src_cc_id,
                                                          routing_cell_id.value());

                        bool pushed = false;
                        for (auto channel_to_send : channels_to_send) {
                            if (this->send_channel_per_neighbor[channel_to_send].push(operon)) {

                                // Set the distance class for this operon
                                this->send_channel_per_neighbor_current_distance_class
                                    [channel_to_send] = 0; // j + 1;

                                // Break out of the for loop. Discard other paths.
                                pushed = true;
                                break;
                            }
                        }

                        if (!pushed) {
                            left_over_operons.push_back(operon);
                        }
                    }
                }

                for (Operon const& operon : left_over_operons) {
                    this->recv_channel_per_neighbor[i][j].push(operon);
                }

                //  recv_channel_index = (recv_channel_index + 1) % this->number_of_neighbors;
            }
        }
        // Update the index of the starting channel for the next cycle
        // TODO: Make use of this
        /*    this->current_recv_channel_to_start_a_cycle =
               (this->current_recv_channel_to_start_a_cycle + 1) % this->number_of_neighbors; */
    }
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
    // This function does both. First it performs work if there is any. Then it performs
    // communication

    // Apply the network operations from the previous cycle and prepare this cycle for
    // computation and communication
    this->prepare_a_cycle(CCA_chip);

    // Perform execution of work. Exectute a task if the task_queue is not empty
    if (!this->task_queue.empty()) {
        //  Get a task from the task_queue
        Task const current_task = this->task_queue.front();

        // Check if the staging buffer is not full and the task type is send operon
        // In that case stall and don't do anything. Because the task can't send operon
        if (this->staging_operon_from_logic &&
            (current_task.first == taskType::send_operon_task_type)) {
        } else {
            // Apply throttle if enabled.
            bool was_cerently_congested = false;
            if constexpr (throttling_switch) {
                if (this->last_congested_cycle) {
                    was_cerently_congested =
                        (this->current_cycle - this->last_congested_cycle.value()) <
                        curently_congested_threshold;
                }
            }

            if (!was_cerently_congested) {
                // Remove the task from the queue and execute it.
                this->task_queue.pop();
                // Execute the task
                current_task.second();
            }
        }

    } else if (!this->action_queue.empty()) {
        // Else execute an action if the action_queue is not empty
        this->execute_action(function_events);
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
            this->insert_action(operon_.second);
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

            for (auto channel_to_send : channels_to_send) {
                if (this->send_channel_per_neighbor[channel_to_send].push(
                        this->staging_operon_from_logic.value())) {

                    // Set to distance class 0 since this operon originates from this CC
                    this->send_channel_per_neighbor_current_distance_class[channel_to_send] = 0;

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

            if (this->send_channel_per_neighbor[i].size()) {

                // TODO: NOTE: the size of each channel is fixed to be `1`, therfore this leftover
                // etc is of no use for now
                std::vector<Operon> send_operons;
                while (this->send_channel_per_neighbor[i].size()) {
                    send_operons.push_back(this->send_channel_per_neighbor[i].front());
                    this->send_channel_per_neighbor[i].pop();
                }

                std::vector<Operon> left_over_operons;
                for (Operon const& operon : send_operons) {
                    u_int32_t const dst_cc_id = operon.first.dst_cc_id;

                    // Check if this operon is destined for this compute cell
                    assert(this->id != dst_cc_id);
                    assert(this->neighbor_compute_cells[i] != std::nullopt);

                    u_int32_t const neighbor_id_ = this->neighbor_compute_cells[i].value().first;

                    if (!CCA_chip[neighbor_id_]->recv_operon(
                            operon,
                            receiving_direction[i],
                            this->send_channel_per_neighbor_current_distance_class[i])) {

                        this->send_channel_per_neighbor_contention_count[i].increment();

                        /*  std::cout
                             << "\tCC : " << this->cooridates << " Not able to send to neighbor: "
                             << this->neighbor_compute_cells[i].value().second << " i = " << i
                             << ", contention_count: max:"
                             << this->send_channel_per_neighbor_contention_count[i].get_max_count()
                             << ", contention_count: current : "
                             << this->send_channel_per_neighbor_contention_count[i].get_count()
                             << "\n"; */

                        left_over_operons.push_back(operon);
                    } else {
                        this->send_channel_per_neighbor_contention_count[i].reset();
                    }

                    for (Operon const& operon : left_over_operons) {
                        this->send_channel_per_neighbor[i].push(operon);
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
        if (this->send_channel_per_neighbor[i].size()) {
            send_channels = true;
            break;
        }
        for (u_int32_t j = 0; j < this->recv_channel_per_neighbor[i].size(); j++) {
            if (this->recv_channel_per_neighbor[i][j].size()) {
                recv_channels = true;
                break;
            }
        }
    }
    bool const compute_active = !this->action_queue.empty() || !this->task_queue.empty();
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

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
#include "Cell.hpp"
#include "Function.hpp"
#include "Object.hpp"

#if debug_code == true
// TODO: Later make it generic to be data structure agnostic
#include "SimpleVertex.hpp"
#endif

// For memcpy()
#include <cstring>

#include <cassert>

// Recieved an acknowledgement message back. Decreament my deficit.
int
terminator_acknowledgement_func(ComputeCell& cc,
                                const Address& addr,
                                int nargs,
                                const std::shared_ptr<int[]>& args)
{
    Object* obj = static_cast<Object*>(cc.get_object(addr));

    obj->terminator.acknowledgement(cc);
    return 0;
}

// Get the object memory location at address addr_in
void*
ComputeCell::get_object(Address addr_in) const
{
    if (addr_in.cc_id == this->host_id) {
        assert(addr_in.type == adressType::host_address);
        return (this->host_memory.get() + addr_in.addr);
    }
    return (this->memory.get() + addr_in.addr);
}

// Return the memory used in bytes
u_int32_t
ComputeCell::get_memory_used()
{
    return this->memory_curr_ptr - this->memory_raw_ptr;
}

// In bytes
u_int32_t
ComputeCell::get_memory_curr_ptr_offset()
{
    return get_memory_used();
}

// Get memory left in bytes
u_int32_t
ComputeCell::memory_available_in_bytes()
{
    return this->memory_size_in_bytes - get_memory_used();
}

// Returns the offset in memory for this newly created object. Also copies the object from host to
// Compute Cell
std::optional<Address>
ComputeCell::create_object_in_memory(void* obj_in, size_t size_of_obj)
{
    if (this->memory_available_in_bytes() < size_of_obj) {
        return std::nullopt;
    }

    Object* cca_obj = static_cast<Object*>(obj_in);

    u_int32_t obj_memory_addr_offset = get_memory_curr_ptr_offset();
    Address obj_addr(this->id, obj_memory_addr_offset);

    cca_obj->terminator.my_object = obj_addr;

    memcpy(this->memory_curr_ptr, obj_in, size_of_obj);
    this->memory_curr_ptr += size_of_obj;

    return obj_addr;
}

// Each compute cell has a sink cell configured such that when it has to send an operon to far flung
// compute cell it routes to the Htree network and has to sink the operon into the sink cell that is
// nearby
std::optional<Coordinates>
ComputeCell::get_cc_htree_sink_cell()
{
    if (this->hdepth == 0) {
        return std::nullopt;
    }

    u_int32_t nearby_row = (this->hx / 2) + (this->cooridates.second / this->hx) * this->hx;
    u_int32_t nearby_col = (this->hy / 2) + (this->cooridates.first / this->hy) * this->hy;

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
Task
ComputeCell::send_operon(Operon operon_in)
{

    // Increament the deficit for termination detection if actionType !=
    // terminator_acknowledgement_action
    actionType action_type = operon_in.second.action_type;
    if (action_type == actionType::application_action) {
        Address addr = operon_in.second.origin_addr;
        Object* obj = static_cast<Object*>(this->get_object(addr));

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

Operon
ComputeCell::construct_operon(const u_int32_t src_cc_id,
                              const u_int32_t dst_cc_id,
                              const Action& action)
{
    return std::pair<SourceDestinationPair, Action>(SourceDestinationPair(src_cc_id, dst_cc_id),
                                                    action);
}

void
ComputeCell::execute_action(void* function_events)
{

    // Using `void*` becuase there is conflict in compiler with dependencies between classes
    // TODO: later find a graceful way and then remove this `void*`

    if (!this->action_queue.empty()) {
        Action action = this->action_queue.front();
        this->action_queue.pop();

        FunctionEventManager* function_events_manager =
            static_cast<FunctionEventManager*>(function_events);

#if debug_code == true
        if (action.obj_addr.cc_id != this->id) {
            std::cout << "Invalid addr! The vertex does not exist on this CC\n";
            return;
        }
        SimpleVertex<Address>* vertex = (SimpleVertex<Address>*)this->get_object(action.obj_addr);
        print_SimpleVertex(vertex, action.obj_addr);
#endif
        if (action.action_type == actionType::application_action) {

            Object* obj = static_cast<Object*>(this->get_object(action.obj_addr));

            // Signal that this object is active for termination detection
            // origin_addr is set to be parent if deficit == 0
            obj->terminator.signal(*this, action.origin_addr);

            // if predicate
            int predicate_resolution = function_events_manager->get_function_event_handler(
                action.predicate)(*this, action.obj_addr, action.nargs, action.args);
            // event_handlers[action.predicate](*this, action.obj_addr, action.nargs, action.args);

            if (predicate_resolution == 1) {

                // work
                function_events_manager->get_function_event_handler(action.work)(
                    *this, action.obj_addr, action.nargs, action.args);
                // event_handlers[action.work](*this, action.obj_addr, action.nargs, action.args);
                this->statistics.actions_performed_work++;

                // diffuse
                function_events_manager->get_function_event_handler(action.diffuse)(
                    *this, action.obj_addr, action.nargs, action.args);
                // event_handlers[action.diffuse](*this, action.obj_addr, action.nargs,
                // action.args);
            } else {
                // This action is discarded/subsumed
                this->statistics.actions_false_on_predicate++;
            }

            /*   if (obj->terminator.parent.value().cc_id == this->host_id) {
                  std::cout << "CC: " << this->id << " with parent = " << this->host_id
                            << " deficit: " << obj->terminator.deficit << "\n";
              } */

            obj->terminator.unsignal(*this);

        } else if (action.action_type == actionType::terminator_acknowledgement_action) {

            function_events_manager->get_acknowledgement_event_handler()(
                *this, action.obj_addr, action.nargs, action.args);
            // event_handlers[eventId::terminator_acknowledgement](*this, action.obj_addr, 0,
            // nullptr);
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
// TODO: Perhaps just move this to Cell.cpp since its the same code for all inherited classes
bool
ComputeCell::recv_operon(Operon operon, u_int32_t direction_in, u_int32_t distance_class)
{
    return this->recv_channel_per_neighbor[direction_in][distance_class].push(operon);
}

void
ComputeCell::prepare_a_cycle(std::vector<std::shared_ptr<Cell>>& CCA_chip)
{
    if (!this->is_compute_cell_active()) {
        return;
    }

    /* std::cout << this->id << ": Compute Cell " << this->cooridates
             << "  prepare_a_cycle : " << *this << "\n";  */

    // Move the operon from previous cycle (that was not able to be sent due to congestion) to
    // the send link of the network
    this->prepare_a_communication_cycle(CCA_chip);

    // Used for fairness. So that the channels don't get priority over others based on iterator
    // u_int32_t recv_channel_index = 0; // this->current_recv_channel_to_start_a_cycle;

    // Move the operon from previous cycle recv channel to thier destination: action queue or
    // send channel of a neighbor
    for (u_int32_t i = 0; i < this->recv_channel_per_neighbor.size(); i++) {
        for (u_int32_t j = 0; j < this->recv_channel_per_neighbor[i].size(); j++) {

            if (this->recv_channel_per_neighbor[i][j].size()) {

                // If this is greater them it is a bug
                assert(j <= this->hx + this->hy);

                std::vector<Operon> recv_operons;
                while (this->recv_channel_per_neighbor[i][j].size()) {
                    recv_operons.push_back(this->recv_channel_per_neighbor[i][j].front());
                    this->recv_channel_per_neighbor[i][j].pop();
                }

                std::vector<Operon> left_over_operons;
                for (Operon operon : recv_operons) {

                    u_int32_t dst_cc_id = operon.first.dst_cc_id;

                    // Check if this operon is destined for this compute cell
                    if (this->id == dst_cc_id) {
                        // this->action_queue.push(std::make_shared<Action>(operon_.second));
                        this->insert_action(operon.second);
                    } else {

                        Coordinates dst_cc_coordinates =
                            Cell::cc_id_to_cooridinate(dst_cc_id, this->shape, this->dim_y);

                        // To be routed in the mesh, by default simply
                        u_int32_t routing_cell_id = dst_cc_id;

                        // TODO: Use assert here instead
                        // Check if it needs to be routed via the secondary network
                        if (CCA_chip[dst_cc_id]->type != CellType::sink_cell) {

                            auto dst_compute_cell =
                                std::dynamic_pointer_cast<ComputeCell>(CCA_chip[dst_cc_id]);
                            assert(dst_compute_cell != nullptr);

                            // Routing 1: Aggresively use the H-tree (low latency network)

                            // If it is not nearby AND not in the same sinkcell (Htree block) then
                            // route it in second layer netowrk
                            if (!this->check_cut_off_distance(dst_cc_coordinates) &&
                                (this->sink_cell != dst_compute_cell->sink_cell)) {
                                routing_cell_id = Cell::cc_cooridinate_to_id(
                                    this->sink_cell.value(), this->shape, this->dim_y);
                            }

                            // Routing 1 Ends

                            // Routing 2: Use the mesh network more often

                            // Routing 2 Ends

                            // Routing 3: Adaptive routing

                            // Routing 3: Ends

                        } else {
                            std::cerr << "Bug! Operon can not be destined for a Sink Cell.\n";
                            exit(0);
                        }

                        // The operon needs to be sent/passed to some neighbor
                        u_int32_t channel_to_send = get_route_towards_cc_id(routing_cell_id);

                        if (this->send_channel_per_neighbor[channel_to_send].push(operon)) {
                            // Set the distance class for this operon
                            this->send_channel_per_neighbor_current_distance_class
                                [channel_to_send] = j + 1;
                        } else {
                            // Increament the stall counter for send/recv
                            this->statistics.stall_network_on_send++;
                            left_over_operons.push_back(operon);
                        }
                    }
                }

                for (Operon operon : left_over_operons) {
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

    if (!this->is_compute_cell_active()) {
        return;
    }
    /* std::cout << this->id << ": Compute Cell " << this->cooridates
              << "  run_a_computation_cycle : " << *this << "\n"; */
    // A single compute cell can perform work and communication in parallel in a single cycle
    // This function does both. First it performs work if there is any. Then it performs
    // communication

    // Initialize the counter for measuring resource usage and starvation. Start with all then
    // decreament as they are active. Later use that to find the percent active status for this
    // cycle. If nothing was decreamented it means that this cycle was totally inactive with the
    // CC starving. TODO: These counters are not reliable anymore. Need to rethink all of
    // this....
    this->statistics.cycle_resource_use_counter =
        ComputeCell::get_number_of_neighbors(this->shape) + 1; // +1 for logic

    // Apply the network operations from the previous cycle and prepare this cycle for
    // computation and communication
    this->prepare_a_cycle(CCA_chip);

    // Perform execution of work. Exectute a task if the task_queue is not empty
    if (!this->task_queue.empty()) {
        //  Get a task from the task_queue
        Task current_task = this->task_queue.front();

        // Check if the staging buffer is not full and the task type is send operon
        // In that case stall and don't do anything. Because the task can't send operon
        if (this->staging_operon_from_logic &&
            (current_task.first == taskType::send_operon_task_type)) {

            this->statistics.stall_logic_on_network++;
        } else {
            // Remove the task from the queue and execute it.

            this->task_queue.pop();
            // Execute the task
            current_task.second();
        }
        // The logic was active. In this cycle it diffused or performaned a task.
        this->statistics.cycle_resource_use_counter--;

    } else if (!this->action_queue.empty()) {
        // Else execute an action if the action_queue is not empty
        this->execute_action(function_events);

        // The logic was active. In this cycle it did predicate resolution and action work.
        this->statistics.cycle_resource_use_counter--;
    }
}

// This acts as synchronization and needs to be called before the actual communication cycle so
// as to not cause race conditions on the communicaton buffer. It also applies to the
// prepare_a_cycle since we want to move any staging operon from logic into the communication
// network so as to not cause it to wait on network not more than one cycle
void
ComputeCell::prepare_a_communication_cycle(std::vector<std::shared_ptr<Cell>>& CCA_chip)
{
    if (!this->is_compute_cell_active()) {
        return;
    }

    /*    std::cout << this->id << ": Compute Cell " << this->cooridates
                << "  prepare_a_communication_cycle : " << *this << "\n";  */

    if (this->staging_operon_from_logic) {
        Operon operon_ = this->staging_operon_from_logic.value();
        u_int32_t dst_cc_id = operon_.first.dst_cc_id;

        // Check if this operon is destined for this compute cell
        // Meaning both src and dst vertices are on the same compute cell?
        if (this->id == dst_cc_id) {
            // this->action_queue.push(std::make_shared<Action>(operon_.second));
            this->insert_action(operon_.second);
            // Flush the channel buffer
            this->staging_operon_from_logic = std::nullopt;
        } else {

            Coordinates dst_cc_coordinates =
                Cell::cc_id_to_cooridinate(dst_cc_id, this->shape, this->dim_y);

            u_int32_t routing_cell_id = dst_cc_id;

            // Check if it needs to be routed via the secondary network
            if (CCA_chip[dst_cc_id]->type != CellType::sink_cell) {

                auto dst_compute_cell = std::dynamic_pointer_cast<ComputeCell>(CCA_chip[dst_cc_id]);
                assert(dst_compute_cell != nullptr);

                // If it is not nearby AND not in the same sinkcell (Htree block) then route it in
                // second layer netowrk
                if (!this->check_cut_off_distance(dst_cc_coordinates) &&
                    (this->sink_cell != dst_compute_cell->sink_cell)) {
                    routing_cell_id = Cell::cc_cooridinate_to_id(
                        this->sink_cell.value(), this->shape, this->dim_y);
                }
            } else {
                std::cerr << "Bug! Operon can not be destined for a Sink Cell.\n";
                exit(0);
            }

            // Based on the routing algorithm and the shape of CCs it will return which neighbor
            // to pass this operon to. The returned value is the index [0...number of neighbors)
            // coresponding clockwise the channel id of the physical shape.
            u_int32_t channel_to_send = this->get_route_towards_cc_id(routing_cell_id);

            if (this->send_channel_per_neighbor[channel_to_send].push(
                    this->staging_operon_from_logic.value())) {

                // Set to distance class 0 since this operon originates from this CC
                this->send_channel_per_neighbor_current_distance_class[channel_to_send] = 0;

                // Empty the staging buffer
                this->staging_operon_from_logic = std::nullopt;
            } else {

                this->statistics.stall_logic_on_network++;
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
        int receiving_direction[4] = { 2, 3, 0, 1 };

        for (u_int32_t i = 0; i < this->send_channel_per_neighbor.size(); i++) {

            if (this->send_channel_per_neighbor[i].size()) {
                /* std::cout << "CC : " << this->cooridates << " send_channel_per_neighbor[" << i
                          << "].size(): " << this->send_channel_per_neighbor[i].size() << "\n"; */

                // TODO: NOTE: the size of each channel is fixed to be `1`, therfore this leftover
                // etc is of no use for now
                std::vector<Operon> send_operons;
                while (this->send_channel_per_neighbor[i].size()) {
                    send_operons.push_back(this->send_channel_per_neighbor[i].front());
                    this->send_channel_per_neighbor[i].pop();
                }

                std::vector<Operon> left_over_operons;
                for (Operon operon : send_operons) {

                    // Update the cycle_resource_use_counter. TODO fix these counters lol
                    this->statistics.cycle_resource_use_counter--;
                    u_int32_t dst_cc_id = operon.first.dst_cc_id;

                    // Check if this operon is destined for this compute cell
                    assert(this->id != dst_cc_id);
                    assert(this->neighbor_compute_cells[i] != std::nullopt);

                    u_int32_t neighbor_id_ = this->neighbor_compute_cells[i].value().first;

                    if (!CCA_chip[neighbor_id_]->recv_operon(
                            operon,
                            receiving_direction[i],
                            this->send_channel_per_neighbor_current_distance_class[i])) {

                        /* std::cout << "\tCC : " << this->cooridates
                                  << " Not able to send to neighbor: "
                                  << this->neighbor_compute_cells[i].value().second
                                  << " neighbor recieve size: "
                                  << CCA_chip[neighbor_id_]
                                         ->recv_channel_per_neighbor[receiving_direction[i]]
                                         .size()
                                  << "\n"; */

                        this->statistics.stall_network_on_recv++;
                        // increament the stall counter for send/recv
                        left_over_operons.push_back(operon);
                    }
                    for (Operon operon : left_over_operons) {
                        this->send_channel_per_neighbor[i].push(operon);
                    }
                }
            }
        }
    }
    // Since this is the end of the cycle find out how much percent of the CC was active and
    // whether it was inactive altogether? TODO: Fix all these counter. They are meaningless at
    // this point in the developmetn
    u_int32_t number_of_resources_per_cc = ComputeCell::get_number_of_neighbors(this->shape) + 1;
    if (this->statistics.cycle_resource_use_counter == number_of_resources_per_cc) {
        this->statistics.cycles_inactive++;
    } else {
        this->statistics.cycles_resource_usage +=
            (number_of_resources_per_cc - this->statistics.cycle_resource_use_counter) /
            static_cast<long double>(number_of_resources_per_cc);
    }
}

// Checks if the compute cell is active or not
bool
ComputeCell::is_compute_cell_active()
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
    return (!this->action_queue.empty() || !this->task_queue.empty() ||
            (this->staging_operon_from_logic) || send_channels || recv_channels);
}

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
#include "Cell.hpp"
#include "Function.hpp"
#include "SimpleVertex.hpp"

// For memcpy()
#include <cstring>

// TODO: move this to application
void
print_SimpleVertex(const ComputeCell& cc, const Address& vertex_addr)
{
    if (vertex_addr.cc_id != cc.id) {
        std::cout << "Invalid addr! The vertex does not exist on this CC\n";
        return;
    }

    SimpleVertex<Address>* vertex = (SimpleVertex<Address>*)cc.get_object(vertex_addr);
    std::cout << "Vertex ID: " << vertex->id << ", Addr: " << vertex_addr
              << " sssp_distance: " << vertex->sssp_distance << "\n";

    for (int i = 0; i < vertex->number_of_edges; i++) {
        std::cout << "\t\t[" << vertex->edges[i].edge << ", {w: " << vertex->edges[i].weight
                  << "} ]";
    }
    std::cout << std::endl;
}

// Get the object memory location at address addr_in
void*
ComputeCell::get_object(Address addr_in) const
{
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

// Returns the offset in memory for this newly created object
std::optional<Address>
ComputeCell::create_object_in_memory(void* obj, size_t size_of_obj)
{
    if (this->memory_available_in_bytes() < size_of_obj) {
        return std::nullopt;
    }

    u_int32_t obj_memory_addr_offset = get_memory_curr_ptr_offset();
    memcpy(this->memory_curr_ptr, obj, size_of_obj);
    this->memory_curr_ptr += size_of_obj;

    return Address(this->id, obj_memory_addr_offset);
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

void
ComputeCell::execute_action()
{

    if (!this->action_queue.empty()) {
        Action action = this->action_queue.front();
        this->action_queue.pop();

        if constexpr (debug_code == true) {
            print_SimpleVertex(*this, action.obj_addr);
        }

        // if predicate
        int predicate_resolution =
            event_handlers[action.predicate](*this, action.obj_addr, action.nargs, action.args);

        if (predicate_resolution == 1) {
            // if work
            event_handlers[action.work](*this, action.obj_addr, action.nargs, action.args);
            this->statistics.actions_performed_work++;

            // if diffuse
            event_handlers[action.diffuse](*this, action.obj_addr, action.nargs, action.args);
        } else {
            // This actions is discarded/subsumed
            this->statistics.actions_false_on_predicate++;
        }
        // Increament the counter for actions that were invoked
        this->statistics.actions_invoked++;
        return;
    }
    std::cout << "Cannot execute action as the action_queue is empty!\n";
}

u_int32_t
ComputeCell::get_route_towards_cc_id(u_int32_t dst_cc_id)
{

    // Algorithm == dimensional routing
    if (this->shape == computeCellShape::square) {
        // Remember for a square shaped CC there are four links to neighbors enumerated in
        // clockwise 0 = left, 1 = up, 2 = right, and 3 = down

        Coordinates dst_cc_coordinates =
            ComputeCell::cc_id_to_cooridinate(dst_cc_id, this->shape, this->dim_x, this->dim_y);

        if constexpr (debug_code) {
            std::cout << "cc id : " << this->id << " dst_cc_coordinates = ("
                      << dst_cc_coordinates.first << ", " << dst_cc_coordinates.second
                      << ") -- origin = ( " << this->cooridates.first << ", "
                      << this->cooridates.second << ")\n";
        }
        // First check vertically in y axis then horizontally in x axis
        if (this->cooridates.second > dst_cc_coordinates.second) {
            return 1; // Clockwise 1 = up
        } else if (this->cooridates.second < dst_cc_coordinates.second) {
            return 3; // Clockwise 3 = down
        } else if (this->cooridates.first > dst_cc_coordinates.first) {
            // std::cout <<"left\n";
            return 0; // Clockwise 0 = left
        } else if (this->cooridates.first < dst_cc_coordinates.first) {
            // std::cout <<"right\n";
            return 2; // Clockwise 2 = right
        }
        // TODO: use Cell:: instead
        std::cerr << ComputeCell::get_compute_cell_shape_name(this->shape)
                  << "Bug: routing not sucessful!\n";
    }
    // Shape or routing not supported
    std::cerr << ComputeCell::get_compute_cell_shape_name(this->shape)
              << " or routing not supported!\n";
    exit(0);
}

void
ComputeCell::prepare_a_cycle()
{
    // Move the operon from previous cycle (that was not able to be sent due to congestion) to the
    // send link of the network
    if (this->staging_operon_from_logic) {
        Operon operon_ = this->staging_operon_from_logic.value();
        u_int32_t dst_cc_id = operon_.first;

        // Check if this operon is destined for this compute cell
        // Meaning both src and dst vertices are on the same compute cell?
        if (this->id == dst_cc_id) {
            this->insert_action(operon_.second);
            //  Flush the channel buffer
            this->staging_operon_from_logic = std::nullopt;
        } else {

            // Based on the routing algorithm and the shape of CCs it will return which neighbor to
            // pass this operon to. The returned value is the index [0...number of neighbors)
            // coresponding clockwise the channel id of the physical shape.
            u_int32_t channel_to_send = this->get_route_towards_cc_id(dst_cc_id);

            if (this->send_channel_per_neighbor[channel_to_send] == std::nullopt) {

                // Prepare the send channel
                this->send_channel_per_neighbor[channel_to_send] = this->staging_operon_from_logic;
                // Empty the staging buffer
                this->staging_operon_from_logic = std::nullopt;
            } else {
                this->statistics.stall_logic_on_network++;
            }
        }
    }

    // Move the operon from previous cycle recv channel to thier destination: action queue or send
    // channel of a neighbor
    for (int i = 0; i < this->recv_channel_per_neighbor.size(); i++) {
        if (this->recv_channel_per_neighbor[i]) {
            Operon operon_ = this->recv_channel_per_neighbor[i].value();
            u_int32_t dst_cc_id = operon_.first;

            // Check if this operon is destined for this compute cell
            if (this->id == dst_cc_id) {
                // this->action_queue.push(std::make_shared<Action>(operon_.second));
                this->insert_action(operon_.second);
                // Flush the channel buffer
                this->recv_channel_per_neighbor[i] = std::nullopt;
            } else {
                // It means the operon needs to be sent/passed to some neighbor
                u_int32_t channel_to_send = get_route_towards_cc_id(dst_cc_id);

                if (this->send_channel_per_neighbor[channel_to_send] == std::nullopt) {
                    // Prepare the send channel
                    this->send_channel_per_neighbor[channel_to_send] = operon_;
                    // Flush the channel buffer
                    this->recv_channel_per_neighbor[i] = std::nullopt;
                } else {
                    // Increament the stall counter for send/recv
                    this->statistics.stall_network_on_send++;
                }
            }
        }
    }
}

void
ComputeCell::run_a_computation_cycle()
{
    // A single compute cell can perform work and communication in parallel in a single cycle
    // This function does both. First it performs work if there is any. Then it performs
    // communication

    // Initialize the counter for measuring resource usage and starvation. Start with all then
    // decreament as they are active. Later use that to find the percent active status for this
    // cycle. If nothing was decreamented it means that this cycle was totally inactive with the CC
    // starving.
    this->statistics.cycle_resource_use_counter =
        ComputeCell::get_number_of_neighbors(this->shape) + 1; // +1 for logic

    // Apply the network operations from the previous cycle and prepare this cycle for computation
    // and communication
    this->prepare_a_cycle();

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
        this->execute_action();

        // The logic was active. In this cycle it did predicate resolution and action work.
        this->statistics.cycle_resource_use_counter--;
    }
}

// This act as synchronization and needs to be called before the actual communication cycle so as to
// not cause race conditions on the communicaton buffer. It also applies to the ... TODO fix comment
void
ComputeCell::prepare_a_communication_cycle()
{
    if (this->staging_operon_from_logic) {
        Operon operon_ = this->staging_operon_from_logic.value();
        u_int32_t dst_cc_id = operon_.first;

        // Check if this operon is destined for this compute cell
        // Meaning both src and dst vertices are on the same compute cell?
        if (this->id == dst_cc_id) {
            // this->action_queue.push(std::make_shared<Action>(operon_.second));
            this->insert_action(operon_.second);
            // Flush the channel buffer
            this->staging_operon_from_logic = std::nullopt;
        } else {

            // Based on the routing algorithm and the shape of CCs it will return which neighbor to
            // pass this operon to. The returned value is the index [0...number of neighbors)
            // coresponding clockwise the channel id of the physical shape.

            // std::cout << "cc id: " << this->id << " dst_cc_id: " << dst_cc_id <<
            // "prepare_a_communication_cycle: staging_operon_from_logic get_route_towards_cc_id\n";
            u_int32_t channel_to_send = get_route_towards_cc_id(dst_cc_id);

            if (this->send_channel_per_neighbor[channel_to_send] == std::nullopt) {
                // Prepare the send channel
                this->send_channel_per_neighbor[channel_to_send] = this->staging_operon_from_logic;
                // Empty the staging buffer
                this->staging_operon_from_logic = std::nullopt;
            } else {
                this->statistics.stall_logic_on_network++;
                // increase the stall counter
            }
        }
    }
}

void
ComputeCell::run_a_communication_cycle(std::vector<std::shared_ptr<Cell>>& CCA_chip)
{
    // For shape square
    if (this->shape == computeCellShape::square) {
        int receiving_direction[4] = { 2, 3, 0, 1 };

        for (int i = 0; i < this->send_channel_per_neighbor.size(); i++) {

            if (this->send_channel_per_neighbor[i]) { // is not std::nullopt

                // Update the cycle_resource_use_counter
                this->statistics.cycle_resource_use_counter--;

                Operon operon_ = this->send_channel_per_neighbor[i].value();
                u_int32_t dst_cc_id = operon_.first;

                // Check if this operon is destined for this compute cell
                if (this->id == dst_cc_id) {
                    std::cerr << "Bug! run_a_communication_cycle()\tsend_channel_per_neighbor[" << i
                              << "] on cc " << this->id
                              << " is sending to itself. See why this condition even exists? \n";
                    exit(0);
                }

                if (this->neighbor_compute_cells[i] == std::nullopt) {
                    std::cerr << "Bug! neighbor_compute_cells[" << i << "] on cc " << this->id
                              << " is nullopt. See why the send_channel_per_neighbor is not? \n";
                    exit(0);
                }

                u_int32_t neighbor_id_ = this->neighbor_compute_cells[i].value().first;
                if (CCA_chip[neighbor_id_]->recv_channel_per_neighbor[receiving_direction[i]] ==
                    std::nullopt) {

                    CCA_chip[neighbor_id_]->recv_channel_per_neighbor[receiving_direction[i]] =
                        operon_;

                    // Flush the channel buffer
                    this->send_channel_per_neighbor[i] = std::nullopt;
                } else {
                    this->statistics.stall_network_on_recv++;
                    // increament the stall counter for send/recv
                }
            }
        }
    }

    // Since this is the end of the cycle find out how much percent of the CC was active and whether
    // it was inactive altogether?
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
    for (int i = 0; i < this->number_of_neighbors; i++) {
        if (this->send_channel_per_neighbor[i]) {
            send_channels = true;
            break;
        }
        if (this->recv_channel_per_neighbor[i]) {
            recv_channels = true;
            break;
        }
    }
    return (!this->action_queue.empty() || !this->task_queue.empty() ||
            (this->staging_operon_from_logic) || send_channels || recv_channels);
}

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
#include "Function.hpp"
#include "SimpleVertex.hpp"

// TODO: move this to application
void
print_SimpleVertex(const ComputeCell& cc, const Address& vertex_addr)
{
    if (vertex_addr.cc_id != cc.id) {
        std::cout << "Invalid addr! The vertex does not exist on this CC\n";
        return;
    }

    SimpleVertex<Address>* vertex = (SimpleVertex<Address>*)cc.get_object(vertex_addr);
    std::cout << "Vertex ID: " << vertex->id << ", Addr: " << vertex_addr << "\n";

    for (int i = 0; i < vertex->number_of_edges; i++) {
        std::cout << "[" << vertex->edges[i].edge << ", {w: " << vertex->edges[i].weight << "} ]";
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

void
ComputeCell::insert_action(const std::shared_ptr<Action>& action)
{
    this->action_queue.push(action);
}

// Checks if the compute cell is active or not
// TODO: when communication is added then update checks for the communication buffer too
bool
ComputeCell::is_compute_cell_active()
{
    return (!this->action_queue.empty() || !this->task_queue.empty());
}

void
ComputeCell::add_neighbor(
    std::pair<u_int32_t, std::pair<u_int32_t, u_int32_t>> neighbor_compute_cell)
{
    this->neighbor_compute_cells.push_back(neighbor_compute_cell);
}
void
ComputeCell::execute_action()
{

    if (!this->action_queue.empty()) {
        std::shared_ptr<Action> action = this->action_queue.front();
        this->action_queue.pop();

        if constexpr (debug_code == true) {
            print_SimpleVertex(*this, action->obj_addr);
        }

        // TODO: actually put the ifs

        // if predicate
        int predicate_resolution =
            event_handlers[action->predicate](*this, action->obj_addr, action->nargs, action->args);
        std::cout << "execute_action() : predicate_resolution = " << predicate_resolution << "\n";
        // if work
        event_handlers[action->work](*this, action->obj_addr, action->nargs, action->args);

        // if diffuse
        event_handlers[action->diffuse](*this, action->obj_addr, action->nargs, action->args);
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

        std::pair<u_int32_t, u_int32_t> dst_cc_coordinates =
            ComputeCell::cc_id_to_cooridinate(dst_cc_id, this->shape, this->dim_x, this->dim_y);

        // First check vertically in y axis then horizontally in x axis
        if (this->cooridates.second > dst_cc_coordinates.second) {
            return 1; // Clockwise 1 = up
        } else if (this->cooridates.second < dst_cc_coordinates.second) {
            return 3; // Clockwise 3 = down
        } else if (this->cooridates.first > dst_cc_coordinates.first) {
            return 0; // Clockwise 0 = left
        } else if (this->cooridates.first < dst_cc_coordinates.first) {
            return 2; // Clockwise 2 = right
        }
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

        // Based on the routing algorithm and the shape of CCs it will return which neighbor to pass
        // this operon to. The returned value is the index [0...number of neighbors) coresponding
        // clockwise the channel id of the physical shape.
        u_int32_t channel_to_send = get_route_towards_cc_id(dst_cc_id);

        if (this->send_channel_per_neighbor[channel_to_send] != std::nullopt) {
            std::cerr << "Bug! send_channel_per_neighbor " << channel_to_send
                      << "shouldn't be non-empty\n";
            exit(0);
        }

        // Prepare the send channel
        this->send_channel_per_neighbor[channel_to_send] = this->staging_operon_from_logic;
        // Empty the staging buffer
        this->staging_operon_from_logic = std::nullopt;
    }
    // Move the operon from previous cycle recv channel to thier destination: action queue or send
    // channel of a neighbor
    std::cout << "this->recv_channel_per_neighbor.size() = " << this->recv_channel_per_neighbor.size() << "\n";
    for (int i = 0; i<this->recv_channel_per_neighbor.size(); i++){
        if(this->recv_channel_per_neighbor[i]){
            Operon operon_ = this->recv_channel_per_neighbor[i].value();
            u_int32_t dst_cc_id = operon_.first;
            
            // Check if this operon is destined for this compute cell
            if (this->id == dst_cc_id){
             //   this->action_queue.push(operon_.second);
            }

        }
    }
}

bool
ComputeCell::run_a_computation_cycle()
{

    // A single compute cell can perform work and communication in parallel in a single cycle
    // This function does both. First it performs work if there is any. Then it performs
    // communication

    // Perform execution of work
    // Exectute a task if the task_queue is not empty
    if (!this->task_queue.empty()) {
        // std::cout << "run_a_cycle | task | CC : " << this->id << "\n";
        //  Get a task from the task_queue
        Task current_task = this->task_queue.front();
        std::cout << "(this->staging_operon_from_logic != std::nullopt) = "
                  << (this->staging_operon_from_logic != std::nullopt)
                  << "(current_task.first == taskType::send_operon_task_type) = "
                  << (current_task.first == taskType::send_operon_task_type) << "\n";
        // Check if the staging buffer is not full
        if ((this->staging_operon_from_logic != std::nullopt) &&
            (current_task.first == taskType::send_operon_task_type)) {
            std::cout << "cc: " << this->id
                      << " staging_operon_from_logic buffer is full! This cycle is stalled. Will "
                         "not dequeue the task from the task queue\n";

        } else {
            // remove the task from the queue and execute it.
            this->task_queue.pop();
            // Execute the task
            current_task.second();
        }
    } else if (!this->action_queue
                    .empty()) { // Else execute an action if the action_queue is not empty

        // std::cout << "run_a_cycle | action | CC : " << this->id << "\n";

        this->execute_action();
    }

    // Return the active status of this CC and later it can be used to update the global active
    // compute cells count
    return this->is_compute_cell_active();
}
bool
ComputeCell::run_a_communication_cycle()
{

    // Perform communication

    // House Keeping: Copy communication operator from neighbor to the current communication
    // buffer of this CC
    return true;
}

std::string
ComputeCell::get_compute_cell_shape_name(computeCellShape shape)
{
    switch (shape) {
        case (computeCellShape::block_1D):
            return std::string("block_1D");
            break;
        case (computeCellShape::triangular):
            return std::string("triangular");
            break;
        case (computeCellShape::square):
            return std::string("square");
            break;
        case (computeCellShape::hexagon):
            return std::string("hexagon");
            break;

        default:
            return std::string("Invalid Shape");
            break;
    }
}

computeCellShape
ComputeCell::get_compute_cell_shape_enum(std::string shape)
{
    if (shape == "block_1D") {
        return computeCellShape::block_1D;
    } else if (shape == "triangular") {
        return computeCellShape::triangular;
    } else if (shape == "sqaure") {
        return computeCellShape::square;
    } else if (shape == "hexagon") {
        return computeCellShape::hexagon;
    } else {
        return computeCellShape::computeCellShape_invalid;
    }
}

u_int32_t
ComputeCell::get_number_of_neighbors(computeCellShape shape_in)
{
    switch (shape_in) {
        case (computeCellShape::block_1D):
            return 2;
            break;
        case (computeCellShape::triangular):
            return 3;
            break;
        case (computeCellShape::square):
            return 4;
            break;
        case (computeCellShape::hexagon):
            return 6;
            break;

        default:
            return 0;
            break;
    }
}

std::pair<u_int32_t, u_int32_t>
ComputeCell::cc_id_to_cooridinate(u_int32_t cc_id,
                                  computeCellShape shape_,
                                  u_int32_t dim_x,
                                  u_int32_t dim_y)
{

    if (shape_ == computeCellShape::square) {

        return std::pair<u_int32_t, u_int32_t>(cc_id % dim_y, cc_id / dim_y);
    }
    // Shape not supported
    std::cerr << ComputeCell::get_compute_cell_shape_name(shape_) << " not supported!\n";
    exit(0);
}

u_int32_t
ComputeCell::cc_cooridinate_to_id(std::pair<u_int32_t, u_int32_t> cc_cooridinate,
                                  computeCellShape shape_,
                                  u_int32_t dim_x,
                                  u_int32_t dim_y)
{

    if (shape_ == computeCellShape::square) {
        auto [x, y] = cc_cooridinate;
        // std::cout << "cc_cooridinate_to_id: (" << x << ", " << y << ") ----> " << (y * this->dim)
        // + x << "\n";
        return (y * dim_x) + x;
    }
    // Shape not supported
    std::cerr << ComputeCell::get_compute_cell_shape_name(shape_) << " not supported!\n";
    exit(0);
}

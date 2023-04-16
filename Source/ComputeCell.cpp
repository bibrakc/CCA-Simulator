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

// For memcpy()
#include <cstring>

std::ostream&
operator<<(std::ostream& os, const Operon& operon_)
{
    os << "Operon: cc_id " << operon_.first << ", Action target addr: " << operon_.second.obj_addr;

    os << "\n";
    return os;
}

std::ostream&
operator<<(std::ostream& os, const std::vector<std::optional<Operon>>& operons_)
{

    for (auto& op_ : operons_) {
        if (op_ == std::nullopt) {
            os << "[nullopt] ";
        } else {
            os << op_.value();
        }
    }
    os << "\n";
    return os;
}

template<typename To, typename From>
inline std::pair<To, To>
convert_internal_type_of_pair(const std::pair<From, From>& p)
{
    return std::make_pair(static_cast<To>(p.first), static_cast<To>(p.second));
}

inline bool
ComputeCell::cc_exists(const SignedCoordinates cc_coordinate)
{
    auto [cc_coordinate_x, cc_coordinate_y] = cc_coordinate;
    if (this->shape == computeCellShape::square) {

        // If invalid
        if ((cc_coordinate_x < 0) || (cc_coordinate_x >= this->dim_y) || (cc_coordinate_y < 0) ||
            (cc_coordinate_y >= this->dim_x)) {
            return false;
        } else {
            return true;
        }
    }
    // Shape not supported
    std::cerr << ComputeCell::get_compute_cell_shape_name(this->shape) << " not supported!\n";
    exit(0);
}

void
ComputeCell::add_neighbor_compute_cells()
{

    if (this->shape == computeCellShape::square) {

        // Note: The coordinates are of type unsigned int and we need to do arithematics that
        // may give negative int values. Therefore, we cast them to signed int
        auto coordinate_signed = convert_internal_type_of_pair<int32_t>(this->cooridates);
        int32_t cc_coordinate_x = coordinate_signed.first;
        int32_t cc_coordinate_y = coordinate_signed.second;

        // Left neighbor
        SignedCoordinates left_neighbor =
            std::pair<int32_t, int32_t>(cc_coordinate_x - 1, cc_coordinate_y);
        if (this->cc_exists(left_neighbor)) {
            auto left_neighbor_unsigned = convert_internal_type_of_pair<u_int32_t>(left_neighbor);

            auto left_neighbor_id = ComputeCell::cc_cooridinate_to_id(
                left_neighbor_unsigned, this->shape, this->dim_x, this->dim_y);

            this->add_neighbor(std::pair<u_int32_t, std::pair<u_int32_t, u_int32_t>>(
                left_neighbor_id, left_neighbor_unsigned));
        } else {
            this->add_neighbor(std::nullopt);
        }

        // Up neighbor
        SignedCoordinates up_neighbor =
            std::pair<int32_t, int32_t>(cc_coordinate_x, cc_coordinate_y - 1);
        if (this->cc_exists(up_neighbor)) {
            auto up_neighbor_unsigned = convert_internal_type_of_pair<u_int32_t>(up_neighbor);

            auto up_neighbor_id = ComputeCell::cc_cooridinate_to_id(
                up_neighbor_unsigned, this->shape, this->dim_x, this->dim_y);

            this->add_neighbor(std::pair<u_int32_t, std::pair<u_int32_t, u_int32_t>>(
                up_neighbor_id, up_neighbor_unsigned));
        } else {
            this->add_neighbor(std::nullopt);
        }
        // Right neighbor
        SignedCoordinates right_neighbor =
            std::pair<int32_t, int32_t>(cc_coordinate_x + 1, cc_coordinate_y);
        if (this->cc_exists(right_neighbor)) {
            auto right_neighbor_unsigned = convert_internal_type_of_pair<u_int32_t>(right_neighbor);

            auto right_neighbor_id = ComputeCell::cc_cooridinate_to_id(
                right_neighbor_unsigned, this->shape, this->dim_x, this->dim_y);

            this->add_neighbor(std::pair<u_int32_t, std::pair<u_int32_t, u_int32_t>>(
                right_neighbor_id, right_neighbor_unsigned));
        } else {
            this->add_neighbor(std::nullopt);
        }
        // Down neighbor
        SignedCoordinates down_neighbor =
            std::pair<int32_t, int32_t>(cc_coordinate_x, cc_coordinate_y + 1);
        if (this->cc_exists(down_neighbor)) {
            auto down_neighbor_unsigned = convert_internal_type_of_pair<u_int32_t>(down_neighbor);

            auto down_neighbor_id = ComputeCell::cc_cooridinate_to_id(
                down_neighbor_unsigned, this->shape, this->dim_x, this->dim_y);

            this->add_neighbor(std::pair<u_int32_t, std::pair<u_int32_t, u_int32_t>>(
                down_neighbor_id, down_neighbor_unsigned));
        } else {
            this->add_neighbor(std::nullopt);
        }

    } else if (this->shape == computeCellShape::block_1D) {

        std::cerr << ComputeCell::get_compute_cell_shape_name(this->shape) << " not supported!\n";
        exit(0);

    } else {
        // Shape not supported
        std::cerr << ComputeCell::get_compute_cell_shape_name(this->shape) << " not supported!\n";
        exit(0);
    }
}

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

void
ComputeCell::insert_action(const Action& action)
{
    this->action_queue.push(action);
    this->statistics.actions_pushed++;
}

void
ComputeCell::add_neighbor(
    std::optional<std::pair<u_int32_t, std::pair<u_int32_t, u_int32_t>>> neighbor_compute_cell)
{
    this->neighbor_compute_cells.push_back(neighbor_compute_cell);
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

        // std::cout << "execute_action() : predicate_resolution = " << predicate_resolution <<
        // "\n";

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

        std::pair<u_int32_t, u_int32_t> dst_cc_coordinates =
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

        /*     std::cout << "prepare_a_cycle: cc id : " << this->id << " dst_cc_id : " << dst_cc_id
                     << "\n";  */

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

            //    std::cout << "cc id: " << this->id << " dst_cc_id: " << dst_cc_id <<
            //    "prepare_a_cycle: staging_operon_from_logic get_route_towards_cc_id\n";
            u_int32_t channel_to_send = this->get_route_towards_cc_id(dst_cc_id);

            // Something going on here with these preparations of cycles
            /*    if (this->send_channel_per_neighbor[channel_to_send]) {
                   std::cerr << "Bug! send_channel_per_neighbor " << channel_to_send
                             << "shouldn't be non-empty\n";
                   exit(0);
               } */
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
    /*  std::cout << "this->recv_channel_per_neighbor.size() = "
               << this->recv_channel_per_neighbor.size() << "\n"; */
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

                //        std::cout << "cc id: " << this->id << " dst_cc_id: " << dst_cc_id <<
                //        "prepare_a_cycle: recv loop get_route_towards_cc_id\n";
                u_int32_t channel_to_send = get_route_towards_cc_id(dst_cc_id);
                // std::cout << "cc id : " << this->id << " channel_to_send = " << channel_to_send
                // << "IT IS THIS!!!\n";
                if (this->send_channel_per_neighbor[channel_to_send] == std::nullopt) {
                    // Prepare the send channel
                    this->send_channel_per_neighbor[channel_to_send] = operon_;
                    // Flush the channel buffer
                    this->recv_channel_per_neighbor[i] = std::nullopt;
                } else {
                    this->statistics.stall_network_on_send++;
                    // std::cout << "stalled prepare_a_cycle send_channel_per_neighbor\n";
                    //  increament the stall counter for send/recv
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

    // Apply the network operations from the previous cycle and prepare this cycle for computation
    // and communication
    this->prepare_a_cycle();

    // Perform execution of work
    // Exectute a task if the task_queue is not empty
    if (!this->task_queue.empty()) {
        // std::cout << "run_a_cycle | task | CC : " << this->id << "\n";
        //  Get a task from the task_queue
        Task current_task = this->task_queue.front();

        /*         std::cout << "(this->staging_operon_from_logic != std::nullopt) = "
                          << (this->staging_operon_from_logic != std::nullopt)
                          << "(current_task.first == taskType::send_operon_task_type) = "
                          << (current_task.first == taskType::send_operon_task_type) << "\n"; */

        // Check if the staging buffer is not full and the task type is send operon
        // In that case stall and don't do anything. Because the task can't send operon
        if (this->staging_operon_from_logic &&
            (current_task.first == taskType::send_operon_task_type)) {

            /*       std::cout << "cc: " << this->id
                            << " staging_operon_from_logic buffer is full! This cycle is stalled.
               Will " "not dequeue the task from the task queue\n"; */

            this->statistics.stall_logic_on_network++;

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
        //  return;
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

// Checks if the compute cell is active or not
// TODO: when communication is added then update checks for the communication buffer too
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

void
ComputeCell::run_a_communication_cycle(std::vector<std::shared_ptr<ComputeCell>>& CCA_chip)
{
    // Perform communication
    /*     std::cout << "this->send_channel_per_neighbor.size() = "
                  << this->send_channel_per_neighbor.size() << "\n"; */

    // For shape square
    if (this->shape == computeCellShape::square) {
        int receiving_direction[4] = { 2, 3, 0, 1 };

        // std::cout << "cc id: " << this->id << " run communication cycle send_channel_per_neighbor
        // " << this->send_channel_per_neighbor;
        for (int i = 0; i < this->send_channel_per_neighbor.size(); i++) {

            if (this->send_channel_per_neighbor[i]) { // is not std::nullopt

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
}
/*
bool
ComputeCell::run_a_cycle()
{
    this->prepare_a_cycle();
    this->r


     // Return the active status of this CC and later it can be used to update the global active
    // compute cells count
    return this->is_compute_cell_active();
} */

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
        return (y * dim_y) + x;
    }
    // Shape not supported
    std::cerr << ComputeCell::get_compute_cell_shape_name(shape_) << " not supported!\n";
    exit(0);
}

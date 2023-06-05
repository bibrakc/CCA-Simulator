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
#include "SinkCell.hpp"
#include "Cell.hpp"
#include "HtreeNode.hpp"

#include <cassert>

// For memcpy()
#include <cstring>

// Returns the route in the mesh network

u_int32_t
SinkCell::get_route_towards_cc_id(u_int32_t dst_cc_id)
{
    return get_west_first_route_towards_cc_id(dst_cc_id);
}

// This has deadlocks
u_int32_t
SinkCell::get_dimensional_route_towards_cc_id(u_int32_t dst_cc_id)
{

    // Algorithm == dimensional routing
    if (this->shape == computeCellShape::square) {
        // Remember for a square shaped CC there are four links to neighbors enumerated in
        // clockwise 0 = left, 1 = up, 2 = right, and 3 = down

        Coordinates dst_cc_coordinates =
            Cell::cc_id_to_cooridinate(dst_cc_id, this->shape, this->dim_y);

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

        std::cerr << Cell::get_compute_cell_shape_name(this->shape)
                  << "Bug: routing not sucessful!\n";
    }
    // Shape or routing not supported
    std::cerr << Cell::get_compute_cell_shape_name(this->shape) << " or routing not supported!\n";
    exit(0);
}

u_int32_t
SinkCell::get_west_first_route_towards_cc_id(u_int32_t dst_cc_id)
{

    // Algorithm == dimensional routing
    if (this->shape == computeCellShape::square) {
        // Remember for a square shaped CC there are four links to neighbors enumerated in
        // clockwise 0 = left, 1 = up, 2 = right, and 3 = down

        Coordinates dst_cc_coordinates =
            Cell::cc_id_to_cooridinate(dst_cc_id, this->shape, this->dim_y);

        /*      if (this->cooridates.second != 0){
                 //send down
                 return 3;
             } */

        // West first routing restricts turns to the west side. Take west/left first if needed

        // std::cout << "SinkCell: Routing from:" << this->cooridates << " --> " <<
        // dst_cc_coordinates;
        if (this->cooridates.first > dst_cc_coordinates.first) {
            return 0; // Clockwise 0 = left
        } else if ((this->cooridates.first < dst_cc_coordinates.first) &&
                   (this->cooridates.second > dst_cc_coordinates.second)) {
            // upper right quadrant

            // send up or right
            // based on availablity send there. Right now just send to up
            return 1;

        } else if ((this->cooridates.first < dst_cc_coordinates.first) &&
                   (this->cooridates.second < dst_cc_coordinates.second)) {

            // lower right quadrant

            // send down or right
            // based on availablity send there. Right now just send to down
            return 3;
        } else if (this->cooridates.first < dst_cc_coordinates.first) {
            // send to right
            return 2;
        } else if (this->cooridates.second < dst_cc_coordinates.second) {
            // send to down
            return 3;
        } else if (this->cooridates.second > dst_cc_coordinates.second) {
            // send to up
            return 1;
        }

        std::cerr << Cell::get_compute_cell_shape_name(this->shape)
                  << " Bug: routing not sucessful!\n";
    }
    // Shape or routing not supported
    std::cerr << Cell::get_compute_cell_shape_name(this->shape) << " or routing not supported!\n";
    exit(0);
}

std::pair<u_int32_t, u_int32_t>
return_asymetric_neighbors(u_int32_t channel_to_send)
{
    switch (channel_to_send) {
        case (0):
            return std::make_pair<u_int32_t, u_int32_t>(1, 3);
            break;
        case (1):
            return std::make_pair<u_int32_t, u_int32_t>(0, 2);
            break;
        case (2):
            return std::make_pair<u_int32_t, u_int32_t>(1, 3);
            break;
        case (3):
            return std::make_pair<u_int32_t, u_int32_t>(0, 2);
            break;
        default:
            std::pair<u_int32_t, u_int32_t> same_channel(channel_to_send, channel_to_send);
            return same_channel;
            break;
    }
}

bool
SinkCell::recv_operon(Operon operon, u_int32_t direction_in, u_int32_t distance_class)
{
    return this->recv_channel_per_neighbor[direction_in][distance_class].push(operon);
}

// TODO: Implement fairness in sending. Use some counter on the iterator that starts with a
// different channel every cycle
void
SinkCell::prepare_a_cycle()
{

    if (!this->is_compute_cell_active()) {
        return;
    }
    /* std::cout << this->id << ": Sink Cell " << this->cooridates << "  prepare_a_cycle : " <<
       *this
              << "\n"; */
    // From the regular mesh recv channel to regular send channels
    // Move the operon from previous cycle recv channel to their destination: action queue or
    // send channel of a neighbor

    // Used for fairness. So that the channels don't get priority over others based on iterator
    // u_int32_t recv_channel_index = 0; // this->current_recv_channel_to_start_a_cycle;
    for (u_int32_t i = 0; i < this->recv_channel_per_neighbor.size(); i++) {

        for (u_int32_t j = 0; j < this->recv_channel_per_neighbor[i].size(); j++) {

            if (this->recv_channel_per_neighbor[i][j].size()) {

                if (j > this->hx + this->hy) {
                    std::cout << "SC : " << this->cooridates << " recv_channel_per_neighbor[" << i
                              << "][" << j
                              << "].size(): " << this->recv_channel_per_neighbor[i][j].size()
                              << "\n";
                }
                std::vector<Operon> recv_operons;
                while (this->recv_channel_per_neighbor[i][j].size()) {
                    recv_operons.push_back(this->recv_channel_per_neighbor[i][j].front());
                    this->recv_channel_per_neighbor[i][j].pop();
                }

                std::vector<Operon> left_over_operons;
                for (Operon operon : recv_operons) {

                    u_int32_t dst_cc_id = operon.first;

                    // Check if this operon is destined for this compute cell
                    // Bug check with assert: A SinkCell cannot invoke an action
                    assert(this->id != dst_cc_id);

                    // It means the operon needs to be sent/passed to some neighbor. Find whether it
                    // needs to be sent in the mesh network or second layer/htree network?

                    Coordinates dst_cc_coordinates =
                        Cell::cc_id_to_cooridinate(dst_cc_id, this->shape, this->dim_y);
                    /*
                    auto dst_compute_cell =
                    std::dynamic_pointer_cast<ComputeCell>(CCA_chip[dst_cc_id]);
                                    assert(dst_compute_cell != nullptr); */

                    if (this->check_cut_off_distance(dst_cc_coordinates)) {
                        // Pass it on within the mesh network since the destination is close by.

                        u_int32_t channel_to_send = get_route_towards_cc_id(dst_cc_id);

                        if (this->send_channel_per_neighbor[channel_to_send].push(operon)) {
                            // Set to distance class j + 1
                            this->send_channel_per_neighbor_current_distance_class
                                [channel_to_send] = j + 1;
                        } else {
                            // Increament the stall counter for send/recv
                            this->statistics.stall_network_on_send++;
                            left_over_operons.push_back(operon);
                        }

                    } else {
                        // Send to the second layer Htree network using the sink hole
                        // First form a CooridiantedOperon to send
                        CoordinatedOperon coordinated_operon(dst_cc_coordinates, operon);

                        if (!this->send_channel_to_htree_node.push(coordinated_operon)) {

                            // not sent in this cycle due to the send_channel being full
                            // Increament the stall counter for send/recv
                            this->statistics.stall_network_on_send++;
                            left_over_operons.push_back(operon);
                        }
                    }
                }
                for (Operon operon : left_over_operons) {
                    this->recv_channel_per_neighbor[i][j].push(operon);
                }
            }
        }

        // Pop all operons into a vector to avoid deadlock on the send_channel if it is full. In
        // case a send_channel is full: 1: then just push back that recv_operons vector 2: Check if
        // any of the remaining operons from the recv_operons can be sent to an asymetric neighbor?
        // 3: Finally push the remaining operons into the recv channel where they came from
        std::vector<Operon> recv_operons;
        while (this->recv_channel_to_htree_node.size()) {
            recv_operons.push_back(this->recv_channel_to_htree_node.front());
            this->recv_channel_to_htree_node.pop();
        }

        std::vector<Operon> left_over_operons;
        for (Operon operon : recv_operons) {

            u_int32_t dst_cc_id = operon.first;

            u_int32_t channel_to_send = get_route_towards_cc_id(dst_cc_id);

            if (this->send_channel_per_neighbor[channel_to_send].push(operon)) {
                // Set to distance class 0
                this->send_channel_per_neighbor_current_distance_class[channel_to_send] = 0;
            } else {
                // Increament the stall counter for send/recv
                this->statistics.stall_network_on_send++;

                // Put this back since it was not sent in this cycle due to the
                // send_channel_per_neighbor being full
                left_over_operons.push_back(operon);
            }
        }

        for (Operon operon : left_over_operons) {
            this->recv_channel_to_htree_node.push(operon);
        }
    }
}

void
SinkCell::run_a_computation_cycle(std::vector<std::shared_ptr<Cell>>& CCA_chip)
{
    if (!this->is_compute_cell_active()) {
        return;
    }

    /* std::cout << this->id << ": Sink Cell " << this->cooridates
              << "  run_a_computation_cycle : " << *this << "\n"; */

    // Initialize the counter for measuring resource usage and starvation. Start with all then
    // decreament as they are active. Later use that to find the percent active status for this
    // cycle. If nothing was decreamented it means that this cycle was totally inactive with the
    // CC starving.
    this->statistics.cycle_resource_use_counter =
        Cell::get_number_of_neighbors(this->shape) + 1; // +1 for 2nd layer

    // Apply the network operations from the previous cycle and prepare this cycle for
    // computation and communication
    this->prepare_a_cycle();

    // An SinkCell has nothing much do to here for computation
}

// This act as synchronization and needs to be called before the actual communication cycle so
// as to not cause race conditions on the communicaton buffer.
void
SinkCell::prepare_a_communication_cycle(std::vector<std::shared_ptr<Cell>>& CCA_chip)
{
    if (!this->is_compute_cell_active()) {
        return;
    }

    // Prepare the operons recieved from the htree end node and down and up CCs into the staging
    // buffer or the regular send channels of the CCA mesh

    // Pop all operons into a vector to avoid deadlock on the send_channel if it is full. In
    // case a send_channel is full: 1: then just push back that recv_operons vector 2: Check if
    // any of the remaining operons from the recv_operons can be sent to an asymetric neighbor?
    // 3: Finally push the remaining operons into the recv channel where they came from
    std::vector<Operon> recv_operons;
    while (this->recv_channel_to_htree_node.size()) {
        recv_operons.push_back(this->recv_channel_to_htree_node.front());
        this->recv_channel_to_htree_node.pop();
    }

    std::vector<Operon> left_over_operons;
    for (Operon operon : recv_operons) {

        u_int32_t dst_cc_id = operon.first;

        u_int32_t channel_to_send = get_route_towards_cc_id(dst_cc_id);

        if (!this->send_channel_per_neighbor[channel_to_send].push(operon)) {

            // Put this back since it was not sent in this cycle due to the
            // recv_channel_to_htree_node being full
            left_over_operons.push_back(operon);
            // this->recv_channel_to_htree_node.push(operon);

            // Increament the stall counter for send/recv
            this->statistics.stall_network_on_send++;
        } else {
            // Set to distance class 0 since this operon originates from this SinkCell
            this->send_channel_per_neighbor_current_distance_class[channel_to_send] = 0;
        }
    }

    for (Operon operon : left_over_operons) {
        this->recv_channel_to_htree_node.push(operon);
    }
}

void
SinkCell::run_a_communication_cycle(std::vector<std::shared_ptr<Cell>>& CCA_chip)
{

    if (!this->is_compute_cell_active()) {
        return;
    }
    // For shape square
    if (this->shape == computeCellShape::square) {

        // Send from send sink channel to the recv channel of the underlying htree end node
        while (this->send_channel_to_htree_node.size()) {

            CoordinatedOperon operon = this->send_channel_to_htree_node.front();

            if (this->connecting_htree_node->recv_channel_from_sink_cell->push(operon)) {
                this->send_channel_to_htree_node.pop();
            } else {
                // recv is full at the end node of the htree
                break;
            }
        }

        int receiving_direction[4] = { 2, 3, 0, 1 };

        for (u_int32_t i = 0; i < this->send_channel_per_neighbor.size(); i++) {

            if (this->send_channel_per_neighbor[i].size()) {

                // Update the cycle_resource_use_counter
                this->statistics.cycle_resource_use_counter--;

                std::vector<Operon> send_operons;
                while (this->send_channel_per_neighbor[i].size()) {
                    send_operons.push_back(this->send_channel_per_neighbor[i].front());
                    this->send_channel_per_neighbor[i].pop();
                }

                std::vector<Operon> left_over_operons;
                for (Operon operon : send_operons) {

                    u_int32_t dst_cc_id = operon.first;

                    // Check if this operon is destined for this compute/sink cell. If it does then
                    // it is a bug
                    assert(this->id != dst_cc_id);
                    // The neighbor of this compute cell cannot be null
                    assert(this->neighbor_compute_cells[i] != std::nullopt);

                    u_int32_t neighbor_id_ = this->neighbor_compute_cells[i].value().first;
                    if (!CCA_chip[neighbor_id_]->recv_operon(
                            operon, i, this->send_channel_per_neighbor_current_distance_class[i])) {
                        this->statistics.stall_network_on_recv++;
                        // increament the stall counter for send/recv
                        left_over_operons.push_back(operon);

                        /* std::cout << "SC : " << this->cooridates << " Not able to push on "
                                  << *CCA_chip[neighbor_id_] << " i = " << i << "\n"; */
                    }
                }
                for (Operon operon : left_over_operons) {
                    this->send_channel_per_neighbor[i].push(operon);
                }
            }
        }

        // TODO: Verify this
        // Since this is the end of the cycle find out how much percent of the CC was active and
        // whether it was inactive altogether?
        u_int32_t number_of_resources_per_cc = Cell::get_number_of_neighbors(this->shape) + 1;
        if (this->statistics.cycle_resource_use_counter == number_of_resources_per_cc) {
            this->statistics.cycles_inactive++;
        } else {
            this->statistics.cycles_resource_usage +=
                (number_of_resources_per_cc - this->statistics.cycle_resource_use_counter) /
                static_cast<long double>(number_of_resources_per_cc);
        }
    }
}

// Checks if the compute cell is active or not
bool
SinkCell::is_compute_cell_active()
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
                /* std::cout << "SC : " << this->cooridates << " recv_channel_per_neighbor[" << i
                          << "][" << j
                          << "].size(): " << this->recv_channel_per_neighbor[i][j].size() << "\n";
                 */
                recv_channels = true;
                break;
            }
        }
    }
    bool temp = (send_channels || recv_channels || this->send_channel_to_htree_node.size() ||
                 this->recv_channel_to_htree_node.size());
    /* if (temp) {
        std::cout << "SC : " << this->cooridates << " active = " << temp << "\n";
    } */
    return temp;
}

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

// TODO: Implement fairness in sending. Use some counter on the iterator that starts with a
// different channel every iteration
void
SinkCell::prepare_a_cycle()
{

    // Prepare the operons recieved from the htree end node

    // Pop all operons into a vector to avoid deadlock on the send_channel if it is full. In case a
    // send_channel is full then just push back that operon into the recv channel
    std::vector<Operon> recv_operons;
    while (this->recv_channel_to_htree_node.size()) {
        recv_operons.push_back(this->recv_channel_to_htree_node.front());
        this->recv_channel_to_htree_node.pop();
    }

    for (Operon operon : recv_operons) {
        /*         std::cout << this->id << ": Operon with cc id: " << operon.first
                          << " will be sent to sink cell depending on its recv queue\n"; */
        u_int32_t dst_cc_id = operon.first;

        u_int32_t channel_to_send = get_route_towards_cc_id(dst_cc_id);

        if (this->send_channel_per_neighbor[channel_to_send] == std::nullopt) {
            // Prepare the send channel
            this->send_channel_per_neighbor[channel_to_send] = operon;
            std::cout << this->id << ": SinkCell " << this->cooridates
                      << "  Sent to mesh! dst_cc_id: " << dst_cc_id << " coord: "
                      << Cell::cc_id_to_cooridinate(dst_cc_id, this->shape, this->dim_y) << "\n";

        } else {
            // Put this back since it was not sent in this cycle due to the
            // recv_channel_to_htree_node being full
            this->recv_channel_to_htree_node.push(operon);
            /*          std::cout << this->id
                               << ": SinkCell  recv_channel_to_htree_node.push dst_cc_id: " <<
               dst_cc_id
                               << "\n"; */

            // Increament the stall counter for send/recv
            this->statistics.stall_network_on_send++;
        }
    }

    // Move the operon from previous cycle recv channel to their destination: action queue or send
    // channel of a neighbor
    for (u_int32_t i = 0; i < this->recv_channel_per_neighbor.size(); i++) {
        if (this->recv_channel_per_neighbor[i]) {
            Operon operon_ = this->recv_channel_per_neighbor[i].value();
            u_int32_t dst_cc_id = operon_.first;

            // Check if this operon is destined for this compute cell
            // Bug check with assert: An SinkCell cannot invoke an action
            assert(this->id != dst_cc_id);

            // It means the operon needs to be sent/passed to some neighbor. Find whether it
            // needs to be sent in the mesh network or second layer/htree network?

            Coordinates dst_cc_coordinates =
                Cell::cc_id_to_cooridinate(dst_cc_id, this->shape, this->dim_y);

            if (this->check_cut_off_distance(dst_cc_coordinates)) {
                // Pass it on within the mesh network since the destination is close by.

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

            } else {
                // Send to the second layer Htree network using the sink hole

                // First form a CooridiantedOperon to send
                CoordinatedOperon coordinated_operon(dst_cc_coordinates, operon_);

                if (this->send_channel_to_htree_node.push(coordinated_operon)) {
                    // Flush the recv channel buffer
                    this->recv_channel_per_neighbor[i] = std::nullopt;
                } else {
                    // not sent in this cycle due to the send_channel being full
                    // Increament the stall counter for send/recv
                    this->statistics.stall_network_on_send++;
                }
            }
        }
    }
}

void
SinkCell::run_a_computation_cycle()
{
    // Initialize the counter for measuring resource usage and starvation. Start with all then
    // decreament as they are active. Later use that to find the percent active status for this
    // cycle. If nothing was decreamented it means that this cycle was totally inactive with the CC
    // starving.
    this->statistics.cycle_resource_use_counter =
        Cell::get_number_of_neighbors(this->shape) + 1; // +1 for 2nd layer

    // Apply the network operations from the previous cycle and prepare this cycle for computation
    // and communication
    this->prepare_a_cycle();

    // An SinkCell has nothing much do to here for computation
}

// This act as synchronization and needs to be called before the actual communication cycle so as to
// not cause race conditions on the communicaton buffer. It also applies to the ... TODO fix comment
void
SinkCell::prepare_a_communication_cycle()
{
    // Htree has no Action creation from logic and therefore this function will be empty
}

void
SinkCell::run_a_communication_cycle(std::vector<std::shared_ptr<Cell>>& CCA_chip)
{
    // For shape square
    if (this->shape == computeCellShape::square) {
        int receiving_direction[4] = { 2, 3, 0, 1 };

        for (u_int32_t i = 0; i < this->send_channel_per_neighbor.size(); i++) {

            if (this->send_channel_per_neighbor[i]) { // is not std::nullopt

                // Update the cycle_resource_use_counter
                this->statistics.cycle_resource_use_counter--;

                Operon operon_ = this->send_channel_per_neighbor[i].value();
                u_int32_t dst_cc_id = operon_.first;

                // Check if this operon is destined for this compute cell. If it does then it is a
                // bug
                assert(this->id != dst_cc_id);

                // The neighbor of this compute cell cannot be null
                assert(this->neighbor_compute_cells[i] != std::nullopt);

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

// Checks if the compute cell is active or not
bool
SinkCell::is_compute_cell_active()
{
    bool send_channels = false;
    bool recv_channels = false;
    for (u_int32_t i = 0; i < this->number_of_neighbors; i++) {
        if (this->send_channel_per_neighbor[i]) {
            send_channels = true;
            break;
        }
        if (this->recv_channel_per_neighbor[i]) {
            recv_channels = true;
            break;
        }
    }

    return (send_channels || recv_channels || this->send_channel_to_htree_node.size() ||
            this->recv_channel_to_htree_node.size());
}

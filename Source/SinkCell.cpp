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
#include "Routing.hpp"

#include <cassert>

// For memcpy()
#include <cstring>

// For assert
#include <cassert>

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

// TODO: Implement fairness in sending. Use some counter on the iterator that starts with a
// different channel every cycle
void
SinkCell::prepare_a_cycle(std::vector<std::shared_ptr<Cell>>& CCA_chip)
{

    if (!this->is_compute_cell_active()) {
        return;
    }
    /* std::cout << this->id << ": Sink Cell " << this->cooridates << "  prepare_a_cycle : " <<
       *this
              << "\n"; */

    // Pop all operons into a vector to avoid deadlock on the send_channel if it is full. In
    // case a send_channel is full: 1: then just push back that recv_operons vector 2: Check if
    // any of the remaining operons from the recv_operons can be sent to an asymetric neighbor?
    // (Not Implemented yet) 3: Finally push the remaining operons into the recv channel where
    // they came from
    std::vector<Operon> recv_operons;
    while (this->recv_channel_to_htree_node.size()) {
        recv_operons.push_back(this->recv_channel_to_htree_node.front());
        this->recv_channel_to_htree_node.pop();
    }

    std::vector<Operon> left_over_operons;
    for (Operon operon : recv_operons) {

        u_int32_t dst_cc_id = operon.first.dst_cc_id;

        // Since the operon has come from the Htree change its src id to the sink cell.
        // This is to make sure that the static routing algorithm of routing policy = 1 works
        if (this->mesh_routing_policy == 1) {
            /*   std::cout << "SC: " << this->id << " operon old src: " << operon.first.src_cc_id
                        << "\n"; */
            operon.first.src_cc_id = this->id;
        }

        std::vector<u_int32_t> channels_to_send =
            this->get_route_towards_cc_id(operon.first.src_cc_id, dst_cc_id);

        bool pushed = false;
        for (auto channel_to_send : channels_to_send) {
            if (this->send_channel_per_neighbor[channel_to_send].push(operon)) {

                // Set to distance class 0
                this->send_channel_per_neighbor_current_distance_class[channel_to_send] = 0;

                // Break out of the for loop. Discard other paths.
                pushed = true;
                break;
            }
        }

        if (!pushed) {
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

    // From the regular mesh recv channel to regular send channels
    // Move the operon from previous cycle recv channel to their destination: action queue or
    // send channel of a neighbor

    // Used for fairness. So that the channels don't get priority over others based on iterator
    // u_int32_t recv_channel_index = 0; // this->current_recv_channel_to_start_a_cycle;
    for (u_int32_t i = 0; i < this->recv_channel_per_neighbor.size(); i++) {

        for (u_int32_t j = 0; j < this->recv_channel_per_neighbor[i].size(); j++) {

            if (this->recv_channel_per_neighbor[i][j].size()) {

                // If this is greater them it is a bug
                assert(j <= this->distance_class_length);

                std::vector<Operon> recv_operons;
                while (this->recv_channel_per_neighbor[i][j].size()) {
                    recv_operons.push_back(this->recv_channel_per_neighbor[i][j].front());
                    this->recv_channel_per_neighbor[i][j].pop();
                }

                std::vector<Operon> left_over_operons;
                for (Operon operon : recv_operons) {

                    u_int32_t dst_cc_id = operon.first.dst_cc_id;

                    // Check if this operon is destined for this compute cell
                    // Bug check with assert: A SinkCell cannot invoke an action
                    assert(this->id != dst_cc_id);

                    // It means the operon needs to be sent/passed to some neighbor. Find whether it
                    // needs to be sent in the mesh network or second layer/htree network?

                    // Get the route using Routing 0
                    std::optional<u_int32_t> routing_cell_id = Routing::get_next_move<SinkCell>(
                        CCA_chip, operon, this->id, this->mesh_routing_policy);

                    if (routing_cell_id != std::nullopt) {
                        // Pass it on within the mesh network since the destination is close by.

                        std::vector<u_int32_t> channels_to_send = this->get_route_towards_cc_id(
                            operon.first.src_cc_id, routing_cell_id.value());

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
                            // Increament the stall counter for send/recv
                            this->statistics.stall_network_on_send++;
                            left_over_operons.push_back(operon);
                        }

                    } else {

                        Coordinates dst_cc_coordinates =
                            Cell::cc_id_to_cooridinate(dst_cc_id, this->shape, this->dim_y);
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
    }
}

void
SinkCell::run_a_computation_cycle(std::vector<std::shared_ptr<Cell>>& CCA_chip,
                                  void* function_events)
{
    // assert(CCA_chip.size() != 0);

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
    this->prepare_a_cycle(CCA_chip);

    // A SinkCell has nothing much do to here for computation
}

// This act as synchronization and needs to be called before the actual communication cycle so
// as to not cause race conditions on the communicaton buffer.
void
SinkCell::prepare_a_communication_cycle(std::vector<std::shared_ptr<Cell>>& CCA_chip)
{
    // assert(CCA_chip.size() != 0);

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

        u_int32_t dst_cc_id = operon.first.dst_cc_id;

        // Since the operon has come from the Htree change its src id to the sink cell.
        // This is to make sure that the static routing algorithm of routing policy = 1 works
        if (this->mesh_routing_policy == 1) {
            /*   std::cout << "SC: " << this->id << " operon old src: " << operon.first.src_cc_id
                        << "\n"; */
            operon.first.src_cc_id = this->id;
        }

        std::vector<u_int32_t> channels_to_send =
            this->get_route_towards_cc_id(operon.first.src_cc_id, dst_cc_id);

        bool pushed = false;
        for (auto channel_to_send : channels_to_send) {
            if (this->send_channel_per_neighbor[channel_to_send].push(operon)) {

                // Set to distance class 0 since this operon originates from this SinkCell
                this->send_channel_per_neighbor_current_distance_class[channel_to_send] = 0;

                // Break out of the for loop. Discard other paths.
                pushed = true;
                break;
            }
        }

        if (!pushed) {
            // Increament the stall counter for send/recv
            // Put this back since it was not sent in this cycle due to the
            // recv_channel_to_htree_node being full
            left_over_operons.push_back(operon);
            // this->recv_channel_to_htree_node.push(operon);

            // Increament the stall counter for send/recv
            this->statistics.stall_network_on_send++;
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

                    u_int32_t dst_cc_id = operon.first.dst_cc_id;

                    // Check if this operon is destined for this compute/sink cell. If it does then
                    // it is a bug
                    assert(this->id != dst_cc_id);
                    // The neighbor of this compute cell cannot be null
                    assert(this->neighbor_compute_cells[i] != std::nullopt);

                    u_int32_t neighbor_id_ = this->neighbor_compute_cells[i].value().first;
                    if (!CCA_chip[neighbor_id_]->recv_operon(
                            operon,
                            receiving_direction[i],
                            this->send_channel_per_neighbor_current_distance_class[i])) {
                        this->statistics.stall_network_on_recv++;
                        // increament the stall counter for send/recv
                        left_over_operons.push_back(operon);

                        /* std::cout << "SC : " << this->cooridates << " Not able to push on "
                                  << *CCA_chip[neighbor_id_] << " i = " << i << " distance class = "
                                  << this->send_channel_per_neighbor_current_distance_class[i]
                                  << "\n"; */
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
u_int32_t
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
                recv_channels = true;
                break;
            }
        }
    }
    if (send_channels || recv_channels || this->send_channel_to_htree_node.size() ||
        this->recv_channel_to_htree_node.size()) {
        // Only communication active
        return 1;
    }
    // Inactive
    return 0;
}

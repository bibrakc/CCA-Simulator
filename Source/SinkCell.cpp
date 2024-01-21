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
#include "HtreeNode.hpp"
#include "Routing.hpp"

auto
return_asymetric_neighbors(u_int32_t channel_to_send) -> std::pair<u_int32_t, u_int32_t>
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

    // From the regular mesh recv channel to regular send channels
    // Move the operon from previous cycle recv channel to their destination: action queue or
    // send channel of a neighbor

    // Used for fairness. So that the channels don't get priority over others based on iterator
    u_int32_t recv_channel_index = this->current_recv_channel_to_start_a_cycle;

    for (u_int32_t i = 0; i < this->recv_channel_per_neighbor.size(); i++) {
        // Distance Class
        for (int j = this->recv_channel_per_neighbor[recv_channel_index].size() - 1; j >= 0; j--) {
            // for (u_int32_t j = 0; j < this->recv_channel_per_neighbor[i].size(); j++) {

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

                    // Check if this operon is destined for this compute cell
                    // Bug check with assert: A SinkCell cannot invoke an action
                    assert(this->id != dst_cc_id);

                    // It means the operon needs to be sent/passed to some neighbor. Find whether it
                    // needs to be sent in the mesh network or second layer/htree network?

                    // Get the route using Routing `mesh_routing_policy`
                    std::optional<u_int32_t> routing_cell_id = Routing::get_next_move<SinkCell>(
                        CCA_chip, operon, this->id, this->mesh_routing_policy);

                    if (routing_cell_id != std::nullopt) {
                        // Pass it on within the mesh network since the destination is close by.

                        std::vector<u_int32_t> const channels_to_send =
                            this->get_route_towards_cc_id(operon.first.src_cc_id,
                                                          routing_cell_id.value());

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

                    } else {

                        Coordinates const dst_cc_coordinates =
                            Cell::cc_id_to_cooridinate(dst_cc_id, this->shape, this->dim_y);
                        // Send to the second layer Htree network using the sink hole
                        // First form a CooridiantedOperon to send
                        CoordinatedOperon const coordinated_operon(dst_cc_coordinates, operon);

                        operon_was_inserted_or_sent =
                            this->send_channel_to_htree_node.push(coordinated_operon);
                    }

                    // not sent in this cycle due to the send_channel being full
                    if (!operon_was_inserted_or_sent) {
                        left_over_operons.push_back(operon);
                    }
                }

                for (Operon const& operon : left_over_operons) {
                    if (!this->recv_channel_per_neighbor[recv_channel_index][j].push(operon)) {
                        std::cerr << "SinkCell push on recv_channel_per_neighbor failed. Perhaps a "
                                     "bug.\n";
                    }
                }
            }
        }
        recv_channel_index = (recv_channel_index + 1) % this->number_of_neighbors;
    }
    // Update the index of the starting channel for the next cycle.
    this->current_recv_channel_to_start_a_cycle =
        (this->current_recv_channel_to_start_a_cycle + 1) % this->number_of_neighbors;

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

        u_int32_t const dst_cc_id = operon.first.dst_cc_id;

        // Since the operon has come from the Htree change its src id to the sink cell.
        // This is to make sure that the static routing algorithm of routing policy = 1 works
        if (this->mesh_routing_policy == 1) {
            /*   std::cout << "SC: " << this->id << " operon old src: " << operon.first.src_cc_id
                        << "\n"; */
            operon.first.src_cc_id = this->id;
        }

        std::vector<u_int32_t> const channels_to_send =
            this->get_route_towards_cc_id(operon.first.src_cc_id, dst_cc_id);

        // Always use the default virtual channel 0 as this is the begining of the journey for
        // this operon when it came out of the low latency network h-tree. It shouldn't matter for
        // deadlocks maybe.
        u_int32_t virtual_channel_to_use = 0;

        bool pushed = false;
        for (auto channel_to_send : channels_to_send) {
            if (this->send_channel_per_neighbor[channel_to_send][virtual_channel_to_use].push(
                    operon)) {

                // Set to distance class 0
                // this->send_channel_per_neighbor_current_distance_class[channel_to_send] = 0;

                // Break out of the for loop. Discard other paths.
                pushed = true;
                break;
            }
        }

        if (!pushed) {
            // Put this back since it was not sent in this cycle due to the
            // send_channel_per_neighbor being full
            left_over_operons.push_back(operon);
        }
    }

    for (Operon const& operon : left_over_operons) {
        if (!this->recv_channel_to_htree_node.push(operon)) {
            std::cerr << "SinkCell push on recv_channel_to_htree_node failed. Perhaps a "
                         "bug.\n";
        }
    }
}

void
SinkCell::run_a_computation_cycle(std::vector<std::shared_ptr<Cell>>& CCA_chip,
                                  void* /*function_events*/)
{
    if (!this->is_compute_cell_active()) {
        return;
    }

    // Apply the network operations from the previous cycle and prepare this cycle for
    // computation and communication
    this->prepare_a_cycle(CCA_chip);

    // A SinkCell has nothing much do to here for computation
}

// This act as synchronization and needs to be called before the actual communication cycle so
// as to not cause race conditions on the communicaton buffer.
void
SinkCell::prepare_a_communication_cycle(std::vector<std::shared_ptr<Cell>>& /*CCA_chip*/)
{
    // assert(CCA_chip.size() != 0);

    if (!this->is_compute_cell_active()) {
        return;
    }

    // Prepare the operons recieved from the htree end node and down and up CCs into the staging
    // buffer or the regular send channels of the CCA mesh

    // Note: Not all points in this comment maybe implemented.
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

        u_int32_t const dst_cc_id = operon.first.dst_cc_id;

        // Since the operon has come from the Htree change its src id to the sink cell.
        // This is to make sure that the static routing algorithm of routing policy = 1 works
        if (this->mesh_routing_policy == 1) {
            /*   std::cout << "SC: " << this->id << " operon old src: " << operon.first.src_cc_id
                        << "\n"; */
            operon.first.src_cc_id = this->id;
        }

        std::vector<u_int32_t> const channels_to_send =
            this->get_route_towards_cc_id(operon.first.src_cc_id, dst_cc_id);

        // Always use the default virtual channel 0 as this is the begining of the journey for
        // this operon when it came out of the low latency network h-tree. It shouldn't matter for
        // deadlocks maybe.
        u_int32_t virtual_channel_to_use = 0;

        bool pushed = false;
        for (auto channel_to_send : channels_to_send) {
            if (this->send_channel_per_neighbor[channel_to_send][virtual_channel_to_use].push(
                    operon)) {

                // Set to distance class 0 since this operon originates from this SinkCell
                // this->send_channel_per_neighbor_current_distance_class[channel_to_send] = 0;

                // Break out of the for loop. Discard other paths.
                pushed = true;
                break;
            }
        }

        if (!pushed) {

            // Put this back since it was not sent in this cycle due to the
            // recv_channel_to_htree_node being full
            left_over_operons.push_back(operon);
            // this->recv_channel_to_htree_node.push(operon);
        }
    }

    for (Operon const& operon : left_over_operons) {
        if (!this->recv_channel_to_htree_node.push(operon)) {
            std::cerr << "SinkCell: recv_channel_to_htree_node failed on push. Perhaps a bug.";
        }
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

            CoordinatedOperon const operon = this->send_channel_to_htree_node.front();

            if (this->connecting_htree_node->recv_channel_from_sink_cell->push(operon)) {
                this->send_channel_to_htree_node.pop();
            } else {
                // recv is full at the end node of the htree
                break;
            }
        }

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

                        // Check if this operon is destined for this compute/sink cell. If it does
                        // then it is a bug
                        assert(this->id != dst_cc_id);
                        // The neighbor of this compute cell cannot be null
                        assert(this->neighbor_compute_cells[i] != std::nullopt);

                        u_int32_t const neighbor_id_ =
                            this->neighbor_compute_cells[i].value().first;
                        if (!CCA_chip[neighbor_id_]->recv_operon(
                                operon, receiving_direction[i], virtual_channel_index)) {

                            this->send_channel_per_neighbor_contention_count[i].increment();
                            left_over_operons.push_back(operon);

                            /* std::cout << "SC : " << this->cooridates << " Not able to push on "
                                      << *CCA_chip[neighbor_id_] << " i = " << i << " distance class
                               = "
                                      << this->send_channel_per_neighbor_current_distance_class[i]
                                      << "\n"; */
                        } else {
                            this->send_channel_per_neighbor_contention_count[i].reset();
                            this->statistics.operons_moved++;
                        }
                    }
                    for (Operon const& operon : left_over_operons) {
                        if (!this->send_channel_per_neighbor[i][virtual_channel_index].push(
                                operon)) {
                            std::cerr << "SinkCell: send_channel_per_neighbor failed on push. "
                                         "Perhaps a bug.";
                        }
                    }
                }
            }
        }
    }
}

// Checks if the compute cell is active or not
auto
SinkCell::is_compute_cell_active() -> u_int32_t
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
    constexpr u_int32_t communication_congested_status = 4;

    if (send_channels || recv_channels || this->send_channel_to_htree_node.size() ||
        this->recv_channel_to_htree_node.size()) {
        // Only communication active
        if (is_congested) {
            return (communication_congested_status + congestion_level_addition);
        }
        return communication_status;
    }
    // Inactive
    return inactive_status;
}

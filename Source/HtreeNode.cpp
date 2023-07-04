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

#include "HtreeNode.hpp"

// Overload printing for HtreeNode
auto
HtreeNode::operator<<(std::ostream& os) -> std::ostream&
{

    if (this->in_first == nullptr && this->in_second == nullptr) {
        os << "[" << this->id << " -> " << this->cooridinates
           << " {Coverage 1: " << this->coverage_top_left
           << ", Coverage 2: " << this->coverage_bottom_right << "}]\n";
    } else {
        os << this->id << " {Coverage 1: " << this->coverage_top_left
           << ", Coverage 2: " << this->coverage_bottom_right << "}\n";
    }
    return os;
}

// TODO: see if we keep this globally here or use the HtreeNode function?
auto
is_coordinate_in_a_particular_range(const Coordinates start,
                                    const Coordinates end,
                                    const Coordinates point) -> bool
{
    // Check if the point is within the range
    return ((point.first >= start.first) && (point.first <= end.first) &&
            (point.second >= start.second) && (point.second <= end.second));
}

auto
HtreeNode::put_operon_from_sink_cell(const CoordinatedOperon& operon) -> bool
{
    return this->recv_channel_from_sink_cell->push(operon);
}

auto
HtreeNode::is_coordinate_in_my_range(const Coordinates point) -> bool
{
    // Check if the point is within the range
    return ((point.first >= this->coverage_top_left.first) &&
            (point.first <= this->coverage_bottom_right.first) &&
            (point.second >= this->coverage_top_left.second) &&
            (point.second <= this->coverage_bottom_right.second));
}

auto
HtreeNode::is_end_htree_node() -> bool
{
    return (!this->in_first && !this->in_second);
}

auto
HtreeNode::is_htree_node_active() -> bool
{
    bool send_channels_active = false;
    bool recv_channels_active = false;
    for (int i = 0; i < 4; i++) {
        if (this->send_channel[i]) {
            if (this->send_channel[i].value()->size() != 0) {
                send_channels_active = true;
                break;
            }
        }
        if (this->recv_channel[i]) {
            if (this->recv_channel[i].value()->size() != 0) {
                recv_channels_active = true;
                break;
            }
        }
    }

    bool send_sink_active = false;
    bool recv_sink_active = false;
    if (this->is_end_htree_node()) {

        if (this->send_channel_to_sink_cell->size() != 0) {
            send_sink_active = true;
        }
        if (this->recv_channel_from_sink_cell->size() != 0) {
            recv_sink_active = true;
        }
    }

    return (send_channels_active || recv_channels_active || send_sink_active || recv_sink_active);
}

void
HtreeNode::transfer(const std::shared_ptr<FixedSizeQueue<CoordinatedOperon>>& recv,
                    std::optional<std::shared_ptr<FixedSizeQueue<CoordinatedOperon>>> send,
                    const CoordinatedOperon& operon)
{
    if (send == std::nullopt) {
        std::cerr << this->id << ": Bug! transfer: send_channel cannot be null\n";
        exit(0);
    }

    // std::cout << this->cooridinates << ": HtreeNode transfer\n";

    std::shared_ptr<FixedSizeQueue<CoordinatedOperon>> const current_send_channel = send.value();

    if (!current_send_channel->push(operon)) {

        // Put this back since it was not sent in this cycle due to the send_channel
        // being full
        recv->push(operon);
    }
}

void
HtreeNode::transfer_send_to_recv(
    std::optional<std::shared_ptr<FixedSizeQueue<CoordinatedOperon>>> send,
    std::optional<std::shared_ptr<FixedSizeQueue<CoordinatedOperon>>> recv)
{
    // std::cout << this->cooridinates << ": HtreeNode transfer_send_to_recv\n";

    if (send == std::nullopt) {
        std::cerr << this->id << ": Bug! transfer_send_to_recv: send_channel cannot be null\n";
        exit(0);
    }
    if (recv == std::nullopt) {
        std::cerr << this->id << ": Bug! transfer_send_to_recv: recv_channel cannot be null\n";
        exit(0);
    }

    while (send.value()->size()) {

        CoordinatedOperon const operon = send.value()->front();
        if (recv.value()->push(operon)) {
            send.value()->pop();
        } else {
            // recv is full
            break;
        }
    }
}

void
HtreeNode::shift_from_a_single_recv_channel_to_send_channels(
    const std::shared_ptr<FixedSizeQueue<CoordinatedOperon>>& recv,
    std::optional<std::shared_ptr<FixedSizeQueue<CoordinatedOperon>>> send[])
{

    // Pop all operons into a vector to avoid deadlock on a send_channel if it is full.
    // In case a send_channel is full then just push back that operon into the recv channel
    std::vector<CoordinatedOperon> recv_operons;
    while (recv->size()) {
        recv_operons.push_back(recv->front());
        recv->pop();
    }

    for (CoordinatedOperon const& operon : recv_operons) {

        Coordinates const destination_cc_coorinates = operon.first;

        // Find route.

        // First check if this is the end htree node
        if (this->is_end_htree_node()) {
            // Does the route needs to go thought the sink channel?
            if (this->is_coordinate_in_my_range(destination_cc_coorinates)) {

                Operon const simple_operon = operon.second;
                if (!this->send_channel_to_sink_cell->push(simple_operon)) {

                    // Put this back since it was not sent in this cycle due to the send_channel
                    // being full
                    recv->push(operon);
                }

            } else {
                // Send the operon out from this end node
                transfer(recv, send[this->local_index_send_channel_out], operon);
            }
            // Check if it can go to `in_first`?
        } else if (is_coordinate_in_a_particular_range(this->in_first->coverage_top_left,
                                                       this->in_first->coverage_bottom_right,
                                                       destination_cc_coorinates)) {

            // Send to in_first

            transfer(recv, send[this->local_index_send_channel_in_first], operon);

        } else if (is_coordinate_in_a_particular_range(this->in_second->coverage_top_left,
                                                       this->in_second->coverage_bottom_right,
                                                       destination_cc_coorinates)) {

            // Send to in_second
            transfer(recv, send[this->local_index_send_channel_in_second], operon);
        } else {

            // Send to out
            transfer(recv, send[this->local_index_send_channel_out], operon);
        }
    }
}

void
HtreeNode::prepare_communication_cycle()
{

    if (!this->is_htree_node_active()) {
        return;
    }

    // Shift from recv_channel queues to send_channel queues

    // First recv from sink cell
    if (this->is_end_htree_node()) {
        shift_from_a_single_recv_channel_to_send_channels(this->recv_channel_from_sink_cell,
                                                          this->send_channel);
    }

    // Then from in and out recv channels
    u_int32_t recv_channel_index = this->current_recv_channel_to_start_a_cycle;
    for (int i = 0; i < 4; i++) {

        if (this->recv_channel[recv_channel_index] != std::nullopt) {
            shift_from_a_single_recv_channel_to_send_channels(
                this->recv_channel[recv_channel_index].value(), this->send_channel);
        }

        recv_channel_index = (recv_channel_index + 1) % 4;
    }

    this->current_recv_channel_to_start_a_cycle =
        (this->current_recv_channel_to_start_a_cycle + 1) % 4;
}

void
HtreeNode::run_a_communication_cylce()
{
    if (!this->is_htree_node_active()) {
        return;
    }

    // Send from `send_*` queues to remote `recv_*` queues

    // First send from end node to sink cell
    if (this->is_end_htree_node()) {

        // Pop all operons into a vector to avoid deadlock on the recv_channel at the remote
        // destination if it is full. In case a recv_channel is full then just push back that operon
        // into the send channel
        std::vector<Operon> send_operons;
        while (this->send_channel_to_sink_cell->size()) {
            send_operons.push_back(this->send_channel_to_sink_cell->front());
            this->send_channel_to_sink_cell->pop();
        }

        for (Operon const& operon : send_operons) {

            /* std::cout
                << this->id << ": HtreeNode. Operon for cc: " << operon.first.dst_cc_id
                << " will be sent to sink cell depending on its recv queue. Coord of sink cell: \n";
             */

            if (!this->sink_cell_connector.value()->recv_channel_to_htree_node.push(operon)) {

                // std::cout << this->id << ": HtreeNode. Not able to push in sinkcell recv\n";

                // Put this back since it was not sent in this cycle due to the
                // recv_channel_to_htree_node being full
                this->send_channel_to_sink_cell->push(operon);
            }
        }
    }

    if (!this->is_end_htree_node()) { // End nodes dont have in_first and in_second
        // Then send from in_first
        transfer_send_to_recv(
            this->send_channel[this->local_index_send_channel_in_first],
            this->in_first->recv_channel[this->remote_index_recv_channel_in_first]);

        // Then send from in_second
        transfer_send_to_recv(
            this->send_channel[this->local_index_send_channel_in_second],
            this->in_second->recv_channel[this->remote_index_recv_channel_in_second]);
    }

    // Then send from out
    transfer_send_to_recv(this->send_channel[this->local_index_send_channel_out],
                          this->out->recv_channel[this->remote_index_recv_channel_out]);
}

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

#ifndef HTREE_NODE_HPP
#define HTREE_NODE_HPP

#include "SinkCell.hpp"

// H-Tree Node
struct HtreeNode
{

    auto operator<<(std::ostream& os) -> std::ostream&;

    auto put_operon_from_sink_cell(const CoordinatedOperon& operon) -> bool;
    auto is_coordinate_in_my_range(Coordinates point) -> bool;

    auto is_end_htree_node() -> bool;

    auto is_htree_node_active() -> bool;

    void transfer(const std::shared_ptr<FixedSizeQueue<CoordinatedOperon>>& recv,
                  std::optional<std::shared_ptr<FixedSizeQueue<CoordinatedOperon>>> send,
                  const CoordinatedOperon& operon);

    void transfer_send_to_recv(
        std::optional<std::shared_ptr<FixedSizeQueue<CoordinatedOperon>>> send,
        std::optional<std::shared_ptr<FixedSizeQueue<CoordinatedOperon>>> recv);

    void shift_from_a_single_recv_channel_to_send_channels(
        const std::shared_ptr<FixedSizeQueue<CoordinatedOperon>>& recv,
        std::optional<std::shared_ptr<FixedSizeQueue<CoordinatedOperon>>> send[]);

    void prepare_communication_cycle();

    void run_a_communication_cylce();

    int id;
    Coordinates cooridinates;

    std::shared_ptr<HtreeNode> in_first;  // up or left
    std::shared_ptr<HtreeNode> in_second; // down or right
    std::shared_ptr<HtreeNode> out;       // outside of this htree

    // How to know if this node is up, down, left, or right of its neighbor node?
    // The neighbor node itself will set this to either 0, 1, 2 or 3.
    // 0: I am your left neighbor, which means that when you output operons to me please put
    // in my `recv_in_channel[0]`.
    // 1: Likewie this is up, meaning out in my `recv_in_channel[1]`
    // 2: right `recv_in_channel[2]`
    // 3: down `recv_in_channel[3]`
    u_int32_t remote_index_recv_channel_out;

    u_int32_t remote_index_recv_channel_in_first;
    u_int32_t remote_index_recv_channel_in_second;

    u_int32_t local_index_send_channel_out;

    u_int32_t local_index_send_channel_in_first;
    u_int32_t local_index_send_channel_in_second;

    int in_bandwidth;  // Number of channels per in lane
    int out_bandwidth; // Number of channels per out lane

    Coordinates coverage_top_left;
    Coordinates coverage_bottom_right;

    // ID of the sink cell that this HtreeNode connects to. Only if it is an end htree node.
    std::optional<std::shared_ptr<SinkCell>> sink_cell_connector;

    // Only to be used for end htree nodes that connect with the CCA chip through sink cells
    std::shared_ptr<FixedSizeQueue<Operon>> send_channel_to_sink_cell;
    std::shared_ptr<FixedSizeQueue<CoordinatedOperon>> recv_channel_from_sink_cell;

    std::optional<std::shared_ptr<FixedSizeQueue<CoordinatedOperon>>> send_channel[4];
    std::optional<std::shared_ptr<FixedSizeQueue<CoordinatedOperon>>> recv_channel[4];

    // This is used to be fair in routing. When we start a communication cycle from the same
    // recv_channel what will happen is that that channel will get priority in sending its operons
    // at expense of thoer channels/neighbors. We want to start every cycle with a different
    // starting recv_channel and then alternate between them. This will provide fairness and not
    // cause congestion at any one link.
    u_int32_t current_recv_channel_to_start_a_cycle{};

    HtreeNode(int index, int x, int y, int in_bandhwidth_in, int out_bandhwidth_in)
    {
        this->id = index;
        this->cooridinates.first = x;
        this->cooridinates.second = y;

        this->in_first = nullptr;
        this->in_second = nullptr;
        this->out = nullptr;

        this->in_bandwidth = in_bandhwidth_in;
        this->out_bandwidth = out_bandhwidth_in;

        // Later for each end node htree node that connects with a sink cell in the CCA chip these
        // will be assigned their proper values
        this->sink_cell_connector = std::nullopt;

        this->send_channel_to_sink_cell = nullptr;
        this->recv_channel_from_sink_cell = nullptr;

        // This means that it is an end htree node that is connected to a sink cell in the CCA chip
        if (in_bandhwidth_in == 0) {

            // End node has 4 links to the square type shape sink cell
            this->send_channel_to_sink_cell =
                std::make_shared<FixedSizeQueue<Operon>>(out_bandhwidth_in);
            this->recv_channel_from_sink_cell =
                std::make_shared<FixedSizeQueue<CoordinatedOperon>>(out_bandhwidth_in);
        }

        for (int i = 0; i < 4; i++) {
            this->recv_channel[i] = std::nullopt;
            this->send_channel[i] = std::nullopt;
        }

        // Start from 0th and then alternate by % 4
        this->current_recv_channel_to_start_a_cycle = 0;
    }
};

#endif // HTREE_NODE_HPP

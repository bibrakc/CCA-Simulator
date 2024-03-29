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

#ifndef SINK_CELL_HPP
#define SINK_CELL_HPP

#include "Cell.hpp"

#include <cassert>
#include <utility>

// How many operons can the sink cell receive and send in a single cycle.
inline u_int32_t constexpr sink_cell_bandwidth = 4;

// Forward declaring to solve the circular dependency.
struct HtreeNode;

class SinkCell : public Cell
{
  public:
    // Prepare the cycle. This involves moving operon data into either the action queue or send
    // buffers of the network links
    void prepare_a_cycle(std::vector<std::shared_ptr<Cell>>& CCA_chip);

    // Execute a single cycle for this cell
    void run_a_computation_cycle(std::vector<std::shared_ptr<Cell>>& CCA_chip,
                                 void* function_events) override;

    // TODO: write comments
    void prepare_a_communication_cycle(std::vector<std::shared_ptr<Cell>>& CCA_chip) override;

    // TODO: write comments
    void run_a_communication_cycle(std::vector<std::shared_ptr<Cell>>& CCA_chip) override;

    // Checks if the cell is active or not
    auto is_compute_cell_active() -> u_int32_t override;

    // This is the id of the Htree node in the second layer network. Think of this as connecting in
    // the 3rd dimension under the chip using TSA (Through Silicon Via)
    std::shared_ptr<HtreeNode> connecting_htree_node;

    // Send channel/link toward the second layer Htree. The width of this channel is based on the
    // shape of the Cells. For an square it is 4, meaning 4 Operons can be sent in a single cycle.
    FixedSizeQueue<CoordinatedOperon> send_channel_to_htree_node;

    // Same as the base class `Cell` this is needed to satisty simulation. Read the comment for
    // `recv_channel_per_neighbor` in the base class `Cell`
    FixedSizeQueue<Operon> recv_channel_to_htree_node;

    // Constructor
    SinkCell(u_int32_t id_in,
             CellType type_in,
             std::shared_ptr<HtreeNode> connecting_htree_node_in,
             computeCellShape shape_in,
             u_int32_t dim_x_in,
             u_int32_t dim_y_in,
             u_int32_t hx_in,
             u_int32_t hy_in,
             u_int32_t hdepth_in,
             u_int32_t primary_network_type_in,
             u_int32_t mesh_routing_policy_id_in)
        : send_channel_to_htree_node(FixedSizeQueue<CoordinatedOperon>(sink_cell_bandwidth))
        , recv_channel_to_htree_node(FixedSizeQueue<Operon>(sink_cell_bandwidth))

    {
        this->id = id_in;
        this->type = type_in;
        this->statistics.type = this->type;

        this->connecting_htree_node = std::move(connecting_htree_node_in);

        this->shape = shape_in;
        this->number_of_neighbors = Cell::get_number_of_neighbors(this->shape);

        this->dim_x = dim_x_in;
        this->dim_y = dim_y_in;

        this->hx = hx_in;
        this->hy = hy_in;
        this->hdepth = hdepth_in;

        this->cooridates = Cell::cc_id_to_cooridinate(this->id, this->shape, this->dim_y);

        // Torus or Mesh?
        this->primary_network_type = primary_network_type_in;
        assert(this->primary_network_type == 0);

        // Assign neighbor CCs to this Cell. This is based on the Shape and Dim
        this->add_neighbor_compute_cells();

        this->mesh_routing_policy = mesh_routing_policy_id_in;

        // this->distance_class_length = 2; //(this->hx * 15) + (this->hy * 15);
        this->number_of_virtual_channels = 2;

        this->recv_channel_per_neighbor.resize(
            this->number_of_neighbors,
            std::vector<FixedSizeQueue<Operon>>(this->number_of_virtual_channels,
                                                FixedSizeQueue<Operon>(buffer_size)));

        // send channel buffer can only hold one operon since its there to put send (put) in the
        // recv channel of the neighbor.
        this->send_channel_per_neighbor.resize(
            this->number_of_neighbors,
            std::vector<FixedSizeQueue<Operon>>(this->number_of_virtual_channels,
                                                FixedSizeQueue<Operon>(1)));

        // this->send_channel_per_neighbor_current_distance_class.resize(this->number_of_neighbors);
        this->send_channel_per_neighbor_contention_count.resize(this->number_of_neighbors,
                                                                MaxCounter());

        // Start from 0th and then alternate by % 4 (here 4 = number of neighbers for square cell
        // type for example)
        this->current_recv_channel_to_start_a_cycle = 0;

        this->last_congested_cycle = std::nullopt;

        // Experimental. The cells don't have sense of a global cycle. It is here for debuging and
        // making the implementation of the simulator easier such as throttling.
        this->current_cycle = 0;
    }

    ~SinkCell() override = default;
};

#endif // SINK_CELL_HPP

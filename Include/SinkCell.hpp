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

#include <iostream>
#include <stdlib.h>

class SinkCell : public Cell
{
  public:
    // Prepare the cycle. This involves moving operon data into either the action queue or send
    // buffers of the network links
    void prepare_a_cycle();

    // Execute a single cycle for this cell
    void run_a_computation_cycle();

    // TODO: write comments
    void prepare_a_communication_cycle();

    // TODO: write comments
    void run_a_communication_cycle(std::vector<std::shared_ptr<Cell>>& CCA_chip);

    // Checks if the cell is active or not
    bool is_compute_cell_active();

    // Routing
    // Based on the routing algorithm and the shape of CCs it will return which neighbor to pass
    // this operon to. The returned value is the index [0...number of neighbors) coresponding
    // clockwise the channel id of the physical shape.
    u_int32_t get_route_towards_cc_id(u_int32_t dst_cc_id);

    // This is the id of the Htree node in the second layer network. Think of this as connecting in
    // the 3rd dimension under the chip using TSA (Through Silicon Via)
    u_int32_t connecting_htree_node_id;

    // Send channel/link toward the second layer Htree. The width of this channel is based on the
    // shape of the Cells. For an square it is 4, meaning 4 Operons can be sent in a single cycle.
    std::vector<std::optional<Operon>> send_channel_to_htree_node;

    // Same as the base class `Cell` this is needed to satisty simulation. Read the comment for
    // `recv_channel_per_neighbor` in the base class `Cell`
    std::vector<std::optional<Operon>> recv_channel_to_htree_node;

    // Constructor
    SinkCell(u_int32_t id_in,
             CellType type_in,
             u_int32_t connecting_htree_node_id_in,
             computeCellShape shape_in,
             u_int32_t dim_x_in,
             u_int32_t dim_y_in,
             u_int32_t hx_in,
             u_int32_t hy_in,
             u_int32_t hdepth_in)
    {
        this->id = id_in;
        this->type = type_in;
        this->statistics.type = this->type;

        this->connecting_htree_node_id = connecting_htree_node_id_in;
        this->shape = shape_in;
        this->number_of_neighbors = Cell::get_number_of_neighbors(this->shape);

        this->dim_x = dim_x_in;
        this->dim_y = dim_y_in;

        this->hx = hx_in;
        this->hy = hy_in;
        this->hdepth = hdepth_in;

        this->cooridates = Cell::cc_id_to_cooridinate(this->id, this->shape, this->dim_y);

        // Assign neighbor CCs to this Cell. This is based on the Shape and Dim
        this->add_neighbor_compute_cells();

        for (u_int32_t i = 0; i < this->number_of_neighbors; i++) {
            // Channels/Links per neighbor in the 2D mesh
            this->send_channel_per_neighbor.push_back(std::nullopt);
            this->recv_channel_per_neighbor.push_back(std::nullopt);

            // Channels/Links to the secondary network Htree node
            this->send_channel_to_htree_node.push_back(std::nullopt);
            this->recv_channel_to_htree_node.push_back(std::nullopt);
        }
    }
};

#endif // SINK_CELL_HPP

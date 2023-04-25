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

#ifndef CELL_HPP
#define CELL_HPP

#include "Action.hpp"
#include "Address.hpp"
#include "Constants.hpp"
#include "Operon.hpp"
#include "Task.hpp"

#include "memory_management.hpp"

#include <map>
#include <optional>
#include <queue>
#include <stdlib.h>

typedef std::pair<int32_t, int32_t> SignedCoordinates;

// Type of the Cell: ComputeCell or HtreeNode
enum class CellType : u_int32_t
{
    compute_cell = 0,
    second_layer_network_node, // Htree node
    CellType_invalid
};

// Shape of the ComputeCell
enum class computeCellShape : u_int32_t
{
    block_1D = 0, // TODO: remove this
    triangular,
    square, // block 2D
    hexagon,
    computeCellShape_invalid
};

// Base class for Cells: These cells can be regular compute cells or nodes to the secondary network
// such as a Htree
class Cell
{
  public:
    // Identity of the Cell.
    u_int32_t id;

    // Type of the Cell: ComputeCell or Htree node?
    CellType type;

    // Coordinates of this Cell in the CCA chip. It depends on the Chip dinemsions and
    // shapes of Cells.
    std::pair<u_int32_t, u_int32_t> cooridates;

    // Shape of the Cell
    computeCellShape shape;

    // Dimensions of the CCA chip this cell belongs to. This is needed for routing as the routing
    // logic is implemented inside the cells and they need to be aware of the global chip
    // configuration.
    u_int32_t dim_x;
    u_int32_t dim_y;

    // Communication of the Cell.

    // number_of_neighbors is the maximum number of connections a single Cell can architecturally
    // have. A triangular Cell has 3 neighbors, a square has 4, and hexagon has 6 etc.
    // TODO: This seems to be redundant unless we really want to discriminate the std::nullopt,
    // later
    u_int32_t number_of_neighbors;

    // IDs and Coordinates of the neighbors
    std::vector<std::optional<std::pair<u_int32_t, std::pair<u_int32_t, u_int32_t>>>>
        neighbor_compute_cells;

    // Per neighbor send channel/link
    std::vector<std::optional<Operon>> send_channel_per_neighbor;

    // This is needed to satisty simulation. Because a sending Cell can not just enqueue an operon
    // into the current working buffer of a neighbor Cell. If it does then the neighbor may start
    // computation (or work) on that operon in the current simulation cycle. The receiving neighbor
    // must process it in the next cycle. Therefore, this send/recv buffer serves that purpose. At
    // the end of each simulation cycle each Cell will move an operon from this buffer to its
    // working set of operons and will then execute those operons in the next cycle. This move is
    // not part of the computation but is only there for simulation so as not to break the
    // semantics/pragmatics of CCA.
    std::vector<std::optional<Operon>> recv_channel_per_neighbor;
};

#endif // CELL_HPP

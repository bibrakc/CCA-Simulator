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

#ifndef HTREE_CELL_HPP
#define HTREE_CELL_HPP

// #include "Action.hpp"
// #include "Address.hpp"
#include "Cell.hpp"
// #include "Constants.hpp"
// #include "Operon.hpp"
// #include "Task.hpp"

#include <iostream>
#include <stdlib.h>

class HtreeCell : public Cell
{
  public:
    // Constructor
    HtreeCell(u_int32_t id_in,
              CellType type_in,
              computeCellShape shape_in,
              u_int32_t dim_x_in,
              u_int32_t dim_y_in)
    {
        this->id = id_in;
        this->type = type_in;
        this->shape = shape_in;
        this->number_of_neighbors = ComputeCell::get_number_of_neighbors(this->shape);

        this->dim_x = dim_x_in;
        this->dim_y = dim_y_in;

        this->cooridates =
            ComputeCell::cc_id_to_cooridinate(this->id, this->shape, this->dim_x, this->dim_y);

        // Assign neighbor CCs to this CC. This is based on the Shape and Dim
        // this->add_neighbor_compute_cells();

        for (int i = 0; i < this->number_of_neighbors; i++) {
            this->send_channel_per_neighbor.push_back(std::nullopt);
            this->recv_channel_per_neighbor.push_back(std::nullopt);
        }
    }
};

#endif // HTREE_CELL_HPP

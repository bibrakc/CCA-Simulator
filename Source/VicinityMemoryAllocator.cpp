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

#include "VicinityMemoryAllocator.hpp"
#include "CCASimulator.hpp"

// Vicinity allocator across Compute Cells that are nearby a source Compute Cell.

auto
VicinityMemoryAllocator::get_next_available_cc(CCASimulator& cca_simulator) -> u_int32_t
{

    auto get_next_cc_id = []() {
        // How much can it move
        u_int32_t left_right_freedom = this->spread_cols / 2;
        u_int32_t up_down_freedom = this->spread_rows / 2;

        auto [col, row] = Cell::cc_id_to_cooridinate(this->next_cc_id);

        u_int32_t next_cc_col = col + 1;
        u_int32_t next_cc_row = row;
        if (next_cc_col == cca_simulator.dim_y) {
            next_cc_col = col - left_right_freedom;
            next_cc_row++;
            if (next_cc_row == cca_simulator.dim_x) {
                next_cc_row = row - up_down_freedom;
            }
        }
        Coordinates next_cc(next_cc_col, next_cc_row);

        // Finally return the `next_cc_id`
        return Cell::cc_cooridinate_to_id(
            next_cc, cca_simulator.shape_of_compute_cells, cca_simulator.dim_y);
    };

    u_int32_t source_cc_id = Cell::cc_cooridinate_to_id(
        this->source_cc, cca_simulator.shape_of_compute_cells, cca_simulator.dim_y);

    // Skip the Cell if it is not of type ComputeCell or of the source cc id
    while (cca_simulator.CCA_chip[this->next_cc_id]->type != CellType::compute_cell ||
           this->next_cc_id == source_cc_id) {
        // Get next `next_cc_id`

        // Finally get the `next_cc_id`
        this->next_cc_id = get_next_cc_id();
    }
    u_int32_t const cc_available = this->next_cc_id;
    this->next_cc_id = get_next_cc_id();
    return cc_available;
}

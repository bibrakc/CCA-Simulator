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

#ifndef VICINITY_MEMORY_ALLOCATOR_HPP
#define VICINITY_MEMORY_ALLOCATOR_HPP

#include "MemoryAllocator.hpp"

#include "types.hpp"

// Vicinity allocator across Compute Cells that are nearby a source Compute Cell.
class VicinityMemoryAllocator : public MemoryAllocator
{
  public:
    // The source CC from where to form this vicinity
    Coordinates source_cc{};

    // Defines the size of the vicinity as a matrix (square), where the `source_cc` is at the
    // center. The matrix dims are `spread_rows` x `spread_cols`.
    u_int32_t spread_rows;
    u_int32_t spread_cols;

    /* auto candidate_cc_exists(SignedCoordinates candidate_next_cc,
                         u_int32_t CCA_dim_x,
                         u_int32_t CCA_dim_y) -> bool
{
    auto [cc_coordinate_x, cc_coordinate_y] = candidate_next_cc;
    int const zero = 0;
    if ((cc_coordinate_x < zero) || (cc_coordinate_x >= static_cast<int>(CCA_dim_y)) ||
        (cc_coordinate_y < zero) || (cc_coordinate_y >= static_cast<int>(CCA_dim_x))) {
        return false;
    }
    return true;
} */

    VicinityMemoryAllocator(Coordinates source_cc_in,
                            u_int32_t spread_rows_in,
                            u_int32_t spread_cols_in,
                            computeCellShape shape_of_cc,
                            u_int32_t CCA_dim_x,
                            u_int32_t CCA_dim_y)
        : source_cc(source_cc_in)
        , spread_rows(spread_rows_in)
        , spread_cols(spread_cols_in)
    {
        assert(this->shape_of_cc == computeCellShape::square);

        // Make sure they are odd.
        assert(this->spread_rows % 2);
        assert(this->spread_cols % 2);

        // Make sure to put reasonable values.
        assert(this->spread_rows > 2);
        assert(this->spread_cols > 2);
        assert(this->spread_rows < CCA_dim_y / 2);
        assert(this->spread_cols < CCA_dim_x / 2);

        // Initialize `next_cc_id`
        // How much can it move
        u_int32_t left_right_freedom = this->spread_cols / 2;
        u_int32_t up_down_freedom = this->spread_rows / 2;

        auto [source_col, source_row] = this->source_cc;

        u_int32_t next_cc_col = source_col + 1;
        u_int32_t next_cc_row = source_row;
        if (next_cc_col == CCA_dim_y) {
            next_cc_col = source_col - left_right_freedom;
            next_cc_row++;
            if (next_cc_row == CCA_dim_x) {
                next_cc_row = source_row - up_down_freedom;
            }
        }
        Coordinates next_cc(next_cc_col, next_cc_row);

        // Finally initialize `next_cc_id`
        this->next_cc_id = Cell::cc_cooridinate_to_id(next_cc, shape_of_cc, CCA_dim_y);
    }

    auto get_next_available_cc(CCASimulator& cca_simulator) -> u_int32_t override;
};

#endif // VICINITY_MEMORY_ALLOCATOR_HPP

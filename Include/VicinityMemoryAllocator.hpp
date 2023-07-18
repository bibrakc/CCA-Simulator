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

#include "Cell.hpp"
#include "MemoryAllocator.hpp"

#include "types.hpp"

#include <cassert>

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

    // Dimensions of the CCA chip.
    u_int32_t cca_dim_x;
    u_int32_t cca_dim_y;

    VicinityMemoryAllocator(Coordinates source_cc_in,
                            u_int32_t spread_rows_in,
                            u_int32_t spread_cols_in,
                            u_int32_t cca_dim_x_in,
                            u_int32_t cca_dim_y_in,
                            computeCellShape shape_of_cc)
        : source_cc(source_cc_in)
        , spread_rows(spread_rows_in)
        , spread_cols(spread_cols_in)
        , cca_dim_x(cca_dim_x_in)
        , cca_dim_y(cca_dim_y_in)
    {
        assert(shape_of_cc == computeCellShape::square);

        // Make sure to put reasonable values.
        assert(this->spread_rows > 1);
        assert(this->spread_cols > 1);
        assert(this->spread_rows <= this->cca_dim_y / 2);
        assert(this->spread_cols <= this->cca_dim_x / 2);

        // Initialize `next_cc_id`
        this->next_cc_id = Cell::cc_cooridinate_to_id(
            this->generate_random_coordinates(), shape_of_cc, this->cca_dim_y);
    }

    auto get_next_available_cc(CCASimulator& cca_simulator) -> u_int32_t override;

    VicinityMemoryAllocator() = default;

  private:
    auto generate_random_coordinates() -> Coordinates;
};

#endif // VICINITY_MEMORY_ALLOCATOR_HPP

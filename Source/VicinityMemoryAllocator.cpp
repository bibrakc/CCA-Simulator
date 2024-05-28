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
    u_int32_t source_cc_id = Cell::cc_cooridinate_to_id(
        this->source_cc, cca_simulator.shape_of_compute_cells, cca_simulator.dim_y);

    // Skip the Cell if it is not of type ComputeCell or of the source cc id
    while (cca_simulator.CCA_chip[this->next_cc_id]->type != CellType::compute_cell ||
           this->next_cc_id == source_cc_id) {
        // Get next `next_cc_id`
        this->next_cc_id = Cell::cc_cooridinate_to_id(this->generate_random_coordinates(),
                                                      cca_simulator.shape_of_compute_cells,
                                                      this->cca_dim_y);
    }
    u_int32_t const cc_available = this->next_cc_id;
    this->next_cc_id = Cell::cc_cooridinate_to_id(
        this->generate_random_coordinates(), cca_simulator.shape_of_compute_cells, this->cca_dim_y);

    /*
      // For debugging.s
      // cols
      auto source_cols = static_cast<int>(this->source_cc.first);
      // rows
      auto source_rows = static_cast<int>(this->source_cc.second);

      auto [random_x, random_y] = Cell::cc_id_to_cooridinate(
          cc_available, cca_simulator.shape_of_compute_cells, this->cca_dim_y);
      std::cout << "Source: (" << source_cols << ", " << source_rows << "), "
                << "Randomly generated coordinates: (" << random_x << ", " << random_y << ")"
                << std::endl;
    */

    return cc_available;
}

auto
VicinityMemoryAllocator::get_next_available_cc(ComputeCell& cc) -> u_int32_t
{
    // Currently not supporting H-tree network as not checking for SinkCells.
    assert(cc.hdepth == 0);

    u_int32_t const cc_available = this->next_cc_id;
    this->next_cc_id =
        Cell::cc_cooridinate_to_id(this->generate_random_coordinates(), cc.shape, this->cca_dim_y);

    return cc_available;
}

auto
VicinityMemoryAllocator::generate_random_coordinates() -> Coordinates
{

    // cols
    auto source_cols = static_cast<int>(this->source_cc.first);
    // rows
    auto source_rows = static_cast<int>(this->source_cc.second);

    auto vicinity_spread_cols = static_cast<int>(this->spread_cols);
    auto vicinity_spread_rows = static_cast<int>(this->spread_rows);

    // Calculate the boundaries for random coordinates.
    int const min_x = std::max(source_cols - vicinity_spread_cols, 0);
    int const max_x =
        std::min(source_cols + vicinity_spread_cols, static_cast<int>(this->cca_dim_y) - 1);
    int const min_y = std::max(source_rows - vicinity_spread_rows, 0);
    int const max_y =
        std::min(source_rows + vicinity_spread_rows, static_cast<int>(this->cca_dim_x) - 1);

    // Generate random x and y coordinates within the boundaries.
    u_int32_t random_x = min_x + (std::rand() % (max_x - min_x + 1));
    u_int32_t random_y = min_y + (std::rand() % (max_y - min_y + 1));

    /*  std::cout << "Source: (" << source_cols << ", " << source_rows << "), "
               << "Randomly generated coordinates: (" << random_x << ", " << random_y << ")"
               << std::endl; */

    // Return the random coordinates.
    return Coordinates(random_x, random_y);
}

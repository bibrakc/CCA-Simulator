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

#ifndef CCASimulator_HPP
#define CCASimulator_HPP

#include "Action.hpp"
#include "Address.hpp"
#include "Cell.hpp"
#include "ComputeCell.hpp"
#include "Constants.hpp"
#include "Operon.hpp"
#include "Task.hpp"

#include "memory_management.hpp"

#include <map>
#include <queue>
#include <stdlib.h>

typedef unsigned long u_long;

class CCASimulator
{
  public:
    computeCellShape shape_of_compute_cells;

    // Dimensions of the CCA chip.
    u_int32_t dim_x, dim_y;

    // Dimensions and depth of the Htree. Here hx and hy are the dimensions of the block of cells
    // covered by a single end node of the Htree.
    u_int32_t hx, hy, hdepth;
    u_int32_t total_compute_cells;

    // Memory per compute cell and the total combined memory of this CCA chip
    u_int32_t memory_per_cc;
    u_int32_t total_chip_memory;

    // Declare the CCA Chip that is composed of Compute Cell(s) and any SinkCell(s)
    std::vector<std::shared_ptr<Cell>> CCA_chip;

    bool global_active_cc;
    u_long total_cycles;

    CCASimulator(computeCellShape shape_in,
                 u_int32_t dim_x_in,
                 u_int32_t dim_y_in,
                 u_int32_t hx_in,
                 u_int32_t hy_in,
                 u_int32_t hdepth_in,
                 u_int32_t total_compute_cells_in,
                 u_int32_t memory_per_cc_in)
        : shape_of_compute_cells(shape_in)
        , dim_x(dim_x_in)
        , dim_y(dim_y_in)
        , hx(hx_in)
        , hy(hy_in)
        , hdepth(hdepth_in)
        , total_compute_cells(total_compute_cells_in)
        , memory_per_cc(memory_per_cc_in)
    {
        this->global_active_cc = false;
        this->total_cycles = 0;
        this->total_chip_memory = this->total_compute_cells * this->memory_per_cc;
        this->create_the_chip();
    }

    inline void generate_label(std::ostream& os)
    {
        os << "shape\tdim_x\tdim_y\thx\thy\thdepth\ttotal_compute_cells\ttotal_chip_memory(byes)\n";
    }

    inline void output_description_in_a_single_line(std::ostream& os)
    {
        os << ComputeCell::get_compute_cell_shape_name(this->shape_of_compute_cells) << "\t"
           << this->dim_x << "\t" << this->dim_y << "\t" << this->hx << "\t" << this->hy << "\t"
           << this->hdepth << "\t" << this->total_compute_cells << "\t" << this->total_chip_memory
           << "\n";
    }

    inline std::pair<u_int32_t, u_int32_t> get_compute_cell_coordinates(
        u_int32_t cc_id,
        computeCellShape shape_of_compute_cells,
        u_int32_t dim_x,
        u_int32_t dim_y);

    std::pair<u_int32_t, u_int32_t> cc_id_to_cooridinate(u_int32_t cc_id);

    u_int32_t cc_cooridinate_to_id(std::pair<u_int32_t, u_int32_t> cc_cooridinate);

    void add_neighbor_compute_cells(std::shared_ptr<ComputeCell> cc);

    void create_the_chip();
    std::optional<Address> allocate_and_insert_object_on_cc(u_int32_t cc_id,
                                                            void* obj,
                                                            size_t size_of_obj);

    void run_simulation();

    // Get the pointer to the object at `Address addr_in`
    void* get_object(Address addr_in) const;
};

// TODO: Find a better file/location for MemoryAlloctor classes
// Base class for memroy allocation
class MemoryAlloctor
{
  public:
    u_int32_t next_cc_id{};
    virtual u_int32_t get_next_available_cc(CCASimulator&) = 0;
};

// Cyclic allocator across all Compute Cells
class CyclicMemoryAllocator : public MemoryAlloctor
{
  public:
    u_int32_t get_next_available_cc(CCASimulator& cca_simulator)
    {
        // Skip the Cell is it is not of type ComputeCell
        if (cca_simulator.CCA_chip[this->next_cc_id]->type != CellType::compute_cell) {
            this->next_cc_id = (this->next_cc_id + 1) % cca_simulator.total_compute_cells;
        }
        u_int32_t cc_available = this->next_cc_id;
        this->next_cc_id = (this->next_cc_id + 1) % cca_simulator.total_compute_cells;
        return cc_available;
    }
};

#endif // CCASimulator_HPP

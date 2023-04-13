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

#include "CCASimulator.hpp"
#include "Action.hpp"
#include "Address.hpp"
#include "ComputeCell.hpp"
#include "Constants.hpp"
#include "Operon.hpp"
#include "Task.hpp"

#include "memory_management.hpp"

/* #include <map>
#include <queue> */
#include <stdlib.h>

/*
CCASimulator::CCASimulator(computeCellShape shape_in,
                           u_int32_t dim_in,
                           u_int32_t total_compute_cells_in)
    : shape_of_compute_cells(shape_in)
    , dim(dim_in)
    , total_compute_cells(total_compute_cells_in)
{
    this->global_active_cc = false;
    this->total_cycles = 0;
    this->create_the_chip();
} */

// Chip is coordinates are from top-left....
inline std::pair<u_int32_t, u_int32_t>
CCASimulator::get_compute_cell_coordinates(u_int32_t cc_id,
                                           computeCellShape shape_of_compute_cells,
                                           u_int32_t dim_x,
                                           u_int32_t dim_y)
{
    // std::cout << "cc_id: " << cc_id << " dim_x: " << dim_x << " dim_y: " << dim_y << " ---> (" <<
    // cc_id % dim_y << ", " << cc_id / dim_y << ")\n";
    return std::pair<u_int32_t, u_int32_t>(cc_id % dim_y, cc_id / dim_y);
}

std::pair<u_int32_t, u_int32_t>
CCASimulator::cc_id_to_cooridinate(u_int32_t cc_id)
{
    return ComputeCell::cc_id_to_cooridinate(
        cc_id, this->shape_of_compute_cells, this->dim_x, this->dim_y);
}

u_int32_t
CCASimulator::cc_cooridinate_to_id(std::pair<u_int32_t, u_int32_t> cc_cooridinate)
{

    return ComputeCell::cc_cooridinate_to_id(
        cc_cooridinate, this->shape_of_compute_cells, this->dim_x, this->dim_y);
}

void
CCASimulator::create_the_chip()
{

    // Cannot simply openmp parallelize this. It is very atomic.
    for (u_int32_t i = 0; i < this->total_compute_cells; i++) {

        // Create individual compute cells of shape computeCellShape::block_1D
        this->CCA_chip.push_back(std::make_shared<ComputeCell>(i,
                                                               shape_of_compute_cells,
                                                               this->dim_x,
                                                               this->dim_y,
                                                               7 * 1024 * 1024)); // 7 MB

        if constexpr (debug_code) {
            std::cout << *this->CCA_chip.back().get();
        }
    }
}

std::optional<Address>
CCASimulator::allocate_and_insert_object_on_cc(u_int32_t cc_id, void* obj, size_t size_of_obj)
{
    return this->CCA_chip[cc_id]->create_object_in_memory(obj, size_of_obj);
}

void
CCASimulator::run_simulation()
{
    // TODO: later we can remove this and implement the termination detection itself. But for
    // now this works.
    this->total_cycles = 0;

    bool global_active_cc_local = true;

    // while (this->total_cycles < 3000) {
    while (global_active_cc_local) {

        // std::cout << "Cycle # "<< this->total_cycles << "\n\n";

        global_active_cc_local = false;

// Run a cycle: First the computation cycle (that includes the preparation of operons from
// previous cycle)
#pragma omp parallel for
        for (int i = 0; i < this->CCA_chip.size(); i++) {
            this->CCA_chip[i]->run_a_computation_cycle();
        }

// Prepare communication cycle
#pragma omp parallel for
        for (int i = 0; i < this->CCA_chip.size(); i++) {
            this->CCA_chip[i]->prepare_a_communication_cycle();
        }

// Run communication cycle
#pragma omp parallel for
        for (int i = 0; i < this->CCA_chip.size(); i++) {
            this->CCA_chip[i]->run_a_communication_cycle(this->CCA_chip);
        }
// Check for termination
#pragma omp parallel for reduction(| : global_active_cc_local)
        for (int i = 0; i < this->CCA_chip.size(); i++) {
            global_active_cc_local |= this->CCA_chip[i]->is_compute_cell_active();
        }
        total_cycles++;
        //  std::cout << "\n--------\n";
    }
    this->global_active_cc = global_active_cc_local;
}

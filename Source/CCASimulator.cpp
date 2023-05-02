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
#include "HtreeNode.hpp"
#include "Operon.hpp"
#include "SinkCell.hpp"
#include "Task.hpp"

#include "memory_management.hpp"

#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <stdlib.h>
#include <vector>

// Chip's coordinates are from top-left....
/*
For a CCA chip of 4x4 with square shaped compute cells
(0,0)----(1,0)----(2,0)----(3,0)
 |         |         |         |
 |         |         |         |
(0,1)----(1,1)----(2,1)----(3,1)
 |         |         |         |
 |         |         |         |
(0,2)----(1,2)----(2,2)----(3,2)
 |         |         |         |
 |         |         |         |
(0,3)----(1,3)----(2,3)----(3,3)
*/
inline Coordinates
CCASimulator::get_compute_cell_coordinates(u_int32_t cc_id,
                                           computeCellShape shape_of_compute_cells,
                                           u_int32_t dim_x,
                                           u_int32_t dim_y)
{
    return Coordinates(cc_id % dim_y, cc_id / dim_y);
}

Coordinates
CCASimulator::cc_id_to_cooridinate(u_int32_t cc_id)
{
    return ComputeCell::cc_id_to_cooridinate(
        cc_id, this->shape_of_compute_cells, this->dim_x, this->dim_y);
}

u_int32_t
CCASimulator::cc_cooridinate_to_id(Coordinates cc_cooridinate)
{

    return ComputeCell::cc_cooridinate_to_id(
        cc_cooridinate, this->shape_of_compute_cells, this->dim_x, this->dim_y);
}

// Create the chip of type square cells with Htree. It includes creating the cells and initializing
// them with IDs and their types and more
void
CCASimulator::create_square_cell_htree_chip()
{
    // Where the first sink cells exist for the Htree
    // If second layer network type == Htree
    u_int32_t next_row_sink_cells = this->hx / 2;
    u_int32_t next_col_sink_cells = this->hy / 2;
    // Cannot simply openmp parallelize this. It is very atomic.
    for (u_int32_t i = 0; i < this->dim_x; i++) {
        for (u_int32_t j = 0; j < this->dim_y; j++) {

            u_int32_t cc_id = i * this->dim_y + j;

            // Insert the sink cell
            if ((next_row_sink_cells == i) && (next_col_sink_cells == j)) {

                // std::cout << "SinkCell: (" << next_col_sink_cells << "," << next_row_sink_cells
                // << ")\n";

                // Create the sink cells where the chip connects to the underlying
                // second layer network (for example the Htree)
                this->CCA_chip.push_back(std::make_shared<SinkCell>(cc_id,
                                                                    CellType::sink_cell,
                                                                    0, // id of the connecting node
                                                                    shape_of_compute_cells,
                                                                    this->dim_x,
                                                                    this->dim_y,
                                                                    this->hx,
                                                                    this->hy,
                                                                    this->hdepth));
                next_col_sink_cells += this->hy;

            } else {
                // Create individual compute cells of computeCellShape shape_of_compute_cells
                this->CCA_chip.push_back(std::make_shared<ComputeCell>(cc_id,
                                                                       CellType::compute_cell,
                                                                       shape_of_compute_cells,
                                                                       this->dim_x,
                                                                       this->dim_y,
                                                                       this->hx,
                                                                       this->hy,
                                                                       this->hdepth,
                                                                       this->memory_per_cc));
            }
            if constexpr (debug_code) {
                std::cout << *this->CCA_chip.back().get();
            }
        }
        if (next_row_sink_cells == i) {
            next_row_sink_cells += this->hx;
            next_col_sink_cells = this->hy / 2;
        }
    }
}

// Create the chip of type square cells with only mesh connetion. There is not htree or any second
// layer network involved. It includes creating the cells and initializing them with IDs and their
// types and more
void
CCASimulator::create_square_cell_mesh_only_chip()
{

    // Cannot simply openmp parallelize this. It is very atomic.
    for (u_int32_t i = 0; i < this->dim_x; i++) {
        for (u_int32_t j = 0; j < this->dim_y; j++) {

            u_int32_t cc_id = i * this->dim_y + j;

            // Create individual compute cells of computeCellShape shape_of_compute_cells
            this->CCA_chip.push_back(std::make_shared<ComputeCell>(cc_id,
                                                                   CellType::compute_cell,
                                                                   shape_of_compute_cells,
                                                                   this->dim_x,
                                                                   this->dim_y,
                                                                   this->hx,
                                                                   this->hy,
                                                                   this->hdepth,
                                                                   this->memory_per_cc));

            if constexpr (debug_code) {
                std::cout << *this->CCA_chip.back().get();
            }
        }
    }
}

// The main chip creation function
void
CCASimulator::create_the_chip()
{

    if (this->shape_of_compute_cells == computeCellShape::square) {
        if (this->hdepth == 0) {
            this->create_square_cell_mesh_only_chip();
        } else {

            this->create_square_cell_htree_chip();
        }
    } else {
        std::cerr << "Error! Cannot create chip of non-supported type cell shape\n";
        exit(0);
    }
}

// Get the pointer to the object at `Address addr_in`
void*
CCASimulator::get_object(Address addr_in) const
{
    // dynamic_pointer_cast to go down/across class hierarchy
    auto compute_cell = std::dynamic_pointer_cast<ComputeCell>(this->CCA_chip[addr_in.cc_id]);
    if (!compute_cell) {
        std::cerr << "Bug! Invalid addr in CCASimulator::get_object\n";
    }
    return compute_cell->get_object(addr_in);
}

std::optional<Address>
CCASimulator::allocate_and_insert_object_on_cc(u_int32_t cc_id, void* obj, size_t size_of_obj)
{

    // TODO: Hahaha comedy! See how to made this casting and pointers more graceful and elegant ASAP

    // std::shared_ptr<ComputeCell> cc_ptr = this->CCA_chip[cc_id];
    ComputeCell* cc_ptr = static_cast<ComputeCell*>(this->CCA_chip[cc_id].get());
    return cc_ptr->create_object_in_memory(obj, size_of_obj);
    // return this->CCA_chip[cc_id]->create_object_in_memory(obj, size_of_obj);
}

void
CCASimulator::run_simulation()
{
    // TODO: later we can remove this and implement the termination detection itself. But for
    // now this works.
    this->total_cycles = 0;

    bool global_active_cc_local = true;
    bool global_active_htree = true;

    bool is_system_active = true;

    while (is_system_active) {

        global_active_cc_local = false;
        global_active_htree = false;
        is_system_active = false;

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

        // std::cout << "prepare_communication_cycle # " << total_cycles << "\n";
        for (int i = 0; i < this->htree_all_nodes.size(); i++) {
            this->htree_all_nodes[i]->prepare_communication_cycle();
        }

// Run communication cycle
#pragma omp parallel for
        for (int i = 0; i < this->CCA_chip.size(); i++) {
            this->CCA_chip[i]->run_a_communication_cycle(this->CCA_chip);
        }

        // std::cout << "run_a_communication_cylce # " << total_cycles << "\n";
        for (int i = 0; i < this->htree_all_nodes.size(); i++) {
            this->htree_all_nodes[i]->run_a_communication_cylce();
        }

// Check for termination
#pragma omp parallel for reduction(| : global_active_cc_local)
        for (int i = 0; i < this->CCA_chip.size(); i++) {
            global_active_cc_local |= this->CCA_chip[i]->is_compute_cell_active();
        }

        for (int i = 0; i < htree_all_nodes.size(); i++) {
            global_active_htree |= htree_all_nodes[i]->is_htree_node_active();
        }
        total_cycles++;

        is_system_active = global_active_cc_local || global_active_htree;
    }
    this->global_active_cc = global_active_cc_local;
}

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
#include "Object.hpp"
#include "Operon.hpp"
#include "SinkCell.hpp"
#include "Task.hpp"

#include "memory_management.hpp"
#include "operators.hpp"

#include <cmath>
#include <cstring>
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
CCASimulator::get_compute_cell_coordinates(u_int32_t cc_id, u_int32_t dim_y)
{
    // Note: Later when new shapes are added this function night need to be changed to decide based
    // on the cell shape and chip dimensions
    return Coordinates(cc_id % dim_y, cc_id / dim_y);
}

Coordinates
CCASimulator::cc_id_to_cooridinate(u_int32_t cc_id)
{
    return ComputeCell::cc_id_to_cooridinate(cc_id, this->shape_of_compute_cells, this->dim_y);
}

u_int32_t
CCASimulator::cc_cooridinate_to_id(Coordinates cc_cooridinate)
{

    return ComputeCell::cc_cooridinate_to_id(
        cc_cooridinate, this->shape_of_compute_cells, this->dim_y);
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

                Coordinates sink_cell_cooridnates =
                    Coordinates(next_col_sink_cells, next_row_sink_cells);

                // std::cout << "SinkCell: " << sink_cell_cooridnates << "\n";

                auto htree_node_address_entry =
                    this->htree_network.htree_end_nodes.find(sink_cell_cooridnates);

                if (htree_node_address_entry == this->htree_network.htree_end_nodes.end()) {
                    // Key does not exist in the map
                    std::cout << "Bug! SinkCell not found" << std::endl;
                    exit(0);
                }

                // Create the sink cells where the chip connects to the underlying
                // second layer network (for example the Htree)
                std::shared_ptr<SinkCell> sink_cell =
                    std::make_shared<SinkCell>(cc_id,
                                               CellType::sink_cell,
                                               htree_node_address_entry->second,
                                               shape_of_compute_cells,
                                               this->dim_x,
                                               this->dim_y,
                                               this->hx,
                                               this->hy,
                                               this->hdepth,
                                               this->mesh_routing_policy_id);
                this->CCA_chip.push_back(sink_cell);

                // Connect the underlying Htree end node to this sink cell
                htree_node_address_entry->second->sink_cell_connector = sink_cell;

                next_col_sink_cells += this->hy;

            } else {
                // Create individual compute cells of computeCellShape shape_of_compute_cells
                this->CCA_chip.push_back(
                    std::make_shared<ComputeCell>(cc_id,
                                                  CellType::compute_cell,
                                                  shape_of_compute_cells,
                                                  this->dim_x,
                                                  this->dim_y,
                                                  this->hx,
                                                  this->hy,
                                                  this->hdepth,
                                                  this->memory_per_cc,
                                                  this->host_memory,
                                                  this->mesh_routing_policy_id));
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
                                                                   this->memory_per_cc,
                                                                   this->host_memory,
                                                                   this->mesh_routing_policy_id));

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

// Register a function event
CCAFunctionEvent
CCASimulator::register_function_event(handler_func function_event_handler)
{
    return this->function_events.register_function_event(function_event_handler);
}

// Return the memory used in bytes
u_int32_t
CCASimulator::get_host_memory_used()
{
    return this->host_memory_curr_ptr - this->host_memory_raw_ptr;
}

// In bytes
u_int32_t
CCASimulator::get_host_memory_curr_ptr_offset()
{
    return get_host_memory_used();
}

// Get memory left in bytes
u_int32_t
CCASimulator::host_memory_available_in_bytes()
{
    return this->host_memory_size_in_bytes - this->get_host_memory_used();
}

// Get the pointer to the object at `Address addr_in`
void*
CCASimulator::get_object(Address addr_in) const
{

    if (addr_in.type == adressType::host_address) {
        return (this->host_memory.get() + addr_in.addr);
    }

    // dynamic_pointer_cast to go down/across class hierarchy
    auto compute_cell = std::dynamic_pointer_cast<ComputeCell>(this->CCA_chip[addr_in.cc_id]);
    if (!compute_cell) {
        std::cerr << "Bug! Invalid addr in CCASimulator::get_object\n";
    }
    return compute_cell->get_object(addr_in);
}

// Create a CCATerminator object on host and return the address
std::optional<Address>
CCASimulator::create_terminator()
{
    if (this->host_memory_available_in_bytes() < sizeof(CCATerminator)) {
        return std::nullopt;
    }

    CCATerminator host_terminator;
    u_int32_t obj_memory_addr_offset = get_host_memory_curr_ptr_offset();
    Address host_terminator_addr(this->host_id, obj_memory_addr_offset, adressType::host_address);

    host_terminator.terminator.my_object = host_terminator_addr;

    memcpy(this->host_memory_curr_ptr, &host_terminator, sizeof(CCATerminator));
    this->host_memory_curr_ptr += sizeof(CCATerminator);

    return host_terminator_addr;
}
bool
CCASimulator::is_diffusion_active(Address terminator_in)
{

    CCATerminator* terminator_obj = static_cast<CCATerminator*>(this->get_object(terminator_in));
    return terminator_obj->terminator.is_active();
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
CCASimulator::run_simulation(Address app_terminator)
{
    // TODO: later we can remove this and implement the termination detection itself. But for
    // now this works.
    this->total_cycles = 0;

    bool is_system_active = true;

    // while (is_system_active) {
    while (this->is_diffusion_active(app_terminator)) {
        // u_int32_t count_temp = 0;
        // while (count_temp < 200) {
        //     count_temp++;

        is_system_active = false;

// Run a cycle: First the computation cycle (that includes the preparation of operons from
// previous cycle)
#pragma omp parallel for
        for (u_int32_t i = 0; i < this->CCA_chip.size(); i++) {
            this->CCA_chip[i]->run_a_computation_cycle(this->CCA_chip, &this->function_events);
        }

// Prepare communication cycle
#pragma omp parallel for
        for (u_int32_t i = 0; i < this->CCA_chip.size(); i++) {
            this->CCA_chip[i]->prepare_a_communication_cycle(this->CCA_chip);
        }

        if (this->htree_network.hdepth != 0) {
            for (u_int32_t i = 0; i < this->htree_network.htree_all_nodes.size(); i++) {
                this->htree_network.htree_all_nodes[i]->prepare_communication_cycle();
            }
        }

// Run communication cycle
#pragma omp parallel for
        for (u_int32_t i = 0; i < this->CCA_chip.size(); i++) {
            this->CCA_chip[i]->run_a_communication_cycle(this->CCA_chip);
        }

        if (this->htree_network.hdepth != 0) {
#pragma omp parallel for
            for (u_int32_t i = 0; i < this->htree_network.htree_all_nodes.size(); i++) {
                this->htree_network.htree_all_nodes[i]->run_a_communication_cylce();
            }
        }
        // Check for termination
        u_int32_t sum_global_active_cc_local = 0;
        // Also put the active status in the statistics to create an animation later
        std::shared_ptr<u_int32_t[]> active_status_frame_per_cells(
            new u_int32_t[this->CCA_chip.size()](), std::default_delete<u_int32_t[]>());

#pragma omp parallel for reduction(+ : sum_global_active_cc_local)
        for (u_int32_t i = 0; i < this->CCA_chip.size(); i++) {
            active_status_frame_per_cells[i] = this->CCA_chip[i]->is_compute_cell_active();
            if (active_status_frame_per_cells[i]) {
                sum_global_active_cc_local++;
            }
        }
        this->cca_statistics.individual_cells_active_status_per_cycle.push_back(
            active_status_frame_per_cells);

        u_int32_t sum_global_active_htree = 0;
        if (this->htree_network.hdepth != 0) {
#pragma omp parallel for reduction(+ : sum_global_active_htree)
            for (u_int32_t i = 0; i < htree_network.htree_all_nodes.size(); i++) {
                if (htree_network.htree_all_nodes[i]->is_htree_node_active()) {
                    sum_global_active_htree++;
                }
            }
        }

        if (sum_global_active_cc_local || sum_global_active_htree) {
            is_system_active = true;
        }
        double percent_CCs_active = 100.0 * static_cast<double>(sum_global_active_cc_local) /
                                    static_cast<double>(this->CCA_chip.size());
        double percent_htree_active = 100.0 * static_cast<double>(sum_global_active_htree) /
                                      static_cast<double>(htree_network.htree_all_nodes.size());
        std::cout << "End of cycle # " << total_cycles << " CCs Active: " << percent_CCs_active
                  << "%, htree Active: " << percent_htree_active << "%\n";
        //  << "%, Diffusion Termianated? = " << this->is_diffusion_active(app_terminator)

        this->cca_statistics.active_status.push_back(
            ActiveStatusPerCycle(percent_CCs_active, percent_htree_active));
        total_cycles++;

// Set new cycle # for every Cell: Experimental
#pragma omp parallel for
        for (u_int32_t i = 0; i < this->CCA_chip.size(); i++) {
            this->CCA_chip[i]->current_cycle++;
        }
    }
    this->global_active_cc = is_system_active;

    // Copy any internal Cell records, counters etc to its statistics
#pragma omp parallel for
    for (u_int32_t i = 0; i < this->CCA_chip.size(); i++) {
        this->CCA_chip[i]->copy_cell_simulation_records_to_statistics();
    }
}

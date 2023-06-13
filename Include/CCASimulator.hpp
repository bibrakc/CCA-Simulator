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
#include "CCAFunctionEvents.hpp"
#include "Cell.hpp"
#include "ComputeCell.hpp"
#include "Constants.hpp"
#include "Function.hpp"
#include "HtreeNetwork.hpp"
#include "HtreeNode.hpp"
#include "Routing.hpp"
#include "Task.hpp"

#include "memory_management.hpp"

#include <map>
#include <queue>
#include <stdlib.h>

// Used for `pow` function
#include <cmath>

typedef unsigned long u_long;

using CCATerminator = Object;

struct ActiveStatusPerCycle
{
    double cells_active_percent;
    double htree_active_percent;

    ActiveStatusPerCycle(double cells_active_percent_in, double htree_active_percent_in)
        : cells_active_percent(cells_active_percent_in)
        , htree_active_percent(htree_active_percent_in)
    {
    }
};
struct CCASimulatorStatistics
{
    std::vector<ActiveStatusPerCycle> active_status;

    // Use for animation of the simulation as the cells in the CCA chip become active and inactive
    std::vector<std::shared_ptr<u_int32_t[]>> individual_cells_active_status_per_cycle;
};

class CCASimulator
{
  public:
    computeCellShape shape_of_compute_cells;

    // Dimensions of the CCA chip.
    u_int32_t dim_x, dim_y;

    // Dimensions and depth of the Htree. Here hx and hy are the dimensions of the block of cells
    // covered by a single end node of the Htree.
    u_int32_t hx, hy, hdepth;
    // Max possible lanes in the htree joints
    u_int32_t hbandwidth_max;

    // Total CCA Cells (including Sink Cells)
    u_int32_t total_compute_cells;

    // Memory per compute cell and the total combined memory of this CCA chip
    u_int32_t memory_per_cc;
    u_int32_t total_chip_memory;

    // Declare the CCA Chip that is composed of Compute Cell(s) and any SinkCell(s)
    std::vector<std::shared_ptr<Cell>> CCA_chip;

    // The routing policy and algorithms id for the mesh network
    u_int32_t mesh_routing_policy_id;

    // ID of the host. Just like Compute Cells have Id the host also has. This is useful for
    // invoking functions on objects in the host memory. For example: The root terminator form the
    // host.
    u_int32_t host_id;

    // The host memory
    std::shared_ptr<char[]> host_memory;

    // Memory of host in bytes. Used to store objects at host such as the root terminator provided
    // by the user.
    static inline constexpr u_int32_t host_memory_size_in_bytes = 2048; // 2 KB
    char* host_memory_raw_ptr;
    char* host_memory_curr_ptr;

    // For statistics
    bool global_active_cc;
    u_long total_cycles;

    // Seconds layer network
    HtreeNetwork htree_network;

    // Registers and manages the function events
    FunctionEventManager function_events;

    // To record various measurements of the system to be used for analysis
    CCASimulatorStatistics cca_statistics;

    CCASimulator(computeCellShape shape_in,
                 u_int32_t dim_x_in,
                 u_int32_t dim_y_in,
                 u_int32_t hx_in,
                 u_int32_t hy_in,
                 u_int32_t hdepth_in,
                 u_int32_t hbandwidth_max_in,
                 u_int32_t total_compute_cells_in,
                 u_int32_t memory_per_cc_in,
                 u_int32_t mesh_routing_policy_id_in)
        : shape_of_compute_cells(shape_in)
        , dim_x(dim_x_in)
        , dim_y(dim_y_in)
        , hx(hx_in)
        , hy(hy_in)
        , hdepth(hdepth_in)
        , hbandwidth_max(hbandwidth_max_in)
        , total_compute_cells(total_compute_cells_in)
        , memory_per_cc(memory_per_cc_in)
        , mesh_routing_policy_id(mesh_routing_policy_id_in)
        , htree_network(hx_in, hy_in, hdepth_in, hbandwidth_max_in)
    {
        this->global_active_cc = false;
        this->total_cycles = 0;
        this->total_chip_memory = this->total_compute_cells * this->memory_per_cc;

        this->host_id = this->dim_x * this->dim_y;
        this->host_memory = std::make_shared<char[]>(this->host_memory_size_in_bytes);
        this->host_memory_raw_ptr = this->host_memory.get();
        this->host_memory_curr_ptr = this->host_memory_raw_ptr;

        this->create_the_chip();

        assert(this->host_id == this->CCA_chip.size());
    }

    inline void generate_label(std::ostream& os)
    {
        os << "shape\tdim_x\tdim_y\thx\thy\thdepth\thbandwidth_max\ttotal_compute_cells\ttotal_"
              "chip_memory(byes)\n";
    }

    inline void output_description_in_a_single_line(std::ostream& os)
    {
        os << ComputeCell::get_compute_cell_shape_name(this->shape_of_compute_cells) << "\t"
           << this->dim_x << "\t" << this->dim_y << "\t" << this->hx << "\t" << this->hy << "\t"
           << this->hdepth << "\t" << this->hbandwidth_max << "\t" << this->total_compute_cells
           << "\t" << this->total_chip_memory << "\n";
    }

    inline void output_CCA_active_status_per_cycle(std::ostream& os)
    {
        os << "Cycle#\tCells_Active_Percent\tHtree_Active_Percent\n";
        for (size_t i = 0; i < this->cca_statistics.active_status.size(); i++) {
            os << i << "\t" << this->cca_statistics.active_status[i].cells_active_percent << "\t"
               << this->cca_statistics.active_status[i].htree_active_percent << "\n";
        }
    }

    inline void output_CCA_active_status_per_cell_cycle(std::ostream& os)
    {
        os << "Cycles\tDim_x\tDim_y\n";
        os << this->total_cycles << "\t" << this->dim_x << "\t" << this->dim_y << "\n";
        os << "Active_Status_Per_Cell_Per_Cycle\n";
        for (size_t i = 0; i < this->cca_statistics.individual_cells_active_status_per_cycle.size();
             i++) {
            std::shared_ptr<u_int32_t[]> frame =
                this->cca_statistics.individual_cells_active_status_per_cycle[i];
            for (u_int32_t rows = 0; rows < this->dim_x; rows++) {
                for (u_int32_t cols = 0; cols < this->dim_y; cols++) {
                    os << frame[rows * this->dim_y + cols];
                    if (cols != this->dim_y - 1) {
                        os << " ";
                    }
                }
                if (rows != this->dim_x - 1) {
                    os << ", ";
                }
            }
            if (i != this->cca_statistics.individual_cells_active_status_per_cycle.size() - 1) {
                os << "\n";
            }
        }
    }

    inline Coordinates get_compute_cell_coordinates(u_int32_t cc_id, u_int32_t dim_y);

    Coordinates cc_id_to_cooridinate(u_int32_t cc_id);

    u_int32_t cc_cooridinate_to_id(Coordinates cc_cooridinate);

    // The main chip creation function
    void create_the_chip();

    // Register a function event
    CCAFunctionEvent register_function_event(handler_func function_event_handler);

    // Return the memory used in bytes
    u_int32_t get_host_memory_used();
    // In bytes
    u_int32_t get_host_memory_curr_ptr_offset();
    // Get memory left in bytes
    u_int32_t host_memory_available_in_bytes();

    // Create a CCATerminator object on host and return the address
    std::optional<Address> create_terminator();

    // Check for termination of the diffusion
    bool is_diffusion_active(Address terminator_in);

    std::optional<Address> allocate_and_insert_object_on_cc(u_int32_t cc_id,
                                                            void* obj,
                                                            size_t size_of_obj);

    void run_simulation(Address app_terminator);

    // Get the pointer to the object at `Address addr_in`
    void* get_object(Address addr_in) const;

  private:
    // Create the chip. It includes creating the cells and initializing them with IDs and their
    // types and more
    void create_square_cell_htree_chip();

    // Create the chip of type square cells with only mesh connetion. There is not htree or any
    // second layer network involved. It includes creating the cells and initializing them with IDs
    // and their types and more
    void create_square_cell_mesh_only_chip();
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
        // Skip the Cell if it is not of type ComputeCell
        while (cca_simulator.CCA_chip[this->next_cc_id]->type != CellType::compute_cell) {
            this->next_cc_id = (this->next_cc_id + 1) % cca_simulator.total_compute_cells;
        }
        u_int32_t cc_available = this->next_cc_id;
        this->next_cc_id = (this->next_cc_id + 1) % cca_simulator.total_compute_cells;
        return cc_available;
    }
};

#endif // CCASimulator_HPP

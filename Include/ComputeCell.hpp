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

#ifndef COMPUTE_CELL_HPP
#define COMPUTE_CELL_HPP

#include "Action.hpp"
#include "Address.hpp"
#include "Cell.hpp"
#include "Constants.hpp"
#include "Operon.hpp"
#include "Task.hpp"

#include "memory_management.hpp"

#include <map>
#include <optional>
#include <queue>
#include <stdlib.h>

// Note this class is not thread-safe.
class ComputeCell : public Cell
{
  public:
    // Get the object memory location at address addr_in
    void* get_object(Address addr_in) const;

    // Return the memory used in bytes
    u_int32_t get_memory_used();

    // In bytes
    u_int32_t get_memory_curr_ptr_offset();

    // Get memory left in bytes
    u_int32_t memory_available_in_bytes();

    // Returns the offset in memory for this newly created object
    std::optional<Address> create_object_in_memory(void* obj, size_t size_of_obj);

    void insert_action(const Action& action);

    void execute_action();

    // Prepare the cycle. This involves moving operon data into either the action queue or send
    // buffers of the network links
    void prepare_a_cycle();

    // Execute a single cycle for this cell
    void run_a_computation_cycle();

    // TODO: write comments
    void prepare_a_communication_cycle();

    // TODO: write comments
    void run_a_communication_cycle(std::vector<std::shared_ptr<Cell>>& CCA_chip);

    // Checks if the cell is active or not
    bool is_compute_cell_active();

    // This is also needed to satify the simulation as the network and logic on a single compute
    // cell both work in paralell. We first perform logic operations (work) then we do networking
    // related operations. This allows not just ease of programming but also opens the compute cells
    // to be embarasingly parallel for openmp.
    std::optional<Operon> staging_operon_from_logic;

    // Routing
    // Based on the routing algorithm and the shape of CCs it will return which neighbor to pass
    // this operon to. The returned value is the index [0...number of neighbors) coresponding
    // clockwise the channel id of the physical shape.
    u_int32_t get_route_towards_cc_id(u_int32_t dst_cc_id);

    // Sink cell nearby this compute cell. Used to route operons that are sent to far flung compute
    // cells in the CCA chip. If the network is mesh only then there is no sink cell hence the use
    // of `std::optional`
    std::optional<Coordinates> sink_cell;

    // Memory of the Compute Cell in bytes.
    u_int32_t memory_size_in_bytes;

    // The memory
    std::unique_ptr<char[]> memory;
    char* memory_raw_ptr;
    char* memory_curr_ptr;

    // Actions queue of the Compute Cell
    std::queue<Action> action_queue;

    // TODO: maybe later make a function like this that gets from the queue in an intelligent matter
    // or depending on the policy. So it can be both FIFO and LIFO, maybe something even better
    // std::shared_ptr<Action> get_an_action();

    // Tasks for the Compute Cell. These tasks exist only for the simulator and are not part of the
    // actual internals of any Compute Cell.
    std::queue<Task> task_queue;

    // Constructor
    ComputeCell(u_int32_t id_in,
                CellType type_in,
                computeCellShape shape_in,
                u_int32_t dim_x_in,
                u_int32_t dim_y_in,
                u_int32_t hx_in,
                u_int32_t hy_in,
                u_int32_t hdepth_in,
                u_int32_t memory_per_cc_in_bytes)
    {
        this->id = id_in;
        this->type = type_in;
        this->statistics.type = this->type;

        this->shape = shape_in;
        this->number_of_neighbors = ComputeCell::get_number_of_neighbors(this->shape);

        this->dim_x = dim_x_in;
        this->dim_y = dim_y_in;

        this->hx = hx_in;
        this->hy = hy_in;
        this->hdepth = hdepth_in;

        this->cooridates = ComputeCell::cc_id_to_cooridinate(this->id, this->shape, this->dim_y);

        this->sink_cell = this->get_cc_htree_sink_cell();

        this->memory_size_in_bytes = memory_per_cc_in_bytes;
        this->memory = std::make_unique<char[]>(this->memory_size_in_bytes);
        this->memory_raw_ptr = memory.get();
        this->memory_curr_ptr = memory_raw_ptr;

        // Assign neighbor CCs to this CC. This is based on the Shape and Dim
        this->add_neighbor_compute_cells();

        this->staging_operon_from_logic = std::nullopt;
        for (u_int32_t i = 0; i < this->number_of_neighbors; i++) {
            this->send_channel_per_neighbor.push_back(std::nullopt);
            this->recv_channel_per_neighbor.push_back(std::nullopt);
        }
        // Start from 0th and then alternate by % 4 (here 4 = number of neighbers for square cell
        // type for example)
        this->current_recv_channel_to_start_a_cycle = 0;
    }

  private:
    // Each compute cell has a sink cell configured such that when it has to send an operon to far
    // flung compute cell it routes to the Htree network and has to sink the operon into the sink
    // cell that is nearby
    std::optional<Coordinates> get_cc_htree_sink_cell();
};

#endif // COMPUTE_CELL_HPP

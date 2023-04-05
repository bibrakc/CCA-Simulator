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
#include "Constants.hpp"
#include "Operon.hpp"
#include "Task.hpp"

#include "memory_management.hpp"

#include <map>
#include <queue>
#include <stdlib.h>

inline constexpr u_int32_t edges_max = 2;
struct SimpleVertex
{
    u_int32_t id;
    Address edges[edges_max];
    u_int32_t number_of_edges;
    u_int32_t sssp_distance;
};

enum class computeCellShape : u_int32_t
{
    block_1D = 0,
    triangular,
    sqaure,
    hexagon,
    computeCellShape_count
};

static std::map<computeCellShape, u_int32_t> computeCellShape_num_channels = {
    { computeCellShape::block_1D, 2 },
    { computeCellShape::triangular, 3 },
    { computeCellShape::sqaure, 4 },
    { computeCellShape::hexagon, 6 }
};

// Note this class is not thread-safe.
class ComputeCell
{
  public:
    void* get_object(Address addr_in) const { return (this->memory.get() + addr_in.addr); }

    // Return the memory used in bytes
    u_int32_t get_memory_used() { return this->memory_curr_ptr - this->memory_raw_ptr; }

    // In bytes
    u_int32_t get_memory_curr_ptr_offset() { return get_memory_used(); }

    // Get memory left in bytes
    u_int32_t memory_available_in_bytes() { return this->memory_size_in_bytes - get_memory_used(); }

    // Returns the offset in memory for this newly created object
    template<typename T>
    std::optional<Address> create_object_in_memory(T obj)
    {
        if (this->memory_available_in_bytes() < sizeof(T)) {
            return std::nullopt;
        }

        u_int32_t obj_memory_addr_offset = get_memory_curr_ptr_offset();
        memcpy(this->memory_curr_ptr, &obj, sizeof(T));
        this->memory_curr_ptr = this->memory_curr_ptr + sizeof(T);

        return Address(this->id, obj_memory_addr_offset);
    }

    void insert_action(const std::shared_ptr<Action>& action) { this->action_queue.push(action); }

    void execute_action();

    // Execute a single cycle for this Compute Cell
    // Return whether this compute cell is still active, meaning run_a_cycle needs to be called
    // again
    bool run_a_cycle();

    // Checks if the compute cell is active or not
    // TODO: when communication is added then update checks for the communication buffer too
    bool is_compute_cell_active()
    {
        return (!this->action_queue.empty() || !this->task_queue.empty());
    }

    void add_neighbor(u_int32_t neighbor_compute_cell_id)
    {
        this->neighbor_compute_cells.push_back(neighbor_compute_cell_id);
    }

    // Identity of the Compute Cell
    u_int32_t id;

    // Communication of the Compute Cell

    // number_of_neighbors is the maximum number of connections a single Compute Cell can
    // architecturally have. A triangular CC has 3 neighbors, a square has 4, and hexagon has 6 etc.
    // TODO: Later read the shape from a config file and initialize it in the constructor
    u_int32_t number_of_neighbors;

    // IDs of the neighbors
    std::vector<u_int32_t> neighbor_compute_cells;
    // Per neighbor operon recieve queue.
    std::vector<std::queue<Operon>> channel_buffer_per_neighbor;

    // This is needed to satisty simulation. Because a sending CC can not just enqueue an operon
    // into the current working buffer of a neighbor CC. If it does then the neighbor may start
    // computation (or work) on that operon in the current simulation cycle. The receiving neighbor
    // must process it in the next cycle. Therefore, this send/recv buffer serves that purpose. At
    // the end of each simulation cycle each CC will move an operon from this buffer to its working
    // set of operons and will then execute those operons in the next cycle. This move is not part
    // of the computation but is only there for simulation so as not to break the
    // semantics/pragmatics of CCA.
    std::vector<Operon> send_recv_channel_buffer_per_neighbor;

    // Memory of the Compute Cell in bytes
    static constexpr u_int32_t memory_size_in_bytes = 2 * 1024 * 1024; // 2 MB
    std::unique_ptr<char[]> memory;
    char* memory_raw_ptr;
    char* memory_curr_ptr;

    // Actions queue of the Compute Cell
    std::queue<std::shared_ptr<Action>> action_queue;

    // TODO: maybe later make a function like this that gets from the queue in an intelligent matter
    // or depending on the policy. So it can be both FIFO and LIFO, maybe even better
    // std::shared_ptr<Action> get_an_action();

    // Tasks for the Compute Cell. These tasks exist only for this simulator and are not part of the
    // actual internals of any Compute Cell.
    // TODO:
    // std::queue<std::shared_ptr<Task>> task_queue;
    // TaskQueue task_queue;
    std::queue<Task> task_queue;

    // Constructor
    ComputeCell(u_int32_t id_in, computeCellShape shape)
    {
        /* cout << "Inside constructor of ComputeCell\n"; */

        this->id = id_in;
        this->number_of_neighbors = computeCellShape_num_channels[shape];
        // std::cout << "this->number_of_neighbors = " << this->number_of_neighbors << "\n";

        this->memory = std::make_unique<char[]>(this->memory_size_in_bytes);
        this->memory_raw_ptr = memory.get();
        this->memory_curr_ptr = memory_raw_ptr;
    }

    /* ~ComputeCell() { cout << "Inside destructor of ComputeCell\n"; } */
};

#endif // COMPUTE_CELL_HPP

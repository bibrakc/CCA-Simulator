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

#include "Cell.hpp"

#include "Object.hpp"
#include "Task.hpp"
#include <utility>

// Note this class is not thread-safe.
class ComputeCell : public Cell
{
  public:
    // Get the object memory location at address addr_in
    [[nodiscard]] auto get_object(Address addr_in) const -> void*;

    // Return the memory used in bytes
    auto get_memory_used() -> u_int32_t;

    // In bytes
    auto get_memory_curr_ptr_offset() -> u_int32_t;

    // Get memory left in bytes
    auto memory_available_in_bytes() -> u_int32_t;

    // Returns the offset in memory for this newly created object
    auto create_object_in_memory(void* obj, size_t size_of_obj) -> std::optional<Address>;

    [[nodiscard]] auto insert_action(const Action& action, bool priority) -> bool;

    void execute_action(void* function_events);
    void execute_diffusion_phase(void* function_events);
    void filter_diffusion(void* function_events);

    // Prepare the cycle. This involves moving operon data into either the action queue or send
    // buffers of the network links
    void prepare_a_cycle(std::vector<std::shared_ptr<Cell>>& CCA_chip);

    // Execute a single cycle for this cell
    void run_a_computation_cycle(std::vector<std::shared_ptr<Cell>>& CCA_chip,
                                 void* function_events) override;

    // TODO: write comments
    void prepare_a_communication_cycle(std::vector<std::shared_ptr<Cell>>& CCA_chip) override;

    // TODO: write comments
    void run_a_communication_cycle(std::vector<std::shared_ptr<Cell>>& CCA_chip) override;

    // Checks if the cell is active or not
    auto is_compute_cell_active() -> u_int32_t override;

    // Send an Operon. Create a task that when invoked on a Compute Cell it simply puts the operon
    // on the `staging_operon_from_logic`
    auto send_operon(const Operon& operon_in) -> Task;

    // Construct an Operon
    static auto construct_operon(u_int32_t src_cc_id, u_int32_t dst_cc_id, const Action& action)
        -> Operon;

    void diffuse(const Action& action);

    // This is also needed to satify the simulation as the network and logic on a single compute
    // cell both work in paralell. We first perform logic operations (work) then we do networking
    // related operations. This allows not just ease of programming but also opens the compute cells
    // to be embarasingly parallel for openmp.
    std::optional<Operon> staging_operon_from_logic;

    // Sink cell nearby this compute cell. Used to route operons that are sent to far flung compute
    // cells in the CCA chip. If the network is mesh only then there is no sink cell hence the use
    // of `std::optional`
    std::optional<Coordinates> sink_cell;

    // Used in predicate and work functions of app as they return Closures.
    CCAFunctionEvent null_false_event;
    CCAFunctionEvent null_true_event;

    // Memory of the Compute Cell in bytes.
    u_int32_t memory_size_in_bytes;

    // The memory
    std::unique_ptr<char[]> memory;
    char* memory_raw_ptr;
    char* memory_curr_ptr;

    std::shared_ptr<char[]> host_memory;
    u_int32_t host_id;

    // Actions queue of the Compute Cell
    FixedSizeQueue<Action> action_queue;

    // Diffusion Queue of the Compute Cell that holds the diffusive sections of the action. An
    // action comes into the `action_queue`, where it gets invoked. When it is invoked the diffusion
    // part is not immediately executed but rather it is sent to the `diffuse_queue` and is later
    // scheduled for execution. This separation helps in deadlock avoidance on queues in that if
    // there were a single queue it may becoem full and new actions couldnt be generated for that CC
    // by that CC. It may also improve task scheduling, depending on how it is managed. Need to be
    // clever.
    FixedSizeQueue<Action> diffuse_queue;

    // bool prefer_diffuse_queue{};

    // TODO: maybe later make a function like this that gets from the queue in an intelligent matter
    // or depending on the policy. So it can be both FIFO and LIFO, maybe something even better
    // std::shared_ptr<Action> get_an_action();

    // Tasks for the Compute Cell. These tasks exist only for the simulator and are not part of the
    // actual internals of any Compute Cell. It functions to simulate tasks cycle by cycle thus
    // making the simulation more accurate/realistic.
    std::queue<Task> task_queue;

    // Constructor
    ComputeCell(u_int32_t id_in,
                CellType type_in,
                computeCellShape shape_in,
                CCAFunctionEvent null_false_event_in,
                CCAFunctionEvent null_true_event_in,
                u_int32_t dim_x_in,
                u_int32_t dim_y_in,
                u_int32_t hx_in,
                u_int32_t hy_in,
                u_int32_t hdepth_in,
                u_int32_t memory_per_cc_in_bytes,
                std::shared_ptr<char[]> host_memory_in,
                u_int32_t primary_network_type_in,
                u_int32_t mesh_routing_policy_id_in)
    {

        this->id = id_in;
        this->type = type_in;
        this->statistics.type = this->type;

        this->shape = shape_in;
        this->number_of_neighbors = ComputeCell::get_number_of_neighbors(this->shape);

        this->null_false_event = null_false_event_in;
        this->null_true_event = null_true_event_in;

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

        this->host_memory = std::move(host_memory_in);
        this->host_id = this->dim_x * this->dim_y;

        // Torus or Mesh?
        this->primary_network_type = primary_network_type_in;

        // Assign neighbor CCs to this CC. This is based on the Shape and Dim.
        this->add_neighbor_compute_cells();

        this->staging_operon_from_logic = std::nullopt;

        this->mesh_routing_policy = mesh_routing_policy_id_in;

        // this->distance_class_length = 2; //(this->hx * 15) + (this->hy * 15);
        this->number_of_virtual_channels = 4; // To avoid deadlock, espesially in Torus routing.

        this->recv_channel_per_neighbor.resize(
            this->number_of_neighbors,
            std::vector<FixedSizeQueue<Operon>>(this->number_of_virtual_channels,
                                                FixedSizeQueue<Operon>(buffer_size)));

        // send channel buffer can only hold one operon since its there to put send (put) in the
        // recv channel of the neighbor.
        this->send_channel_per_neighbor.resize(
            this->number_of_neighbors,
            std::vector<FixedSizeQueue<Operon>>(this->number_of_virtual_channels,
                                                FixedSizeQueue<Operon>(1)));

        // this->send_channel_per_neighbor_current_distance_class.resize(this->number_of_neighbors);
        this->send_channel_per_neighbor_contention_count.resize(this->number_of_neighbors,
                                                                MaxCounter());

        // Start from 0th and then alternate by % 4 (here 4 = number of neighbers for square cell
        // type for example).
        this->current_recv_channel_to_start_a_cycle = 0;
        this->last_congested_cycle = std::nullopt;

        // Experimental. The cells don't have sense of a global cycle. It is here for debuging and
        // making the implementation of the simulator easier such as throttling.
        this->current_cycle = 0;

        // edges_max + 2 + 5 = helps to have enough buffer (and not deadlock) for the CC to push
        // actions onto itself in case of a diffusion that requires pushing to itself.
        // 2: ghost edges, 5: just because.
        this->action_queue = FixedSizeQueue<Action>(action_queue_size, edges_max + 2 + 5);
        this->diffuse_queue = FixedSizeQueue<Action>(diffuse_queue_size);

        // Experimental for scheduling.
        // this->prefer_diffuse_queue = false;
    }

    ~ComputeCell() override = default;

  private:
    // Each compute cell has a sink cell configured such that when it has to send an operon to far
    // flung compute cell it routes to the Htree network and has to sink the operon into the sink
    // cell that is nearby.
    auto get_cc_htree_sink_cell() -> std::optional<Coordinates>;
};

#endif // COMPUTE_CELL_HPP

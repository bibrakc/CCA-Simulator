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

typedef std::pair<int32_t, int32_t> SignedCoordinates;

enum class computeCellShape : u_int32_t
{
    block_1D = 0,
    triangular,
    square, // block 2D
    hexagon,
    computeCellShape_invalid
};

// TODO: Currently, decided to not use this and use a function
// (ComputeCell::get_number_of_neighbors()) that returns the number of neighbors. This map was
// designed to offer flexibity when the simulator (if or may) get converted to a compiled library
// and the user adds new shapes.
/* static std::map<computeCellShape, u_int32_t> computeCellShape_num_channels = {
    { computeCellShape::block_1D, 2 },
    { computeCellShape::triangular, 3 },
    { computeCellShape::sqaure, 4 },
    { computeCellShape::hexagon, 6 }
}; */

struct ComputeCellStatistics
{
    u_int32_t actions_created{};
    u_int32_t actions_pushed{};

    u_int32_t actions_invoked{};
    u_int32_t actions_false_on_predicate{}; // actions subsumed
    u_int32_t stall_logic_on_network{};     // When network is busy passing other operon
    u_int32_t stall_network_on_recv{};
    u_int32_t stall_network_on_send{};

    // u_int32_t cycles_active{};

    friend std::ostream& operator<<(std::ostream& os, const ComputeCellStatistics& stat)
    {
        os << "Statistics:\n\tactions_created: " << stat.actions_created
           << "\n\tactions_pushed: " << stat.actions_pushed
           << "\n\tactions_invoked: " << stat.actions_invoked
           << "\n\tactions_false_on_predicate: " << stat.actions_false_on_predicate
           << "\n\tstall_logic_on_network: " << stat.stall_logic_on_network
           << "\n\tstall_network_on_recv: " << stat.stall_network_on_recv
           << "\n\tstall_network_on_send: " << stat.stall_network_on_send << "\n";
        return os;
    }
};

// Note this class is not thread-safe.
// TODO: We can have a template for computeCellShape here
class ComputeCell
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

    friend std::ostream& operator<<(std::ostream& os, const ComputeCell& cc)
    {
        os << "CC Id: " << cc.id << ", CC Coordinates: (" << cc.cooridates.first << ", "
           << cc.cooridates.second << ")\n";

        os << "\t Neighbors: ";
        for (auto& neighbor : cc.neighbor_compute_cells) {
            if (neighbor == std::nullopt) {
                os << "[nullopt] ";
            } else {
                auto [neighbor_id, neighbor_coordinate] = neighbor.value();
                os << "[" << neighbor_id << ", (" << neighbor_coordinate.first << ", "
                   << neighbor_coordinate.second << ")] ";
            }
        }
        os << "\n";
        return os;
    }

    void insert_action(const std::shared_ptr<Action>& action);

    void execute_action();

    // Prepare the cycle. This involves moving operon data into either the action queue or send
    // buffers of the network links
    void prepare_a_cycle();

    // Execute a single cycle for this Compute Cell
    // Return whether this compute cell is still active, meaning run_a_cycle needs to be called
    // again
    void run_a_computation_cycle();

    // TODO: write comments
    void prepare_a_communication_cycle();

    // TODO: write comments
    void run_a_communication_cycle(std::vector<std::shared_ptr<ComputeCell>>& CCA_chip);

    // Run a cycle: This include all computation and communication work within a single cycle
    // bool run_a_cycle();

    // Checks if the compute cell is active or not
    // TODO: when communication is added then update checks for the communication buffer too
    bool is_compute_cell_active();

    inline bool cc_exists(const SignedCoordinates cc_coordinate);

    void add_neighbor_compute_cells();

    void add_neighbor(
        std::optional<std::pair<u_int32_t, std::pair<u_int32_t, u_int32_t>>> neighbor_compute_cell);

    static std::string get_compute_cell_shape_name(computeCellShape shape);

    static computeCellShape get_compute_cell_shape_enum(std::string shape);

    static u_int32_t get_number_of_neighbors(computeCellShape);

    static std::pair<u_int32_t, u_int32_t> cc_id_to_cooridinate(u_int32_t cc_id,
                                                                computeCellShape shape,
                                                                u_int32_t dim_x,
                                                                u_int32_t dim_y);

    static u_int32_t cc_cooridinate_to_id(std::pair<u_int32_t, u_int32_t> cc_cooridinate,
                                          computeCellShape shape_,
                                          u_int32_t dim_x,
                                          u_int32_t dim_y);

    // Identity of the Compute Cell.
    u_int32_t id;

    // Coordinates of this Compute Cell in the CCA chip. It depends on the Chip dinemsions and
    // shapes of CCs.
    std::pair<u_int32_t, u_int32_t> cooridates;

    // Shape of the Compute Cell
    computeCellShape shape;

    // Dimensions of the CCA chip this compute cell belongs to. This is needed for routing as the
    // routing logic is implemented inside the compute cells and they need to be aware of the global
    // chip configuration.
    u_int32_t dim_x;
    u_int32_t dim_y;

    // Communication of the Compute Cell.

    // number_of_neighbors is the maximum number of connections a single Compute Cell can
    // architecturally have. A triangular CC has 3 neighbors, a square has 4, and hexagon has 6 etc.
    // TODO: Later read the shape from a config file and initialize it in the constructor
    // TODO: This seems to be redundant unless we really want to discriminate the std::nullopt,
    // later
    u_int32_t number_of_neighbors;

    // IDs and Coordinates of the neighbors
    std::vector<std::optional<std::pair<u_int32_t, std::pair<u_int32_t, u_int32_t>>>>
        neighbor_compute_cells;

    // Channel neighbor id index. To be initialized based on the shape of this CC and the dimensions
    // of the chip. Note: the channels are enumerated (indexed) clockwise starting from left. 0 =
    // left, 1 = up, 2 = right, and 3 = down for sqaure shape.
    std::vector<std::optional<u_int32_t>> channel_neighbor_index;

    // Per neighbor send channel/link
    std::vector<std::optional<Operon>> send_channel_per_neighbor;

    // This is needed to satisty simulation. Because a sending CC can not just enqueue an operon
    // into the current working buffer of a neighbor CC. If it does then the neighbor may start
    // computation (or work) on that operon in the current simulation cycle. The receiving neighbor
    // must process it in the next cycle. Therefore, this send/recv buffer serves that purpose. At
    // the end of each simulation cycle each CC will move an operon from this buffer to its working
    // set of operons and will then execute those operons in the next cycle. This move is not part
    // of the computation but is only there for simulation so as not to break the
    // semantics/pragmatics of CCA.
    std::vector<std::optional<Operon>> recv_channel_per_neighbor;

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

    // Memory of the Compute Cell in bytes.
    // TODO: This can be `static` since it is a set once and real-only and is the same for all CCs.
    u_int32_t memory_size_in_bytes;

    // The memory
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

    // Performance measurements and counters
    ComputeCellStatistics statistics;

    // Constructor
    ComputeCell(u_int32_t id_in,
                computeCellShape shape_in,
                u_int32_t dim_x_in,
                u_int32_t dim_y_in,
                u_int32_t memory_per_cc_in_bytes)
    {
        /* cout << "Inside constructor of ComputeCell\n"; */

        this->id = id_in;
        this->shape = shape_in;
        this->number_of_neighbors = ComputeCell::get_number_of_neighbors(this->shape);

        this->dim_x = dim_x_in;
        this->dim_y = dim_y_in;

        this->cooridates =
            ComputeCell::cc_id_to_cooridinate(this->id, this->shape, this->dim_x, this->dim_y);

        this->memory_size_in_bytes = memory_per_cc_in_bytes;
        this->memory = std::make_unique<char[]>(this->memory_size_in_bytes);
        this->memory_raw_ptr = memory.get();
        this->memory_curr_ptr = memory_raw_ptr;

        // Assign neighbor CCs to this CC. This is based on the Shape and Dim
        this->add_neighbor_compute_cells();

        this->staging_operon_from_logic = std::nullopt;
        for (int i = 0; i < this->number_of_neighbors; i++) {
            this->send_channel_per_neighbor.push_back(std::nullopt);
            this->recv_channel_per_neighbor.push_back(std::nullopt);
        }

        // TODO remove the channel_neighbor_index
        /*         if (this->shape == computeCellShape::square) {

                    // Initialize the index of the channels clockwise from the left with the cc
           neighbor Ids this->channel_neighbor_index[0] = left neighbor;
                             this->channel_neighbor_index[0] = up neighbor;
                             this->channel_neighbor_index[0] = right neighbor;
                             this->channel_neighbor_index[0] = down neighbor;

                } else {
                    // Shape  not supported
                    std::cerr << ComputeCell::get_compute_cell_shape_name(this->shape)
                              << "  not supported!\n";
                    exit(0);
                } */
    }
};

#endif // COMPUTE_CELL_HPP

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

#ifndef CELL_HPP
#define CELL_HPP

#include "Constants.hpp"
#include "Operon.hpp"

#include <optional>

using SignedCoordinates = std::pair<int32_t, int32_t>;

// Threshold value after which it is considered to be congested. Used for creating active status
// values for statistics and animation.
constexpr u_int32_t congestion_threshold_1 = 1;
constexpr u_int32_t congestion_threshold_2 = 15;
constexpr u_int32_t congestion_threshold_3 = 30;
constexpr u_int32_t congestion_threshold_4 = 60;

// Type of the Cell: ComputeCell or HtreeNode
enum class CellType : u_int32_t
{
    compute_cell = 0,
    sink_cell, // Htree node connection point
    CellType_invalid
};

// Shape of the ComputeCell
enum class computeCellShape : u_int32_t
{
    block_1D = 0, // TODO: remove this
    triangular,
    square, // block 2D
    hexagon,
    computeCellShape_invalid
};

struct ComputeCellStatistics
{
    // Total actions created includes both application actions and system actions such as
    // termination acknowledgement actions.
    u_int32_t actions_created{};

    // Count of ack actions created.
    u_int32_t actions_acknowledgement_created{};

    // Both application actions and termination acknowledgement actions.
    u_int32_t actions_pushed{};
    u_int32_t actions_invoked{};

    u_int32_t actions_acknowledgement_invoked{};

    u_int32_t actions_performed_work{};

    // # of Actions subsumed
    u_int32_t actions_false_on_predicate{};

    std::vector<MaxCounter> send_channel_per_neighbor_contention_count_record;

    // # of objects allocated
    u_int32_t objects_allocated{};

    // Type of the Cell: ComputeCell or Htree node? For which these statistics were taken
    CellType type;

    static inline void generate_label(std::ostream& os)
    {
        os << "cc_id\tcc_type\tcc_coordinate_x\tcc_coordinate_y\tobjects_allocated"
              "\tactions_created\tactions_acknowledgement_created"
              "\tactions_pushed\tactions_invoked\tactions_performed_work"
              "\tactions_acknoledgement_invoked\tactions_false_on_predicate"
              "\tleft_send_contention_max\tleft_send_contention_total"
              "\tup_send_contention_max\tup_send_contention_total"
              "\tright_send_contention_max\tright_send_contention_total"
              "\tdown_send_contention_max\tdown_send_contention_total\n";
    }

    // Print all the stats in a single line
    void output_results_in_a_single_line(std::ostream& os,
                                         u_int32_t cc_id,
                                         Coordinates cc_cooridinates);

    // Overloading <<
    friend auto operator<<(std::ostream& os, const ComputeCellStatistics& stat) -> std::ostream&
    {
        os << "Statistics:"
           << "\n\tobjects_allocated: " << stat.objects_allocated
           << "\n\tactions_created: " << stat.actions_created
           << "\n\tactions_acknowledgement_created: " << stat.actions_acknowledgement_created
           << "\n\tactions_pushed: " << stat.actions_pushed
           << "\n\tactions_invoked: " << stat.actions_invoked
           << "\n\tactions_performed_work: " << stat.actions_performed_work
           << "\n\tactions_acknowledgement_invoked: " << stat.actions_acknowledgement_invoked
           << "\n\tactions_false_on_predicate: " << stat.actions_false_on_predicate << "\n";
        return os;
    }

    auto operator+=(const ComputeCellStatistics& rhs) -> ComputeCellStatistics&
    {
        this->objects_allocated += rhs.objects_allocated;
        this->actions_created += rhs.actions_created;
        this->actions_acknowledgement_created += rhs.actions_acknowledgement_created;
        this->actions_pushed += rhs.actions_pushed;
        this->actions_invoked += rhs.actions_invoked;
        this->actions_performed_work += rhs.actions_performed_work;
        this->actions_acknowledgement_invoked += rhs.actions_acknowledgement_invoked;
        this->actions_false_on_predicate += rhs.actions_false_on_predicate;

        return *this;
    }
};

// Base class for Cells: These cells can be regular compute cells or nodes to the secondary network
// such as a Htree
class Cell
{
  public:
    // Identity of the Cell.
    u_int32_t id;

    // Type of the Cell: ComputeCell or Htree node?
    CellType type;

    // Coordinates of this Cell in the CCA chip. It depends on the Chip dinemsions and
    // shapes of Cells.
    Coordinates cooridates;

    // Shape of the Cell
    computeCellShape shape;

    // Dimensions of the CCA chip this cell belongs to. This is needed for routing as the routing
    // logic is implemented inside the cells and they need to be aware of the global chip
    // configuration.
    u_int32_t dim_x;
    u_int32_t dim_y;

    // Dimensions and depth of the Htree. Here hx and hy are the dimensions of the block of cells
    // covered by a single end node of the Htree.
    u_int32_t hx, hy, hdepth;

    // Communication of the Cell.

    // number_of_neighbors is the maximum number of connections a single Cell can architecturally
    // have. A triangular Cell has 3 neighbors, a square has 4, and hexagon has 6 etc.
    // TODO: This seems to be redundant unless we really want to discriminate the std::nullopt,
    // later
    u_int32_t number_of_neighbors;

    // IDs and Coordinates of the neighbors
    std::vector<std::optional<std::pair<u_int32_t, Coordinates>>> neighbor_compute_cells;

    // Per neighbor send channel/link
    std::vector<FixedSizeQueue<Operon>> send_channel_per_neighbor;
    std::vector<u_int32_t> send_channel_per_neighbor_current_distance_class;

    // Use this to detect deadlock
    std::vector<MaxCounter> send_channel_per_neighbor_contention_count;

    // This is needed to satisty simulation. Because a sending Cell can not just enqueue an operon
    // into the current working buffer of a neighbor Cell. If it does then the neighbor may start
    // computation (or work) on that operon in the current simulation cycle. The receiving neighbor
    // must process it in the next cycle. Therefore, this send/recv buffer serves that purpose. At
    // the end of each simulation cycle each Cell will move an operon from this buffer to its
    // working set of operons and will then execute those operons in the next cycle. This move is
    // not part of the computation but is only there for simulation so as not to break the
    // semantics/pragmatics of CCA.
    std::vector<std::vector<FixedSizeQueue<Operon>>> recv_channel_per_neighbor;

    // Routing policy
    u_int32_t mesh_routing_policy;

    // For deadlock avoidance
    u_int32_t distance_class_length;

    // Performance measurements and counters
    ComputeCellStatistics statistics;

    // This is used to be fair in routing. When we start a communication cycle from the same
    // recv_channel what will happen is that that channel will get priority in sending its operons
    // at expense of thoer channels/neighbors. We want to start every cycle with a different
    // starting recv_channel and then alternate between them. This will provide fairness and not
    // cause congestion at any one link.
    u_int32_t current_recv_channel_to_start_a_cycle{};

    static auto get_cell_type_name(CellType type) -> std::string;

    static auto get_compute_cell_shape_name(computeCellShape shape) -> std::string;

    static auto get_compute_cell_shape_enum(const std::string& shape) -> computeCellShape;

    static auto get_number_of_neighbors(computeCellShape) -> u_int32_t;

    static auto cc_id_to_cooridinate(u_int32_t cc_id, computeCellShape shape, u_int32_t dim_y)
        -> Coordinates;

    static auto cc_cooridinate_to_id(Coordinates cc_cooridinate,
                                     computeCellShape shape_,
                                     u_int32_t dim_y) -> u_int32_t;

    inline auto cc_exists(SignedCoordinates cc_coordinate) -> bool;

    auto should_I_use_mesh(Coordinates src_cc_cooridinate, Coordinates dst_cc_cooridinate) -> bool;

    auto check_cut_off_distance(Coordinates dst_cc_cooridinate) -> bool;

    void add_neighbor(std::optional<std::pair<u_int32_t, Coordinates>> neighbor_compute_cell);

    // Configure this Cell and assign neighbors in the CCA grid
    void add_neighbor_compute_cells();

    // Execute a single cycle for this cell
    virtual void run_a_computation_cycle(std::vector<std::shared_ptr<Cell>>& CCA_chip,
                                         void* function_events) = 0;

    // TODO: write comments
    virtual void prepare_a_communication_cycle(std::vector<std::shared_ptr<Cell>>& CCA_chip) = 0;

    // TODO: write comments
    virtual void run_a_communication_cycle(std::vector<std::shared_ptr<Cell>>& CCA_chip) = 0;

    void essential_house_keeping_cycle(std::vector<std::shared_ptr<Cell>>& CCA_chip);

    // Checks if the cell is active or not
    virtual auto is_compute_cell_active() -> u_int32_t = 0;

    auto is_congested() -> std::pair<bool, u_int32_t>;

    u_int32_t current_cycle;

    // Used to keep record of the last cycle this Cell was congested. It will be used for throttle
    // and other purposes.
    std::optional<u_int32_t> last_congested_cycle;

    // Routing
    // Based on the routing algorithm and the shape of CCs it will return which neighbor to pass
    // this operon to. The returned value is the index [0...number of neighbors) coresponding
    // clockwise the channel id of the physical shape.
    auto get_route_towards_cc_id(u_int32_t src_cc_id, u_int32_t dst_cc_id)
        -> std::vector<u_int32_t>;
    auto get_west_first_route_towards_cc_id(u_int32_t dst_cc_id) -> std::vector<u_int32_t>;
    auto get_vertical_first_route_towards_cc_id(u_int32_t dst_cc_id) -> std::vector<u_int32_t>;
    auto get_horizontal_first_route_towards_cc_id(u_int32_t dst_cc_id) -> std::vector<u_int32_t>;

    // Experimental

    auto get_mixed_first_route_towards_cc_id(u_int32_t src_cc_id, u_int32_t dst_cc_id)
        -> std::vector<u_int32_t>;
    auto get_adaptive_positive_only_routes_towards_cc_id(u_int32_t src_cc_id, u_int32_t dst_cc_id)
        -> std::vector<u_int32_t>;

    auto get_adaptive_west_first_route_towards_cc_id(u_int32_t src_cc_id, u_int32_t dst_cc_id)
        -> std::vector<u_int32_t>;

    inline auto horizontal_first_routing(Coordinates dst_cc_coordinates) -> std::vector<u_int32_t>;
    inline auto vertical_first_routing(Coordinates dst_cc_coordinates) -> std::vector<u_int32_t>;

    // Experimental Ends

    // Depreciated
    auto get_dimensional_route_towards_cc_id(u_int32_t dst_cc_id) -> u_int32_t;

    // Receive an operon from a neighbor
    auto recv_operon(const Operon& operon, u_int32_t direction, u_int32_t distance_class) -> bool;

    void copy_cell_simulation_records_to_statistics();

    friend auto operator<<(std::ostream& os, const Cell& cc) -> std::ostream&
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

    virtual ~Cell() = default;
};

#endif // CELL_HPP

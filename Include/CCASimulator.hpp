/*
BSD 3-Clause License

Copyright (c) 2023-2024, Bibrak Qamar

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

#include "CCAFunctionEvents.hpp"
#include "Cell.hpp"
#include "HtreeNetwork.hpp"
#include "MemoryAllocator.hpp"
#include "Routing.hpp"

using u_long = unsigned long;

// The user uses this alias for `Object` to create `Terminator` since the object contains a
// `Terminator`.
using CCATerminator = Object;

// Utility function to create action arguments.
// TODO: Later maybe move these to some CCAUtils header class?
template<typename ApplicationArgumentType>
inline auto
cca_create_action_argument(const ApplicationArgumentType& src) -> ActionArgumentType
{
    ActionArgumentType const args_x(new char[sizeof(ApplicationArgumentType)],
                                    std::default_delete<char[]>());
    memcpy(args_x.get(), &src, sizeof(ApplicationArgumentType));

    return args_x;
}

// Utility function to get action arguments.
// TODO: Later maybe move these to some CCAUtils header class?
template<typename ApplicationArgumentType>
inline auto
cca_get_action_argument(const ActionArgumentType& src) -> ApplicationArgumentType
{
    ApplicationArgumentType app_args{};
    memcpy(&app_args, src.get(), sizeof(ApplicationArgumentType));

    return app_args;
}

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
    double avg_cells_active_percent{};

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

    // Number of SinkCells per CCA chip.
    u_int64_t total_sink_cells;

    // Total CCA Cells (including Sink Cells).
    u_int64_t total_compute_cells;

    // Memory per compute cell and the total combined memory of this CCA chip.
    u_int64_t memory_per_cc;
    u_int64_t total_chip_memory;

    // Declare the CCA Chip that is composed of Compute Cell(s) and any SinkCell(s).
    std::vector<std::shared_ptr<Cell>> CCA_chip;

    // Declare the I/O Channels (north and south) that are composed of Compute Cell(s) connecting to
    // their correcponsing CCs in the CCA Chip. Index: 0 is north and 1 is south.
    std::vector<std::shared_ptr<Cell>> IO_Channel[2];

    // High bandwidth network type. This is the primary network. By default the mesh. But can be
    // Torus and more.
    // TODO: Instead of `u_int32_t` put some enum type.
    // 0: Mesh
    // 1: Torus
    u_int32_t primary_network_type{};

    // The routing policy and algorithms id for the mesh network.
    u_int32_t mesh_routing_policy_id;

    // ID of the host. Just like Compute Cells have Id the host also has. This is useful for
    // invoking functions on objects in the host memory. For example: The root terminator form the
    // host.
    u_int32_t host_id;

    // The host memory
    std::shared_ptr<char[]> host_memory;

    // Memory of host in bytes. Used to store objects at host such as the root terminator provided
    // by the user.
    static inline constexpr u_int32_t host_memory_size_in_bytes = 12048; // 2 KB
    char* host_memory_raw_ptr;
    char* host_memory_curr_ptr;

    // For statistics
    bool global_active_cc;
    u_long total_current_run_cycles{};
    u_long total_cycles{};

    // Seconds layer network.
    HtreeNetwork htree_network;

    // Registers and manages the function events.
    FunctionEventManager function_events;

    // To record various measurements of the system to be used for analysis.
    CCASimulatorStatistics cca_statistics;

    CCASimulator(computeCellShape shape_in,
                 u_int32_t hx_in,
                 u_int32_t hy_in,
                 u_int32_t hdepth_in,
                 u_int32_t hbandwidth_max_in,
                 u_int32_t memory_per_cc_in,
                 u_int32_t primary_network_type_in,
                 u_int32_t mesh_routing_policy_id_in)
        : shape_of_compute_cells(shape_in)
        , hx(hx_in)
        , hy(hy_in)
        , hdepth(hdepth_in)
        , hbandwidth_max(hbandwidth_max_in)
        , memory_per_cc(memory_per_cc_in)
        , primary_network_type(primary_network_type_in)
        , mesh_routing_policy_id(mesh_routing_policy_id_in)
        , htree_network(hx_in, hy_in, hdepth_in, hbandwidth_max_in)
    {

        // Seed the random generator. Right now rand is being used in the VicinityMemoryAllocator.
        std::srand(1989);

        // Using Low-Latency Network as Htree therefore just by default using it to
        // prepare the CCA chip. When hdepth == 0, then thre is no Htree and dims are hx and hy.
        this->dim_x = HtreeNetwork::get_htree_dims(this->hx, this->hdepth);
        this->dim_y = HtreeNetwork::get_htree_dims(this->hy, this->hdepth);
        this->total_compute_cells = this->dim_x * this->dim_y;

        // Find the total number of SinkCells
        double total_sink_cells_calculator = 0;
        if (hdepth != 0) {
            total_sink_cells_calculator = std::pow(2, hdepth);
        }
        total_sink_cells_calculator *= total_sink_cells_calculator;
        this->total_sink_cells = static_cast<u_int32_t>(total_sink_cells_calculator);

        this->global_active_cc = false;
        this->total_cycles = 0;

        this->total_chip_memory =
            (this->total_compute_cells - this->total_sink_cells) * this->memory_per_cc;

        this->host_id = this->dim_x * this->dim_y;

        // This doesn't work with older compilers that don't have newer C++ features. Therefore
        // using the old way of explicitly providing new and deleters. this->host_memory =
        // std::make_shared<char[]>(this->host_memory_size_in_bytes);
        std::shared_ptr<char[]> const host_memory_ptr(
            new char[CCASimulator::host_memory_size_in_bytes], std::default_delete<char[]>());
        this->host_memory = host_memory_ptr;

        this->host_memory_raw_ptr = this->host_memory.get();
        this->host_memory_curr_ptr = this->host_memory_raw_ptr;

        this->create_the_chip();

        assert(this->host_id == this->CCA_chip.size());
    }

    inline void print_discription(std::ostream& os)
    {
        double const ratio_sink_compute =
            (100.0 * this->total_sink_cells) / static_cast<double>(this->total_compute_cells);

        os << "\nCCA Chip Details:\n\tShape: "
           << ComputeCell::get_compute_cell_shape_name(this->shape_of_compute_cells)
           << "\n\tDim: " << this->dim_x << " x " << this->dim_y;

        if (this->hdepth == 0) {
            os << "\n\tTotal Compute Cells: " << this->total_compute_cells;
        } else {
            os << "\n\tTotal Cells: " << this->total_compute_cells;
        }

        os << "\n\tMemory Per Compute Cell: " << this->memory_per_cc / static_cast<double>(1024)
           << " KB" << "\n\tTotal Chip Memory: "
           << static_cast<double>(this->total_chip_memory / static_cast<double>(1024 * 1024))
           << " MB" << "\n\tMesh Type: " << this->primary_network_type
           << "\n\tRouting Policy: " << this->mesh_routing_policy_id << "\n";

        if (this->hdepth != 0) {

            os << "\n\tHtree End Node Coverage Block: " << this->hx << " x " << this->hy
               << "\n\tHtree Depth: " << this->hdepth
               << "\n\tHtree Possible Bandwidth Max: " << this->hbandwidth_max
               << "\n\tTotal Compute Cells: " << this->total_compute_cells - this->total_sink_cells
               << "\n\tTotal Sink Cells: " << this->total_sink_cells
               << "\n\tSink/Compute Ratio (%): " << ratio_sink_compute << "\n"
               << std::endl;
        }
    }
    inline void output_description_in_a_single_line(std::ostream& os)
    {
        os << ComputeCell::get_compute_cell_shape_name(this->shape_of_compute_cells) << "\t"
           << this->dim_x << "\t" << this->dim_y << "\t" << this->hx << "\t" << this->hy << "\t"
           << this->hdepth << "\t" << this->hbandwidth_max << "\t" << this->total_compute_cells
           << "\t" << this->total_compute_cells - this->total_sink_cells << "\t"
           << this->total_sink_cells << "\t" << this->total_chip_memory << "\t" << THROTTLE << "\t"
           << RECVBUFFSIZE << "\n";
    }
    inline void write_cca_info(std::ostream& os)
    {
        os << "shape\tdim_x\tdim_y\thx\thy\thdepth\thbandwidth_max\ttotal_cells\ttotal_compute_"
              "cells\ttotal_sink_cells\ttotal_chip_memory(bytes)\tthrottle\trecv_buff_size\n";

        this->output_description_in_a_single_line(os);

        os << "congestion_policy\n"
           << "constant_threshold" << "\t" << curently_congested_threshold << "\n";

        std::string queues_text = "single";
        u_int32_t size_of_diffuse_queue = 0;
        if (split_queues) {
            queues_text = "split";
            size_of_diffuse_queue = diffuse_queue_size;
        }

        os << "queues_configuration\n"
           << queues_text << "\t" << action_queue_size << "\t" << size_of_diffuse_queue << "\n";

        os << "ghost_children_max\n" << ghost_children_max << "\n";
    }

    inline void output_CCA_active_status_per_cycle(std::ostream& os)
    {

        for (size_t i = 0; i < this->cca_statistics.active_status.size(); i++) {
            this->cca_statistics.avg_cells_active_percent +=
                this->cca_statistics.active_status[i].cells_active_percent;
        }

        this->cca_statistics.avg_cells_active_percent /= this->cca_statistics.active_status.size();

        std::cout << "\nAvg active cells percent = "
                  << this->cca_statistics.avg_cells_active_percent << "\n";
        os << "avg_cells_active_percent\n" << this->cca_statistics.avg_cells_active_percent << "\n";

        os << "Cycle#\tCells_Active_Percent\tHtree_Active_Percent\n";
        if constexpr (active_percent_switch) {
            for (size_t i = 0; i < this->cca_statistics.active_status.size(); i++) {
                // Print to output file for every output_skip_cycles-th cycle.
                // if (i % output_skip_cycles == 0) {
                os << i << "\t" << this->cca_statistics.active_status[i].cells_active_percent
                   << "\t" << this->cca_statistics.active_status[i].htree_active_percent << "\n";
                //}
            }
        } else {
            // Invalid values just to preserve format.
            os << "0\t0\t0\n";
            os << "0\t0\t0\n";
            os << "0\t0\t0\n";
        }
    }

    inline void output_CCA_active_status_per_cell_cycle(std::ostream& os)
    {
        os << "Cycles\tDim_x\tDim_y\n";
        os << this->total_cycles << "\t" << this->dim_x << "\t" << this->dim_y << "\n";
        os << "Active_Status_Per_Cell_Per_Cycle\n";
        for (size_t i = 0; i < this->cca_statistics.individual_cells_active_status_per_cycle.size();
             i++) {
            std::shared_ptr<u_int32_t[]> const frame =
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

    static inline auto get_compute_cell_coordinates(u_int32_t cc_id,
                                                    u_int32_t dim_y) -> Coordinates;

    auto cc_id_to_cooridinate(u_int32_t cc_id) -> Coordinates;

    auto cc_cooridinate_to_id(Coordinates cc_cooridinate) -> u_int32_t;

    // The main chip creation function.
    void create_the_chip();

    // The IO channels creation function.
    void create_IO_channels();

    // Register a function event
    auto register_function_event(handler_func function_event_handler) -> CCAFunctionEvent;

    // Return the memory used in bytes.
    auto get_host_memory_used() -> u_int32_t;
    // In bytes.
    auto get_host_memory_curr_ptr_offset() -> u_int32_t;
    // Get memory left in bytes.
    auto host_memory_available_in_bytes() -> u_int32_t;

    // Create a CCATerminator object on host and return the address.
    auto create_terminator() -> std::optional<Address>;

    // Check for termination of the diffusion.
    auto is_diffusion_active(Address terminator_in) -> bool;

    // Utility to set a terminator.
    void reset_terminator(Address terminator_in);

    auto allocate_and_insert_object_on_cc(MemoryAllocator& allocator,
                                          void* obj,
                                          size_t size_of_obj) -> std::optional<Address>;

    void germinate_action(const Action& action_to_germinate);

    void run_simulation(Address app_terminator);

    // Output simulation statistics and details.
    void print_statistics(std::ofstream& output_file);

    // Output simulation active animation per CC per cycle.
    void print_animation(std::string output_file_path);

    // Returns key configurations as a string to be appended to output file name.
    auto key_configurations_string() -> std::string;

    // Get the pointer to the object at `Address addr_in`.
    [[nodiscard]] auto get_object(Address addr_in) const -> void*;

  private:
    // Create the chip. It includes creating the cells and initializing them with IDs and their
    // types and more.
    void create_square_cell_htree_chip();

    // Create the chip of type square cells with only mesh connetion. There is not htree or any
    // second layer network involved. It includes creating the cells and initializing them with IDs
    // and their types and more.
    void create_square_cell_mesh_only_chip();
};

#endif // CCASimulator_HPP

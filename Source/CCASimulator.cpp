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

#include <cstring>

// for std::ofstream
#include <fstream>

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
inline auto
CCASimulator::get_compute_cell_coordinates(u_int32_t cc_id, u_int32_t dim_y) -> Coordinates
{
    // Note: Later when new shapes are added this function night need to be changed to decide based
    // on the cell shape and chip dimensions
    return Coordinates(cc_id % dim_y, cc_id / dim_y);
}

auto
CCASimulator::cc_id_to_cooridinate(u_int32_t cc_id) -> Coordinates
{
    return ComputeCell::cc_id_to_cooridinate(cc_id, this->shape_of_compute_cells, this->dim_y);
}

auto
CCASimulator::cc_cooridinate_to_id(Coordinates cc_cooridinate) -> u_int32_t
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

            u_int32_t const cc_id = i * this->dim_y + j;

            // Insert the sink cell
            if ((next_row_sink_cells == i) && (next_col_sink_cells == j)) {

                Coordinates const sink_cell_cooridnates =
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
                std::shared_ptr<SinkCell> const sink_cell =
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

            u_int32_t const cc_id = i * this->dim_y + j;

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
auto
CCASimulator::register_function_event(handler_func function_event_handler) -> CCAFunctionEvent
{
    return this->function_events.register_function_event(function_event_handler);
}

// Return the memory used in bytes
auto
CCASimulator::get_host_memory_used() -> u_int32_t
{
    return this->host_memory_curr_ptr - this->host_memory_raw_ptr;
}

// In bytes
auto
CCASimulator::get_host_memory_curr_ptr_offset() -> u_int32_t
{
    return get_host_memory_used();
}

// Get memory left in bytes
auto
CCASimulator::host_memory_available_in_bytes() -> u_int32_t
{
    return CCASimulator::host_memory_size_in_bytes - this->get_host_memory_used();
}

// Get the pointer to the object at `Address addr_in`
auto
CCASimulator::get_object(Address addr_in) const -> void*
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
auto
CCASimulator::create_terminator() -> std::optional<Address>
{
    if (this->host_memory_available_in_bytes() < sizeof(CCATerminator)) {
        return std::nullopt;
    }

    CCATerminator host_terminator;
    u_int32_t const obj_memory_addr_offset = get_host_memory_curr_ptr_offset();
    Address host_terminator_addr(this->host_id, obj_memory_addr_offset, adressType::host_address);

    host_terminator.terminator.my_object = host_terminator_addr;

    memcpy(this->host_memory_curr_ptr, &host_terminator, sizeof(CCATerminator));
    this->host_memory_curr_ptr += sizeof(CCATerminator);

    return host_terminator_addr;
}
auto
CCASimulator::is_diffusion_active(Address terminator_in) -> bool
{

    auto* terminator_obj = static_cast<CCATerminator*>(this->get_object(terminator_in));
    return terminator_obj->terminator.is_active();
}

void
CCASimulator::reset_terminator(Address terminator_in)
{
    // For now this is only implemented for host type terminators.
    assert(terminator_in.type == adressType::host_address);

    auto* terminator_obj = static_cast<CCATerminator*>(this->get_object(terminator_in));
    terminator_obj->terminator.reset();
}

auto
CCASimulator::allocate_and_insert_object_on_cc(MemoryAllocator& allocator,
                                               void* obj,
                                               size_t size_of_obj) -> std::optional<Address>
{
    // Get the ID of the compute cell where this vertex is to be allocated.
    u_int32_t const cc_id = allocator.get_next_available_cc(*this);

    auto cc_ptr = std::dynamic_pointer_cast<ComputeCell>(this->CCA_chip[cc_id]);
    return cc_ptr->create_object_in_memory(obj, size_of_obj);
}

void
CCASimulator::germinate_action(const Action& action_to_germinate)
{
    // dynamic_pointer_cast to go down/across class hierarchy
    auto compute_cell =
        std::dynamic_pointer_cast<ComputeCell>(this->CCA_chip[action_to_germinate.obj_addr.cc_id]);

    if (!compute_cell) {
        std::cerr << "Bug! Compute Cell not found: " << action_to_germinate.obj_addr.cc_id << "\n";
        exit(0);
    }

    compute_cell->insert_action(action_to_germinate);

    // Get the host terminator object for signal.
    auto* obj = static_cast<Object*>(this->get_object(action_to_germinate.origin_addr));
    obj->terminator.host_signal();

    // TODO: make these separate counter for different kind of actions. FIXME
    compute_cell->statistics.actions_created++;
}

// Output simulation statistics and details.
void
CCASimulator::print_statistics(std::ofstream& output_file)
{

    ComputeCellStatistics simulation_statistics;
    for (auto& cc : this->CCA_chip) {
        simulation_statistics += cc->statistics;
    }

    std::cout << simulation_statistics;
    float avg_objects_per_cc =
        static_cast<double>(simulation_statistics.objects_allocated) /
        static_cast<double>(this->total_compute_cells - this->total_sink_cells);
    std::cout << "Avg Objects per Compute Cell: " << avg_objects_per_cc;

    // Output CCA Chip details
    this->write_cca_info(output_file);

    // Output total cycles, total actions, total actions performed work, total actions false on
    // predicate. TODO: Somehow put the resource usage as a percentage...?
    output_file << "total_cycles\ttotal_objects_created\ttotal_actions_created\ttotal_actions_"
                   "performed_work\ttotal_actions_false_on_predicate\toperons_moved\n"
                << this->total_cycles << "\t" << simulation_statistics.objects_allocated << "\t"
                << simulation_statistics.actions_created << "\t"
                << simulation_statistics.actions_performed_work << "\t"
                << simulation_statistics.actions_false_on_predicate << "\t"
                << simulation_statistics.operons_moved <<"\n";

    output_file << "avg_objects_per_cc\n" << avg_objects_per_cc << "\n";

    // Output the active status of the individual cells and htree per cycle
    this->output_CCA_active_status_per_cycle(output_file);

    // Output statistics for each compute cell
    ComputeCellStatistics::generate_label(output_file);
    for (auto& cc : this->CCA_chip) {
        cc->statistics.output_results_in_a_single_line(output_file, cc->id, cc->cooridates);
        if (&cc != &this->CCA_chip.back()) {
            output_file << "\n";
        }
    }
}

void
CCASimulator::run_simulation(Address app_terminator)
{
    this->total_current_run_cycles = 0;

    bool is_system_active = true;
    bool run_next_cycle = true;
    // while (is_system_active) {

    // while (this->is_diffusion_active(app_terminator)) {
    while (run_next_cycle) {
        /* u_int32_t count_temp = 0;
        while (count_temp < 100) {
            count_temp++; */

        is_system_active = false;
        run_next_cycle = false;

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
#pragma omp parallel for
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

// Run communication cycle
#pragma omp parallel for
        for (u_int32_t i = 0; i < this->CCA_chip.size(); i++) {
            this->CCA_chip[i]->essential_house_keeping_cycle(this->CCA_chip);
        }

        // Check for termination. Not needed now since the terminator is implemented but keeping it
        // here for ploting activation charts.
        u_int32_t sum_global_active_cc_local = 0;
        // Also put the active status in the statistics to create the animation using the python
        // script.
        std::shared_ptr<u_int32_t[]> const active_status_frame_per_cells(
            new u_int32_t[this->CCA_chip.size()](), std::default_delete<u_int32_t[]>());

#pragma omp parallel for reduction(+ : sum_global_active_cc_local)
        for (u_int32_t i = 0; i < this->CCA_chip.size(); i++) {
            active_status_frame_per_cells[i] = this->CCA_chip[i]->is_compute_cell_active();
            if (active_status_frame_per_cells[i]) {
                sum_global_active_cc_local++;
            }
        }

        if constexpr (animation_switch) {
            this->cca_statistics.individual_cells_active_status_per_cycle.push_back(
                active_status_frame_per_cells);
        }

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
        double const percent_CCs_active = 100.0 * static_cast<double>(sum_global_active_cc_local) /
                                          static_cast<double>(this->CCA_chip.size());
        double const percent_htree_active =
            100.0 * static_cast<double>(sum_global_active_htree) /
            static_cast<double>(htree_network.htree_all_nodes.size());

        // Only print on screen every 500th cycle status update.
        if (this->total_current_run_cycles % 500 == 0) {
            std::cout << "End of current run cycle # " << this->total_current_run_cycles
                      << ", Total cycles: " << this->total_cycles
                      << ", CCs Active: " << percent_CCs_active
                      << "%, htree Active: " << percent_htree_active << "%\n";
        }
        this->cca_statistics.active_status.emplace_back(percent_CCs_active, percent_htree_active);
        this->total_cycles++;
        this->total_current_run_cycles++;

// Set new cycle # for every Cell: Experimental
#pragma omp parallel for
        for (u_int32_t i = 0; i < this->CCA_chip.size(); i++) {
            this->CCA_chip[i]->current_cycle++;
        }

        // Find whether to run the next cycle or not? This is based on the termination method being
        // used. When termination_switch == true, it uses the ack based Dijkstra–Scholten algorithm
        // that incurs the overhead of an ack back for every action recieved. When
        // termination_switch == false, it just peeks at the qeues of cells and network queue to see
        // if they are empty or not. This is needed to benchmark termination detection overhead. The
        // termination_switch is set at compile time using -D TERMINATION=true/false.
        if constexpr (termination_switch) {
            run_next_cycle = this->is_diffusion_active(app_terminator);
        } else {
            run_next_cycle = is_system_active;
        }
    }
    this->global_active_cc = is_system_active;

    // Copy any internal Cell records, counters etc to its statistics
#pragma omp parallel for
    for (u_int32_t i = 0; i < this->CCA_chip.size(); i++) {
        this->CCA_chip[i]->copy_cell_simulation_records_to_statistics();
    }
}

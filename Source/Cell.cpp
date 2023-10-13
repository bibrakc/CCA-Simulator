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

#include "Cell.hpp"

#include <cmath>
// #include <random>

// Utility function to convert type of pair
template<typename To, typename From>
inline auto
convert_internal_type_of_pair(const std::pair<From, From>& p) -> std::pair<To, To>
{
    return std::make_pair(static_cast<To>(p.first), static_cast<To>(p.second));
}

void
Cell::add_neighbor(std::optional<std::pair<u_int32_t, Coordinates>> neighbor_compute_cell)
{
    this->neighbor_compute_cells.push_back(neighbor_compute_cell);
}

void
Cell::add_neighbor_compute_cells()
{
    /*
        Left = 0 index
        Up = 1 index
        Right = 2 index
        Down = 3 index
    */

    if (this->primary_network_type != 0 && this->primary_network_type != 1) {
        std::cerr << "primary_network_type: " << primary_network_type << " not supported!\n";
        exit(0);
    }

    if (this->shape == computeCellShape::square) {

        // Note: The coordinates are of type unsigned int and we need to do arithematics that
        // may give negative int values. Therefore, we cast them to signed int
        auto coordinate_signed = convert_internal_type_of_pair<int32_t>(this->cooridates);
        int32_t const cc_coordinate_x = coordinate_signed.first;
        int32_t const cc_coordinate_y = coordinate_signed.second;

        // Left neighbor
        SignedCoordinates left_neighbor =
            std::pair<int32_t, int32_t>(cc_coordinate_x - 1, cc_coordinate_y);
        if (this->primary_network_type == 1) { // Torus
            auto [x_id, y_id] = left_neighbor;
            if (x_id < 0) {
                x_id = this->dim_y - 1;
                left_neighbor.first = x_id;
            }
            auto left_neighbor_unsigned = convert_internal_type_of_pair<u_int32_t>(left_neighbor);

            auto left_neighbor_id =
                Cell::cc_cooridinate_to_id(left_neighbor_unsigned, this->shape, this->dim_y);

            this->add_neighbor(
                std::pair<u_int32_t, Coordinates>(left_neighbor_id, left_neighbor_unsigned));

        } else { // Mesh

            if (this->cc_exists(left_neighbor)) {
                auto left_neighbor_unsigned =
                    convert_internal_type_of_pair<u_int32_t>(left_neighbor);

                auto left_neighbor_id =
                    Cell::cc_cooridinate_to_id(left_neighbor_unsigned, this->shape, this->dim_y);

                this->add_neighbor(
                    std::pair<u_int32_t, Coordinates>(left_neighbor_id, left_neighbor_unsigned));
            } else {
                this->add_neighbor(std::nullopt);
            }
        }

        // Up neighbor
        SignedCoordinates up_neighbor =
            std::pair<int32_t, int32_t>(cc_coordinate_x, cc_coordinate_y - 1);
        if (this->primary_network_type == 1) { // Torus
            auto [x_id, y_id] = up_neighbor;
            if (y_id < 0) {
                y_id = this->dim_x - 1;
                up_neighbor.second = y_id;
            }
            auto up_neighbor_unsigned = convert_internal_type_of_pair<u_int32_t>(up_neighbor);

            auto up_neighbor_id =
                Cell::cc_cooridinate_to_id(up_neighbor_unsigned, this->shape, this->dim_y);

            this->add_neighbor(
                std::pair<u_int32_t, Coordinates>(up_neighbor_id, up_neighbor_unsigned));

        } else { // Mesh
            if (this->cc_exists(up_neighbor)) {
                auto up_neighbor_unsigned = convert_internal_type_of_pair<u_int32_t>(up_neighbor);

                auto up_neighbor_id =
                    Cell::cc_cooridinate_to_id(up_neighbor_unsigned, this->shape, this->dim_y);

                this->add_neighbor(
                    std::pair<u_int32_t, Coordinates>(up_neighbor_id, up_neighbor_unsigned));
            } else {
                this->add_neighbor(std::nullopt);
            }
        }
        // Right neighbor
        SignedCoordinates right_neighbor =
            std::pair<int32_t, int32_t>(cc_coordinate_x + 1, cc_coordinate_y);
        if (this->primary_network_type == 1) { // Torus
            auto [x_id, y_id] = right_neighbor;
            if (x_id == static_cast<int32_t>(this->dim_y)) {
                x_id = 0;
                right_neighbor.first = x_id;
            }
            auto right_neighbor_unsigned = convert_internal_type_of_pair<u_int32_t>(right_neighbor);

            auto right_neighbor_id =
                Cell::cc_cooridinate_to_id(right_neighbor_unsigned, this->shape, this->dim_y);

            this->add_neighbor(
                std::pair<u_int32_t, Coordinates>(right_neighbor_id, right_neighbor_unsigned));

        } else { // Mesh
            if (this->cc_exists(right_neighbor)) {
                auto right_neighbor_unsigned =
                    convert_internal_type_of_pair<u_int32_t>(right_neighbor);

                auto right_neighbor_id =
                    Cell::cc_cooridinate_to_id(right_neighbor_unsigned, this->shape, this->dim_y);

                this->add_neighbor(
                    std::pair<u_int32_t, Coordinates>(right_neighbor_id, right_neighbor_unsigned));
            } else {
                this->add_neighbor(std::nullopt);
            }
        }

        // Down neighbor
        SignedCoordinates down_neighbor = Coordinates(cc_coordinate_x, cc_coordinate_y + 1);
        if (this->primary_network_type == 1) { // Torus
            auto [x_id, y_id] = down_neighbor;
            if (y_id == static_cast<int32_t>(this->dim_x)) {
                y_id = 0;
                down_neighbor.second = y_id;
            }
            auto down_neighbor_unsigned = convert_internal_type_of_pair<u_int32_t>(down_neighbor);

            auto down_neighbor_id =
                Cell::cc_cooridinate_to_id(down_neighbor_unsigned, this->shape, this->dim_y);

            this->add_neighbor(
                std::pair<u_int32_t, Coordinates>(down_neighbor_id, down_neighbor_unsigned));

        } else { // Mesh
            if (this->cc_exists(down_neighbor)) {
                auto down_neighbor_unsigned =
                    convert_internal_type_of_pair<u_int32_t>(down_neighbor);

                auto down_neighbor_id =
                    Cell::cc_cooridinate_to_id(down_neighbor_unsigned, this->shape, this->dim_y);

                this->add_neighbor(
                    std::pair<u_int32_t, Coordinates>(down_neighbor_id, down_neighbor_unsigned));
            } else {
                this->add_neighbor(std::nullopt);
            }
        }

    } else if (this->shape == computeCellShape::block_1D) {

        std::cerr << Cell::get_compute_cell_shape_name(this->shape) << " not supported!\n";
        exit(0);

    } else {
        // Shape not supported
        std::cerr << Cell::get_compute_cell_shape_name(this->shape) << " not supported!\n";
        exit(0);
    }
}

auto
Cell::recv_operon(const Operon& operon, u_int32_t direction_in, u_int32_t virtual_channel) -> bool
{
    return this->recv_channel_per_neighbor[direction_in][virtual_channel].push(operon);
}

inline auto
Cell::cc_exists(const SignedCoordinates cc_coordinate) -> bool
{
    auto [cc_coordinate_x, cc_coordinate_y] = cc_coordinate;
    int const zero = 0;
    if (this->shape == computeCellShape::square) {

        // If invalid
        if ((cc_coordinate_x < zero) || (cc_coordinate_x >= static_cast<int>(this->dim_y)) ||
            (cc_coordinate_y < zero) || (cc_coordinate_y >= static_cast<int>(this->dim_x))) {
            return false;
        }
        return true;
    }
    // Shape not supported
    std::cerr << Cell::get_compute_cell_shape_name(this->shape) << " not supported!\n";
    exit(0);
}

void
ComputeCellStatistics::output_results_in_a_single_line(std::ostream& os,
                                                       u_int32_t cc_id,
                                                       Coordinates cc_cooridinates)
{
    os << cc_id << "\t" << Cell::get_cell_type_name(this->type) << "\t" << cc_cooridinates.first
       << "\t" << cc_cooridinates.second << "\t" << this->objects_allocated << "\t"
       << this->actions_created << "\t" << this->actions_acknowledgement_created << "\t"
       << this->actions_pushed << "\t" << this->actions_invoked << "\t"
       << this->actions_performed_work << "\t" << this->actions_acknowledgement_invoked << "\t"
       << this->actions_false_on_predicate

       << "\t" << this->operons_moved

       << "\t" << this->action_queue_count.get_max_count() << "\t"
       << this->task_queue_count.get_max_count() << "\t" << this->task_queue_count.get_total_count()

       << "\t" << this->send_channel_per_neighbor_contention_count_record[0].get_max_count() << "\t"
       << this->send_channel_per_neighbor_contention_count_record[0].get_total_count()

       << "\t" << this->send_channel_per_neighbor_contention_count_record[1].get_max_count() << "\t"
       << this->send_channel_per_neighbor_contention_count_record[1].get_total_count()

       << "\t" << this->send_channel_per_neighbor_contention_count_record[2].get_max_count() << "\t"
       << this->send_channel_per_neighbor_contention_count_record[2].get_total_count()

       << "\t" << this->send_channel_per_neighbor_contention_count_record[3].get_max_count() << "\t"
       << this->send_channel_per_neighbor_contention_count_record[3].get_total_count();
}

auto
Cell::get_cell_type_name(CellType type) -> std::string
{
    switch (type) {
        case (CellType::compute_cell):
            return { "ComputeCell" };
            break;
        case (CellType::sink_cell):
            return { "SinkCell" };
            break;
        default:
            return { "Invalid CellType" };
            break;
    }
}

auto
Cell::get_compute_cell_shape_name(computeCellShape shape) -> std::string
{
    switch (shape) {
        case (computeCellShape::block_1D):
            return { "block_1D" };
            break;
        case (computeCellShape::triangular):
            return { "triangular" };
            break;
        case (computeCellShape::square):
            return { "square" };
            break;
        case (computeCellShape::hexagon):
            return { "hexagon" };
            break;

        default:
            return { "Invalid Shape" };
            break;
    }
}

auto
Cell::get_compute_cell_shape_enum(const std::string& shape) -> computeCellShape
{
    if (shape == "block_1D") {
        return computeCellShape::block_1D;
    }
    if (shape == "triangular") {
        return computeCellShape::triangular;
    } else if (shape == "sqaure") {
        return computeCellShape::square;
    } else if (shape == "hexagon") {
        return computeCellShape::hexagon;
    } else {
        return computeCellShape::computeCellShape_invalid;
    }
}

auto
Cell::get_number_of_neighbors(computeCellShape shape_in) -> u_int32_t
{
    switch (shape_in) {
        case (computeCellShape::block_1D):
            return 2;
            break;
        case (computeCellShape::triangular):
            return 3;
            break;
        case (computeCellShape::square):
            return 4;
            break;
        case (computeCellShape::hexagon):
            return 6;
            break;

        default:
            std::cerr << "Shape: " << Cell::get_compute_cell_shape_name(shape_in)
                      << " not supported!\n";
            exit(0);
    }
}

auto
Cell::cc_id_to_cooridinate(u_int32_t cc_id, computeCellShape shape_, u_int32_t dim_y) -> Coordinates
{

    if (shape_ == computeCellShape::square) {
        return Coordinates(cc_id % dim_y, cc_id / dim_y);
    }
    // Shape not supported
    std::cerr << Cell::get_compute_cell_shape_name(shape_) << " not supported!\n";
    exit(0);
}

auto
Cell::cc_cooridinate_to_id(Coordinates cc_cooridinate, computeCellShape shape_, u_int32_t dim_y)
    -> u_int32_t
{

    if (shape_ == computeCellShape::square) {
        auto [x, y] = cc_cooridinate;
        // std::cout << "cc_cooridinate_to_id: (" << x << ", " << y << ") ----> " << (y * this->dim)
        // + x << "\n";
        return (y * dim_y) + x;
    }
    // Shape not supported
    std::cerr << Cell::get_compute_cell_shape_name(shape_) << " not supported!\n";
    exit(0);
}

auto
Cell::should_I_use_mesh(Coordinates src_cc_cooridinate, Coordinates dst_cc_cooridinate) -> bool
{
    if (this->shape == computeCellShape::square) {

        /*    bool isWithinBounds(int n_cols,
                               int n_rows,
                               int src_col,
                               int src_row,
                               int dst_col,
                               int dst_row,
                               int bound_cols,
                               int bound_rows) */

        auto [src_col, src_row] = src_cc_cooridinate;
        auto [dst_col, dst_row] = dst_cc_cooridinate;

        int const src_col_int = static_cast<int>(src_col);
        int const src_row_int = static_cast<int>(src_row);

        int const dst_col_int = static_cast<int>(dst_col);
        int const dst_row_int = static_cast<int>(dst_row);

        /*   const double percent = 0.80; */

        // Exponential decay function to calculate the percentage based on the distance
        // from the center
        double const center_col = (this->dim_y - 1) / 2.0;
        double const center_row = (this->dim_x - 1) / 2.0;

        // Calculate the distance from the center
        double const distance = std::sqrt((src_col_int - center_col) * (src_col_int - center_col) +
                                          (src_row_int - center_row) * (src_row_int - center_row));

        // Define the scaling factor (you can adjust this value to control the growth rate)
        double scaling_factor = 0.007;

        // Calculate the adjusted percentage based on the distance from the center
        double center_percent = 0.45; // Center percentage fixed at 40%
        double percentage = center_percent + scaling_factor * distance;

        // Apply clamping to ensure the percentage stays within the desired range (55%
        // to 75%)
        percentage = std::max(center_percent, percentage); // Minimum 45%
        percentage = std::min(0.95, percentage);           // Maximum 75%

        int const bound_cols = static_cast<int>(static_cast<double>(this->dim_y) * percentage);
        int const bound_rows = static_cast<int>(static_cast<double>(this->dim_x) * percentage);

        // Calculate the minimum and maximum allowed coordinates for the bounding region
        int min_col_bound = std::max(src_col_int - bound_cols, 0);
        int max_col_bound = std::min(src_col_int + bound_cols, static_cast<int>(this->dim_y) - 1);
        int min_row_bound = std::max(src_row_int - bound_rows, 0);
        int max_row_bound = std::min(src_row_int + bound_rows, static_cast<int>(this->dim_x) - 1);

        // Check if the destination coordinates are within the bounded region
        if (dst_col_int >= min_col_bound && dst_col_int <= max_col_bound &&
            dst_row_int >= min_row_bound && dst_row_int <= max_row_bound) {
            return true;
        }

        /*   int const max_hops_before_switching_to_low_latency_network = bound_cols + bound_rows;
          int const hops_to_destination = abs(src_col - dst_col) + abs(src_row - dst_row);

          if (hops_to_destination < max_hops_before_switching_to_low_latency_network) {
              return true;
          } */
        /* std::cout << "CC: " << this->id << ", src = " << src_cc_cooridinate
                  << ", dst = " << dst_cc_cooridinate
                  << ", max_hops_before_switching_to_low_latency_network: "
                  << max_hops_before_switching_to_low_latency_network
                  << ", hops_to_destination: " << hops_to_destination << ", predicate: "
                  << (hops_to_destination < max_hops_before_switching_to_low_latency_network)
                  << "\n"; */

        return false;

        /*
                double num_unit_h_in_row = std::pow(2, this->hdepth - 1);
                num_unit_h_in_row = num_unit_h_in_row + (num_unit_h_in_row / 2);
                u_int32_t const mesh_usage_region_length_cols = num_unit_h_in_row * this->hy;
                u_int32_t const mesh_usage_region_length_rows = num_unit_h_in_row * this->hx;

                // TODO: later make this distance customizable, either at compile time or runtime.
                // TODO: Look at narrow_cast and see if we should use that.
                if ((abs(static_cast<int>(src_col) - static_cast<int>(dst_col)) <=
                     static_cast<int>(mesh_usage_region_length_cols)) &&
                    (abs(static_cast<int>(src_row) - static_cast<int>(dst_row)) <=
                     static_cast<int>(mesh_usage_region_length_rows))) {
                    return true;
                }
         */
        /*  std::cout << "CC: " << this->id << ", num_unit_h_in_row = " << num_unit_h_in_row
           << ", mesh_usage_region_length_cols = " << mesh_usage_region_length_cols
           << ", mesh_usage_region_length_rows = " << mesh_usage_region_length_rows
           << "\n";
 std::cout << "CC: " << this->id << ", src: " << src_cc_cooridinate
           << ", dst: " << dst_cc_cooridinate << "\n"; */

        /*   return false; */
    }
    // Shape not supported
    std::cerr << Cell::get_compute_cell_shape_name(this->shape) << " not supported!\n";
    exit(0);
}

auto
Cell::is_congested() -> std::pair<bool, u_int32_t>
{

    bool is_congested = false;
    u_int32_t congestion_level_addition = 0;
    for (auto& congestion_count : this->send_channel_per_neighbor_contention_count) {
        if (congestion_count.get_count() > congestion_threshold_1) {
            is_congested = true;

            if (congestion_count.get_count() >= congestion_threshold_4) {
                congestion_level_addition = 3;
            } else if (congestion_count.get_count() >= congestion_threshold_3) {
                congestion_level_addition = 2;
            } else if (congestion_count.get_count() >= congestion_threshold_2) {
                congestion_level_addition = 1;
            }
            break;
        }
    }
    return std::pair<bool, u_int32_t>(is_congested, congestion_level_addition);
}

void
Cell::essential_house_keeping_cycle(std::vector<std::shared_ptr<Cell>>& /*CCA_chip*/)
{
    // Update the last_congested_cycle if needed;
    auto [is_congested, congestion_level_addition] = this->is_congested();
    if (is_congested) {
        this->last_congested_cycle = this->current_cycle;
    }

    // Update the cycle #
    this->current_cycle++;
}

auto
Cell::check_cut_off_distance(Coordinates dst_cc_cooridinate) -> bool
{
    if (this->shape == computeCellShape::square) {
        auto [src_col, src_row] = this->cooridates;
        auto [dst_col, dst_row] = dst_cc_cooridinate;

        // TODO: later make this distance customizable, either at compile time or runtime
        // TODO: Look at narrow_cast and see if we should use that.
        if ((abs(static_cast<int>(src_col) - static_cast<int>(dst_col)) <=
             static_cast<int>(this->hy / 2)) &&
            (abs(static_cast<int>(src_row) - static_cast<int>(dst_row)) <=
             static_cast<int>(this->hx / 2))) {
            return true;
        }
        return false;
    }
    // Shape not supported
    std::cerr << Cell::get_compute_cell_shape_name(this->shape) << " not supported!\n";
    exit(0);
}

auto
Cell::get_route_towards_cc_id(u_int32_t /*src_cc_id*/, u_int32_t dst_cc_id)
    -> std::vector<u_int32_t>
{
    // return get_west_first_route_towards_cc_id(dst_cc_id);

    // Note: Not a good performance with throtlling since the trottling is currently coarse-grain
    // such that if there is congestion at any channel the CC halts making new operons. Now adaptive
    // west-first could just easily have sent to up or right channels where there wouldn't have been
    // much congestion but it couldnt. TODO: Implement sophisticated throttling that takes into
    // account different channels.
    // return get_adaptive_west_first_route_towards_cc_id(src_cc_id, dst_cc_id);

    // Note: These are good with throttling.
    // return get_vertical_first_route_towards_cc_id(dst_cc_id);
    return get_horizontal_first_route_towards_cc_id(dst_cc_id);

    // This has deadlocks or dont work.
    // return get_adaptive_positive_only_routes_towards_cc_id(src_cc_id, dst_cc_id);
    // return get_mixed_first_route_towards_cc_id(src_cc_id, dst_cc_id);
}

auto
Cell::get_dimensional_route_towards_cc_id(u_int32_t dst_cc_id) -> u_int32_t
{

    // Algorithm == dimensional routing
    if (this->shape == computeCellShape::square) {
        // Remember for a square shaped CC there are four links to neighbors enumerated in
        // clockwise 0 = left, 1 = up, 2 = right, and 3 = down

        Coordinates const dst_cc_coordinates =
            Cell::cc_id_to_cooridinate(dst_cc_id, this->shape, this->dim_y);

        if constexpr (debug_code) {
            std::cout << "cc id : " << this->id << " dst_cc_coordinates = ("
                      << dst_cc_coordinates.first << ", " << dst_cc_coordinates.second
                      << ") -- origin = ( " << this->cooridates.first << ", "
                      << this->cooridates.second << ")\n";
        }
        // First check vertically in y axis then horizontally in x axis
        if (this->cooridates.second > dst_cc_coordinates.second) {
            return 1; // Clockwise 1 = up
        }
        if (this->cooridates.second < dst_cc_coordinates.second) {
            return 3; // Clockwise 3 = down
        } else if (this->cooridates.first > dst_cc_coordinates.first) {
            return 0; // Clockwise 0 = left
        } else if (this->cooridates.first < dst_cc_coordinates.first) {
            return 2; // Clockwise 2 = right
        }

        std::cerr << Cell::get_compute_cell_shape_name(this->shape)
                  << "Bug: routing not sucessful!\n";
    }
    // Shape or routing not supported
    std::cerr << Cell::get_compute_cell_shape_name(this->shape) << " or routing not supported!\n";
    exit(0);
}

auto
Cell::get_adaptive_positive_only_routes_towards_cc_id(u_int32_t /*src_cc_id*/, u_int32_t dst_cc_id)
    -> std::vector<u_int32_t>
{
    // This has deadlock :(

    // Algorithm == abaptive positive only routes
    if (this->shape == computeCellShape::square) {
        // Remember for a square shaped CC there are four links to neighbors enumerated in
        // clockwise 0 = left, 1 = up, 2 = right, and 3 = down

        Coordinates const dst_cc_coordinates =
            Cell::cc_id_to_cooridinate(dst_cc_id, this->shape, this->dim_y);

        std::vector<u_int32_t> paths;

        if ((this->cooridates.first > dst_cc_coordinates.first) &&
            (this->cooridates.second < dst_cc_coordinates.second)) {
            // 3rd Coordinate
            paths.push_back(0); // Clockwise 0 = left
            paths.push_back(3); // Clockwise 3 = down
        } else if ((this->cooridates.first > dst_cc_coordinates.first) &&
                   (this->cooridates.second > dst_cc_coordinates.second)) {
            // 1st Coordinate
            paths.push_back(0); // Clockwise 0 = left
            paths.push_back(1); // Clockwise 1 = up
        } else if ((this->cooridates.first < dst_cc_coordinates.first) &&
                   (this->cooridates.second > dst_cc_coordinates.second)) {
            // 2nd Coordinate
            // send up or right
            paths.push_back(1);
            paths.push_back(2);

        } else if ((this->cooridates.first < dst_cc_coordinates.first) &&
                   (this->cooridates.second < dst_cc_coordinates.second)) {
            // 4th Coordinate
            // send down or right
            paths.push_back(3);
            paths.push_back(2);
        } else if (this->cooridates.first > dst_cc_coordinates.first) {
            paths.push_back(0); // Clockwise 0 = left
        } else if (this->cooridates.first < dst_cc_coordinates.first) {
            // send to right
            paths.push_back(2);
        } else if (this->cooridates.second < dst_cc_coordinates.second) {
            // send to down
            paths.push_back(3);
        } else if (this->cooridates.second > dst_cc_coordinates.second) {
            // send to up
            paths.push_back(1);
        }

        if (paths.empty()) {
            std::cerr << Cell::get_compute_cell_shape_name(this->shape)
                      << " Bug: routing not sucessful!\n";
        }

        return paths;
    }
    // Shape or routing not supported
    std::cerr << Cell::get_compute_cell_shape_name(this->shape) << " or routing not supported!\n";
    exit(0);
}

auto
Cell::get_adaptive_west_first_route_towards_cc_id(u_int32_t /*src_cc_id*/, u_int32_t dst_cc_id)
    -> std::vector<u_int32_t>
{

    // Algorithm == west first adaptive
    if (this->shape == computeCellShape::square) {
        // Remember for a square shaped CC there are four links to neighbors enumerated in
        // clockwise 0 = left, 1 = up, 2 = right, and 3 = down

        Coordinates const dst_cc_coordinates =
            Cell::cc_id_to_cooridinate(dst_cc_id, this->shape, this->dim_y);

        std::vector<u_int32_t> paths;

        // West first routing restricts turns to the west side. Take west/left first if needed
        if (this->cooridates.first > dst_cc_coordinates.first) {
            paths.push_back(0); // Clockwise 0 = left
        } else if ((this->cooridates.first < dst_cc_coordinates.first) &&
                   (this->cooridates.second > dst_cc_coordinates.second)) {
            // 2nd Coordinate
            // send up or right
            paths.push_back(1);
            paths.push_back(2);

        } else if ((this->cooridates.first < dst_cc_coordinates.first) &&
                   (this->cooridates.second < dst_cc_coordinates.second)) {
            // 4th Coordinate
            // send down or right
            paths.push_back(3);
            paths.push_back(2);
        } else if (this->cooridates.first < dst_cc_coordinates.first) {
            // send to right
            paths.push_back(2);
        } else if (this->cooridates.second < dst_cc_coordinates.second) {
            // send to down
            paths.push_back(3);
        } else if (this->cooridates.second > dst_cc_coordinates.second) {
            // send to up
            paths.push_back(1);
        }

        if (paths.empty()) {
            std::cerr << Cell::get_compute_cell_shape_name(this->shape)
                      << " Bug: routing not sucessful!\n";
        }

        /* // Random number generator
        std::random_device rd;
        std::mt19937 generator(rd());

        // Shuffle the paths
        std::shuffle(paths.begin(), paths.end(), generator);

        Note: Tried the randamization but it doesn't help with performance. Will come back to it
        later. */

        return paths;
    }
    // Shape or routing not supported
    std::cerr << Cell::get_compute_cell_shape_name(this->shape) << " or routing not supported!\n";
    exit(0);
}

auto
Cell::get_west_first_route_towards_cc_id(u_int32_t dst_cc_id) -> std::vector<u_int32_t>
{

    // Algorithm == west first
    if (this->shape == computeCellShape::square) {
        // Remember for a square shaped CC there are four links to neighbors enumerated in
        // clockwise 0 = left, 1 = up, 2 = right, and 3 = down

        Coordinates const dst_cc_coordinates =
            Cell::cc_id_to_cooridinate(dst_cc_id, this->shape, this->dim_y);

        // West first routing restricts turns to the west side. Take west/left first if needed
        std::vector<u_int32_t> paths;

        if (this->cooridates.first > dst_cc_coordinates.first) {
            paths.push_back(0); // Clockwise 0 = left
        } else if ((this->cooridates.first < dst_cc_coordinates.first) &&
                   (this->cooridates.second > dst_cc_coordinates.second)) {

            // send up or right
            // based on availablity send there. Right now just send to up
            paths.push_back(1);

        } else if ((this->cooridates.first < dst_cc_coordinates.first) &&
                   (this->cooridates.second < dst_cc_coordinates.second)) {

            // send down or right
            // based on availablity send there. Right now just send to down
            paths.push_back(3);
        } else if (this->cooridates.first < dst_cc_coordinates.first) {
            // send to right
            paths.push_back(2);
        } else if (this->cooridates.second < dst_cc_coordinates.second) {
            // send to down
            paths.push_back(3);
        } else if (this->cooridates.second > dst_cc_coordinates.second) {
            // send to up
            paths.push_back(1);
        }

        if (paths.empty()) {
            std::cerr << Cell::get_compute_cell_shape_name(this->shape)
                      << " Bug: routing not sucessful!\n";
        }

        return paths;
    }
    // Shape or routing not supported
    std::cerr << Cell::get_compute_cell_shape_name(this->shape) << " or routing not supported!\n";
    exit(0);
}

inline auto
Cell::vertical_first_routing(Coordinates dst_cc_coordinates) -> std::vector<u_int32_t>
{

    std::vector<u_int32_t> paths;

    if (this->cooridates.second > dst_cc_coordinates.second) {
        paths.push_back(1); // Clockwise 1 = up
    } else if (this->cooridates.second < dst_cc_coordinates.second) {
        paths.push_back(3); // Clockwise 3 = down
    } else if (this->cooridates.first < dst_cc_coordinates.first) {
        // send to right
        paths.push_back(2);
    } else if (this->cooridates.first > dst_cc_coordinates.first) {
        // send to left
        paths.push_back(0);
    }

    return paths;
}

inline auto
Cell::horizontal_first_routing(Coordinates dst_cc_coordinates) -> std::vector<u_int32_t>
{
    std::vector<u_int32_t> paths;
    if (this->primary_network_type == 0) { // Mesh

        if (this->cooridates.first > dst_cc_coordinates.first) {
            paths.push_back(0); // Clockwise 0 = left
        } else if (this->cooridates.first < dst_cc_coordinates.first) {
            // send to right
            paths.push_back(2);
        } else if (this->cooridates.second > dst_cc_coordinates.second) {
            paths.push_back(1); // Clockwise 1 = up
        } else if (this->cooridates.second < dst_cc_coordinates.second) {
            paths.push_back(3); // Clockwise 3 = down
        }
        return paths;
    } else if (this->primary_network_type == 1) { // Torus

        if (this->cooridates.first != dst_cc_coordinates.first) {

            if (this->cooridates.first < dst_cc_coordinates.first) {
                int dst_to_right_edge = this->dim_y - dst_cc_coordinates.first;
                int wraped_distance_x = dst_to_right_edge + this->cooridates.first;

                int abs_distance_x = std::abs(static_cast<int>(this->cooridates.first) -
                                              static_cast<int>(dst_cc_coordinates.first));

                /* std::cout << "dst_to_right_edge: " << dst_to_right_edge
                          << ", wraped_distance: " << wraped_distance_x
                          << ", abs_distance: " << abs_distance_x << "\n"; */

                if (abs_distance_x <= wraped_distance_x) {
                    // std::cout << "Move Right!\n";
                    // send to right
                    paths.push_back(2);
                } else {
                    // std::cout << "Move Left!\n";
                    paths.push_back(0); // Clockwise 0 = left
                }
                return paths;
            } else {
                int dst_to_left_edge = dst_cc_coordinates.first;
                int wraped_distance_x = dst_to_left_edge + this->dim_y - this->cooridates.first;

                int abs_distance_x = std::abs(static_cast<int>(this->cooridates.first) -
                                              static_cast<int>(dst_cc_coordinates.first));
                if (abs_distance_x <= wraped_distance_x) {
                    paths.push_back(0); // Clockwise 0 = left
                } else {
                    // send to right
                    paths.push_back(2);
                }
                return paths;
            }

        } else {
            if (this->cooridates.second < dst_cc_coordinates.second) {
                int dst_to_down_edge = this->dim_x - dst_cc_coordinates.second;
                int wraped_distance_y = dst_to_down_edge + this->cooridates.second;

                int abs_distance_y = std::abs(static_cast<int>(this->cooridates.second) -
                                              static_cast<int>(dst_cc_coordinates.second));

                /* td::cout << "dst_to_down_edge: " << dst_to_down_edge
                          << ", wraped_distance_y: " << wraped_distance_y
                          << ", abs_distance_y: " << abs_distance_y << "\n"; */

                if (abs_distance_y <= wraped_distance_y) {
                    paths.push_back(3); // Clockwise 3 = down
                } else {
                    paths.push_back(1); // Clockwise 1 = up
                }
                return paths;
            } else {

                int dst_to_up_edge = dst_cc_coordinates.second;
                int wraped_distance_y = dst_to_up_edge + this->dim_x - this->cooridates.second;

                int abs_distance_y = std::abs(static_cast<int>(this->cooridates.second) -
                                              static_cast<int>(dst_cc_coordinates.second));

                /* std::cout << "dst_to_up_edge: " << dst_to_up_edge
                          << ", wraped_distance_y: " << wraped_distance_y
                          << ", abs_distance_y: " << abs_distance_y << "\n"; */

                if (abs_distance_y <= wraped_distance_y) {
                    paths.push_back(1); // Clockwise 1 = up
                } else {
                    paths.push_back(3); // Clockwise 3 = down
                }
                return paths;
            }
        }
    }
    std::cerr << "horizontal_first_routing returning nothing. This is not OK!!! \n";
    exit(0);
}

inline auto
row_chunks(u_int32_t cc_id, u_int32_t row, u_int32_t chunk_size, u_int32_t dim_y) -> bool
{
    u_int32_t const center = dim_y / 2;
    u_int32_t const start_chunk = (dim_y * row) + center + 4;
    return ((cc_id > start_chunk) && (cc_id < start_chunk + chunk_size));
}

auto
Cell::get_mixed_first_route_towards_cc_id(u_int32_t src_cc_id, u_int32_t dst_cc_id)
    -> std::vector<u_int32_t>
{

    // Algorithm == mixed first.
    // For even columns of CCs use horizontal (west or east) first and for odd columns use vertizal
    // first (up or down).
    if (this->shape == computeCellShape::square) {
        // Remember for a square shaped CC there are four links to neighbors enumerated in
        // clockwise 0 = left, 1 = up, 2 = right, and 3 = down

        Coordinates const dst_cc_coordinates =
            Cell::cc_id_to_cooridinate(dst_cc_id, this->shape, this->dim_y);

        Coordinates const src_cc_coordinates =
            Cell::cc_id_to_cooridinate(src_cc_id, this->shape, this->dim_y);

        // .first = col, .second = row

        // bool is_vertical_epoch = (this->current_cycle / 100) % 2;
        bool is_vertical_routing_operon = true;
        if (src_cc_coordinates.second % 2 == 0) {
            if (src_cc_coordinates.first % 2 == 0) {
                is_vertical_routing_operon = false;
            }
        } else {
            if (src_cc_coordinates.first % 2 == 1) {
                is_vertical_routing_operon = true;
            } else {
                is_vertical_routing_operon = false;
            }
        }

        // Route vertically first
        if (is_vertical_routing_operon) {
            // std::cout << "vertical_first_routing\n";
            return this->vertical_first_routing(dst_cc_coordinates);
        } // Route horizontally first
        return this->horizontal_first_routing(dst_cc_coordinates);

        std::cerr << Cell::get_compute_cell_shape_name(this->shape)
                  << " Bug: routing not sucessful!\n";
    }
    // Shape or routing not supported
    std::cerr << Cell::get_compute_cell_shape_name(this->shape) << " or routing not supported!\n";
    exit(0);
}

// Dimension-ordered (Y-X) routing
auto
Cell::get_vertical_first_route_towards_cc_id(u_int32_t dst_cc_id) -> std::vector<u_int32_t>
{

    // Algorithm == vertical first
    // send up or down first
    if (this->shape == computeCellShape::square) {
        // Remember for a square shaped CC there are four links to neighbors enumerated in
        // clockwise 0 = left, 1 = up, 2 = right, and 3 = down

        Coordinates const dst_cc_coordinates =
            Cell::cc_id_to_cooridinate(dst_cc_id, this->shape, this->dim_y);

        // std::cout << "vertical_first_routing\n";
        return this->vertical_first_routing(dst_cc_coordinates);
    }
    // Shape or routing not supported
    std::cerr << Cell::get_compute_cell_shape_name(this->shape) << " or routing not supported!\n";
    exit(0);
}

// Dimension-ordered (X-Y) routing
auto
Cell::get_horizontal_first_route_towards_cc_id(u_int32_t dst_cc_id) -> std::vector<u_int32_t>
{

    // Algorithm == horizontal X-Y.
    if (this->shape == computeCellShape::square) {
        // Remember for a square shaped CC there are four links to neighbors enumerated in
        // clockwise 0 = left, 1 = up, 2 = right, and 3 = down

        Coordinates const dst_cc_coordinates =
            Cell::cc_id_to_cooridinate(dst_cc_id, this->shape, this->dim_y);

        // std::cout << "horizontal_first_routing\n";
        return this->horizontal_first_routing(dst_cc_coordinates);
    }
    // Shape or routing not supported
    std::cerr << Cell::get_compute_cell_shape_name(this->shape) << " or routing not supported!\n";
    exit(0);
}

void
Cell::copy_cell_simulation_records_to_statistics()
{
    this->statistics.send_channel_per_neighbor_contention_count_record =
        this->send_channel_per_neighbor_contention_count;
}

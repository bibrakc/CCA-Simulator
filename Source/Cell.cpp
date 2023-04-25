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

// Utility function to convert type of pair
template<typename To, typename From>
inline std::pair<To, To>
convert_internal_type_of_pair(const std::pair<From, From>& p)
{
    return std::make_pair(static_cast<To>(p.first), static_cast<To>(p.second));
}

void
Cell::add_neighbor(
    std::optional<std::pair<u_int32_t, std::pair<u_int32_t, u_int32_t>>> neighbor_compute_cell)
{
    this->neighbor_compute_cells.push_back(neighbor_compute_cell);
}

void
Cell::add_neighbor_compute_cells()
{

    if (this->shape == computeCellShape::square) {

        // Note: The coordinates are of type unsigned int and we need to do arithematics that
        // may give negative int values. Therefore, we cast them to signed int
        auto coordinate_signed = convert_internal_type_of_pair<int32_t>(this->cooridates);
        int32_t cc_coordinate_x = coordinate_signed.first;
        int32_t cc_coordinate_y = coordinate_signed.second;

        // Left neighbor
        SignedCoordinates left_neighbor =
            std::pair<int32_t, int32_t>(cc_coordinate_x - 1, cc_coordinate_y);
        if (this->cc_exists(left_neighbor)) {
            auto left_neighbor_unsigned = convert_internal_type_of_pair<u_int32_t>(left_neighbor);

            auto left_neighbor_id = Cell::cc_cooridinate_to_id(
                left_neighbor_unsigned, this->shape, this->dim_x, this->dim_y);

            this->add_neighbor(std::pair<u_int32_t, std::pair<u_int32_t, u_int32_t>>(
                left_neighbor_id, left_neighbor_unsigned));
        } else {
            this->add_neighbor(std::nullopt);
        }

        // Up neighbor
        SignedCoordinates up_neighbor =
            std::pair<int32_t, int32_t>(cc_coordinate_x, cc_coordinate_y - 1);
        if (this->cc_exists(up_neighbor)) {
            auto up_neighbor_unsigned = convert_internal_type_of_pair<u_int32_t>(up_neighbor);

            auto up_neighbor_id = Cell::cc_cooridinate_to_id(
                up_neighbor_unsigned, this->shape, this->dim_x, this->dim_y);

            this->add_neighbor(std::pair<u_int32_t, std::pair<u_int32_t, u_int32_t>>(
                up_neighbor_id, up_neighbor_unsigned));
        } else {
            this->add_neighbor(std::nullopt);
        }
        // Right neighbor
        SignedCoordinates right_neighbor =
            std::pair<int32_t, int32_t>(cc_coordinate_x + 1, cc_coordinate_y);
        if (this->cc_exists(right_neighbor)) {
            auto right_neighbor_unsigned = convert_internal_type_of_pair<u_int32_t>(right_neighbor);

            auto right_neighbor_id = Cell::cc_cooridinate_to_id(
                right_neighbor_unsigned, this->shape, this->dim_x, this->dim_y);

            this->add_neighbor(std::pair<u_int32_t, std::pair<u_int32_t, u_int32_t>>(
                right_neighbor_id, right_neighbor_unsigned));
        } else {
            this->add_neighbor(std::nullopt);
        }
        // Down neighbor
        SignedCoordinates down_neighbor =
            std::pair<int32_t, int32_t>(cc_coordinate_x, cc_coordinate_y + 1);
        if (this->cc_exists(down_neighbor)) {
            auto down_neighbor_unsigned = convert_internal_type_of_pair<u_int32_t>(down_neighbor);

            auto down_neighbor_id = Cell::cc_cooridinate_to_id(
                down_neighbor_unsigned, this->shape, this->dim_x, this->dim_y);

            this->add_neighbor(std::pair<u_int32_t, std::pair<u_int32_t, u_int32_t>>(
                down_neighbor_id, down_neighbor_unsigned));
        } else {
            this->add_neighbor(std::nullopt);
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

inline bool
Cell::cc_exists(const SignedCoordinates cc_coordinate)
{
    auto [cc_coordinate_x, cc_coordinate_y] = cc_coordinate;
    if (this->shape == computeCellShape::square) {

        // If invalid
        if ((cc_coordinate_x < 0) || (cc_coordinate_x >= this->dim_y) || (cc_coordinate_y < 0) ||
            (cc_coordinate_y >= this->dim_x)) {
            return false;
        } else {
            return true;
        }
    }
    // Shape not supported
    std::cerr << Cell::get_compute_cell_shape_name(this->shape) << " not supported!\n";
    exit(0);
}

std::string
Cell::get_compute_cell_shape_name(computeCellShape shape)
{
    switch (shape) {
        case (computeCellShape::block_1D):
            return std::string("block_1D");
            break;
        case (computeCellShape::triangular):
            return std::string("triangular");
            break;
        case (computeCellShape::square):
            return std::string("square");
            break;
        case (computeCellShape::hexagon):
            return std::string("hexagon");
            break;

        default:
            return std::string("Invalid Shape");
            break;
    }
}

computeCellShape
Cell::get_compute_cell_shape_enum(std::string shape)
{
    if (shape == "block_1D") {
        return computeCellShape::block_1D;
    } else if (shape == "triangular") {
        return computeCellShape::triangular;
    } else if (shape == "sqaure") {
        return computeCellShape::square;
    } else if (shape == "hexagon") {
        return computeCellShape::hexagon;
    } else {
        return computeCellShape::computeCellShape_invalid;
    }
}

u_int32_t
Cell::get_number_of_neighbors(computeCellShape shape_in)
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

std::pair<u_int32_t, u_int32_t>
Cell::cc_id_to_cooridinate(u_int32_t cc_id,
                           computeCellShape shape_,
                           u_int32_t dim_x,
                           u_int32_t dim_y)
{

    if (shape_ == computeCellShape::square) {

        return std::pair<u_int32_t, u_int32_t>(cc_id % dim_y, cc_id / dim_y);
    }
    // Shape not supported
    std::cerr << Cell::get_compute_cell_shape_name(shape_) << " not supported!\n";
    exit(0);
}

u_int32_t
Cell::cc_cooridinate_to_id(std::pair<u_int32_t, u_int32_t> cc_cooridinate,
                           computeCellShape shape_,
                           u_int32_t dim_x,
                           u_int32_t dim_y)
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
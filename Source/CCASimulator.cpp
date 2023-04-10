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
#include "Action.hpp"
#include "Address.hpp"
#include "ComputeCell.hpp"
#include "Constants.hpp"
#include "Operon.hpp"
#include "Task.hpp"

#include "memory_management.hpp"

/* #include <map>
#include <queue> */
#include <stdlib.h>

template<typename To, typename From>
inline std::pair<To, To>
convert_internal_type_of_pair(const std::pair<From, From>& p)
{
    return std::make_pair(static_cast<To>(p.first), static_cast<To>(p.second));
}
/*
CCASimulator::CCASimulator(computeCellShape shape_in,
                           u_int32_t dim_in,
                           u_int32_t total_compute_cells_in)
    : shape_of_compute_cells(shape_in)
    , dim(dim_in)
    , total_compute_cells(total_compute_cells_in)
{
    this->global_active_cc = false;
    this->total_cycles = 0;
    this->create_the_chip();
} */

// Chip is coordinates are from top-left....
inline std::pair<u_int32_t, u_int32_t>
CCASimulator::get_compute_cell_coordinates(u_int32_t cc_id,
                                           computeCellShape shape_of_compute_cells,
                                           u_int32_t dim_x,
                                           u_int32_t dim_y)
{
    // std::cout << "cc_id: " << cc_id << " dim_x: " << dim_x << " dim_y: " << dim_y << " ---> (" <<
    // cc_id % dim_y << ", " << cc_id / dim_y << ")\n";
    return std::pair<u_int32_t, u_int32_t>(cc_id % dim_y, cc_id / dim_y);
}

std::pair<u_int32_t, u_int32_t>
CCASimulator::cc_id_to_cooridinate(u_int32_t cc_id)
{
    return ComputeCell::cc_id_to_cooridinate(
        cc_id, this->shape_of_compute_cells, this->dim_x, this->dim_y);
}

u_int32_t
CCASimulator::cc_cooridinate_to_id(std::pair<u_int32_t, u_int32_t> cc_cooridinate)
{

    return ComputeCell::cc_cooridinate_to_id(
        cc_cooridinate, this->shape_of_compute_cells, this->dim_x, this->dim_y);
}

inline bool
CCASimulator::cc_exists(const SignedCoordinates cc_coordinate)
{
    auto [cc_coordinate_x, cc_coordinate_y] = cc_coordinate;
    if (this->shape_of_compute_cells == computeCellShape::square) {

        // If invalid
        if ((cc_coordinate_x < 0) || (cc_coordinate_x >= this->dim_y) || (cc_coordinate_y < 0) ||
            (cc_coordinate_y >= this->dim_x)) {
            return false;
        } else {
            return true;
        }
    }
    // Shape not supported
    std::cerr << ComputeCell::get_compute_cell_shape_name(this->shape_of_compute_cells)
              << " not supported!\n";
    exit(0);
}

void
CCASimulator::add_neighbor_compute_cells(std::shared_ptr<ComputeCell> cc)
{

    if (this->shape_of_compute_cells == computeCellShape::square) {

        // Note: The coordinates are of type unsigned int and we need to do arithematics that
        // may give negative int values. Therefore, we cast them to signed int
        auto coordinate_signed = convert_internal_type_of_pair<int32_t>(cc->cooridates);
        int32_t cc_coordinate_x = coordinate_signed.first;
        int32_t cc_coordinate_y = coordinate_signed.second;

        // Left neighbor
        SignedCoordinates left_neighbor =
            std::pair<int32_t, int32_t>(cc_coordinate_x - 1, cc_coordinate_y);
        if (this->cc_exists(left_neighbor)) {
            auto left_neighbor_unsigned = convert_internal_type_of_pair<u_int32_t>(left_neighbor);

            auto left_neighbor_id = this->cc_cooridinate_to_id(left_neighbor_unsigned);

            cc->add_neighbor(std::pair<u_int32_t, std::pair<u_int32_t, u_int32_t>>(
                left_neighbor_id, left_neighbor_unsigned));
        } else {
            // cc.add_neighbor(std::nullopt);
        }

        // Up neighbor
        SignedCoordinates up_neighbor =
            std::pair<int32_t, int32_t>(cc_coordinate_x, cc_coordinate_y - 1);
        if (this->cc_exists(up_neighbor)) {
            auto up_neighbor_unsigned = convert_internal_type_of_pair<u_int32_t>(up_neighbor);

            auto up_neighbor_id = this->cc_cooridinate_to_id(up_neighbor_unsigned);

            cc->add_neighbor(std::pair<u_int32_t, std::pair<u_int32_t, u_int32_t>>(
                up_neighbor_id, up_neighbor_unsigned));
        } else {
            // cc.add_neighbor(std::nullopt);
        }
        // Right neighbor
        SignedCoordinates right_neighbor =
            std::pair<int32_t, int32_t>(cc_coordinate_x + 1, cc_coordinate_y);
        if (this->cc_exists(right_neighbor)) {
            auto right_neighbor_unsigned = convert_internal_type_of_pair<u_int32_t>(right_neighbor);

            auto right_neighbor_id = this->cc_cooridinate_to_id(right_neighbor_unsigned);

            cc->add_neighbor(std::pair<u_int32_t, std::pair<u_int32_t, u_int32_t>>(
                right_neighbor_id, right_neighbor_unsigned));
        } else {
            // cc.add_neighbor(std::nullopt);
        }
        // Down neighbor
        SignedCoordinates down_neighbor =
            std::pair<int32_t, int32_t>(cc_coordinate_x, cc_coordinate_y + 1);
        if (this->cc_exists(down_neighbor)) {
            auto down_neighbor_unsigned = convert_internal_type_of_pair<u_int32_t>(down_neighbor);

            auto down_neighbor_id = this->cc_cooridinate_to_id(down_neighbor_unsigned);

            cc->add_neighbor(std::pair<u_int32_t, std::pair<u_int32_t, u_int32_t>>(
                down_neighbor_id, down_neighbor_unsigned));
        } else {
            // cc.add_neighbor(std::nullopt);
        }

    } else if (this->shape_of_compute_cells == computeCellShape::block_1D) {

        std::cerr << ComputeCell::get_compute_cell_shape_name(this->shape_of_compute_cells)
                  << " not supported!\n";
        exit(0);

        /*  u_int32_t right_neighbor = (i == total_compute_cells - 1) ? 0 : i + 1;
         this->CCA_chip.back()->add_neighbor(right_neighbor);
         u_int32_t left_neighbor = (i == 0) ? total_compute_cells - 1 : i - 1;
         this->CCA_chip.back()->add_neighbor(left_neighbor);
         this->shape_of_compute_cells == computeCellShape::square */
    } else {
        // Shape not supported
        std::cerr << ComputeCell::get_compute_cell_shape_name(this->shape_of_compute_cells)
                  << " not supported!\n";
        exit(0);
    }
}

void
CCASimulator::create_the_chip()
{

    // Cannot simply openmp parallelize this. It is very atomic.
    for (u_int32_t i = 0; i < this->total_compute_cells; i++) {

        // Create individual compute cells of shape computeCellShape::block_1D
        this->CCA_chip.push_back(std::make_shared<ComputeCell>(
            i,
            shape_of_compute_cells,
            this->get_compute_cell_coordinates(i, shape_of_compute_cells, this->dim_x, this->dim_y),
            this->dim_x,
            this->dim_y,
            2 * 1024 * 1024)); // 2 MB

        // Assign neighbor CCs to this CC. This is based on the Shape and Dim
        this->add_neighbor_compute_cells(this->CCA_chip.back());

        if constexpr (debug_code) {
            std::cout << *this->CCA_chip.back().get();
        }
    }
}

std::optional<Address>
CCASimulator::allocate_and_insert_object_on_cc(u_int32_t cc_id, void* obj, size_t size_of_obj)
{
    return this->CCA_chip[cc_id]->create_object_in_memory(obj, size_of_obj);
}

void
CCASimulator::run_simulation()
{
    // TODO: later we can remove this and implement the termination detection itself. But for
    // now this works.
    this->global_active_cc = true;
    this->total_cycles = 0;

    while (this->global_active_cc) {
        this->global_active_cc = false;

#pragma omp parallel for reduction(| : this->global_active_cc)
        for (int i = 0; i < this->CCA_chip.size(); i++) {
            if (this->CCA_chip[i]->is_compute_cell_active()) {
                global_active_cc |= this->CCA_chip[i]->run_a_computation_cycle();
            }
        }
        total_cycles++;
    }
}

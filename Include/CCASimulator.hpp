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

#ifndef CCASimulator_HPP
#define CCASimulator_HPP

#include "Action.hpp"
#include "Address.hpp"
#include "ComputeCell.hpp"
#include "Constants.hpp"
#include "Operon.hpp"
#include "Task.hpp"

#include "memory_management.hpp"

#include <map>
#include <queue>
#include <stdlib.h>

typedef std::pair<int32_t, int32_t> SignedCoordinates;

/* template<typename To, typename From>
inline std::pair<To, To>
convert_internal_type_of_pair(const std::pair<From, From>& p)
{
    return std::make_pair(static_cast<To>(p.first), static_cast<To>(p.second));
} */

class CCASimulator
{
  public:
    computeCellShape shape_of_compute_cells;
    u_int32_t dim;
    u_int32_t total_compute_cells;

    // Declare the CCA Chip that is composed of ComputeCell(s)
    std::vector<std::shared_ptr<ComputeCell>> CCA_chip;

    bool global_active_cc;
    u_long total_cycles;

    CCASimulator(computeCellShape shape_in, u_int32_t dim_in, u_int32_t total_compute_cells_in)
        : shape_of_compute_cells(shape_in)
        , dim(dim_in)
        , total_compute_cells(total_compute_cells_in)
    {
        this->global_active_cc = false;
        this->total_cycles = 0;
        this->create_the_chip();
    }

    inline std::pair<u_int32_t, u_int32_t> get_compute_cell_coordinates(
        u_int32_t cc_id,
        computeCellShape shape_of_compute_cells,
        u_int32_t dim);

    std::pair<u_int32_t, u_int32_t> cc_id_to_cooridinate(u_int32_t cc_id);

    u_int32_t cc_cooridinate_to_id(std::pair<u_int32_t, u_int32_t> cc_cooridinate);

    inline bool cc_exists(const SignedCoordinates cc_coordinate);

    void add_neighbor_compute_cells(std::shared_ptr<ComputeCell> cc);

    void create_the_chip();
    std::optional<Address> allocate_and_insert_object_on_cc(u_int32_t cc_id,
                                                            void* obj,
                                                            size_t size_of_obj);

    void run_simulation();
};

#endif // CCASimulator_HPP
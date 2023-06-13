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

#ifndef ROUTING_HPP
#define ROUTING_HPP

#include "ComputeCell.hpp"
#include "Operon.hpp"
#include "SinkCell.hpp"

#include <cassert>

struct Routing
{
  public:
    template<class SomeCellType>
    static u_int32_t get_next_move(std::vector<std::shared_ptr<Cell>>& CCA_chip,
                                   Operon& operon,
                                   u_int32_t current_cc_id,
                                   u_int32_t routing_algorithm_id)
    {
        u_int32_t dst_cc_id = operon.first.dst_cc_id;
        // To be routed in the mesh, by default simply
        u_int32_t routing_cell_id = dst_cc_id;

        auto dst_compute_cell = std::dynamic_pointer_cast<ComputeCell>(CCA_chip[dst_cc_id]);
        assert(dst_compute_cell != nullptr);

        std::shared_ptr<SomeCellType> current_compute_cell =
            std::dynamic_pointer_cast<SomeCellType>(CCA_chip[current_cc_id]);
        assert(current_compute_cell != nullptr);

        Coordinates dst_cc_coordinates = Cell::cc_id_to_cooridinate(
            dst_cc_id, current_compute_cell->shape, current_compute_cell->dim_y);

        //std::cout << "get_next_move : routing id: " << routing_algorithm_id << "\n";

        // Routing 0: Aggresively use the H-tree (low latency network)

        bool src_dst_are_on_different_sink_cells = true;

        if (current_compute_cell->type == CellType::compute_cell) {
            src_dst_are_on_different_sink_cells =
                (current_compute_cell->sink_cell != dst_compute_cell->sink_cell);
        }
        // If it is not nearby AND not in the same sinkcell (Htree block) then
        // route it in second layer netowrk
        if (!current_compute_cell->check_cut_off_distance(dst_cc_coordinates) &&
            src_dst_are_on_different_sink_cells) {
            routing_cell_id = Cell::cc_cooridinate_to_id(current_compute_cell->sink_cell.value(),
                                                         current_compute_cell->shape,
                                                         current_compute_cell->dim_y);
        }

        return routing_cell_id;
    }
};

#endif // ROUTING_HPP

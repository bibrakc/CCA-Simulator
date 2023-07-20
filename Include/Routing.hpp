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
#include "SinkCell.hpp"

#include <cassert>

struct Routing
{
  public:
    template<class SomeCellType>
    static auto routing_0_aggressively_use_htree(std::vector<std::shared_ptr<Cell>>& CCA_chip,
                                                 Operon& operon,
                                                 u_int32_t current_cc_id)
        -> std::optional<u_int32_t>
    {
        // Routing 1: Aggresively use the mesh. This is a static routing algrithm.

        // u_int32_t src_cc_id = operon.first.src_cc_id;
        u_int32_t dst_cc_id = operon.first.dst_cc_id;

        auto dst_compute_cell = std::dynamic_pointer_cast<ComputeCell>(CCA_chip[dst_cc_id]);
        assert(dst_compute_cell != nullptr);

        std::shared_ptr<SomeCellType> const current_compute_cell =
            std::dynamic_pointer_cast<SomeCellType>(CCA_chip[current_cc_id]);
        assert(current_compute_cell != nullptr);

        Coordinates const dst_cc_coordinates = Cell::cc_id_to_cooridinate(
            dst_cc_id, current_compute_cell->shape, current_compute_cell->dim_y);

        bool const use_mesh_network =
            current_compute_cell->check_cut_off_distance(dst_cc_coordinates);

        // For type ComputeCell
        if constexpr (std::is_same_v<SomeCellType, ComputeCell>) {

            // By default assume they are on different sink cells
            bool src_dst_are_on_different_sink_cells = true;
            src_dst_are_on_different_sink_cells =
                (current_compute_cell->sink_cell != dst_compute_cell->sink_cell);

            // If it is not nearby AND not in the same sinkcell (Htree block) then
            // route it in second layer network
            if (!use_mesh_network && src_dst_are_on_different_sink_cells) {
                return Cell::cc_cooridinate_to_id(current_compute_cell->sink_cell.value(),
                                                  current_compute_cell->shape,
                                                  current_compute_cell->dim_y);
            }
            return dst_cc_id;

        } else {
            // For SinkCell
            if (use_mesh_network) {
                return dst_cc_id;
            }
        }

        // This means that SinkCell returns std::nullptr, which means it will send the operon  to
        // the low latency Htree network.
        return std::nullopt;
    }

    template<class SomeCellType>
    static auto routing_1_use_mesh_more_often(std::vector<std::shared_ptr<Cell>>& CCA_chip,
                                              Operon& operon,
                                              u_int32_t current_cc_id) -> std::optional<u_int32_t>
    {
        // Routing 1: Try to use the mesh network more often.
        u_int32_t const src_cc_id = operon.first.src_cc_id;
        u_int32_t dst_cc_id = operon.first.dst_cc_id;

        /* auto dst_compute_cell = std::dynamic_pointer_cast<ComputeCell>(CCA_chip[dst_cc_id]);
        assert(dst_compute_cell != nullptr); */

        std::shared_ptr<SomeCellType> const current_compute_cell =
            std::dynamic_pointer_cast<SomeCellType>(CCA_chip[current_cc_id]);
        assert(current_compute_cell != nullptr);

        Coordinates const src_cc_coordinates = Cell::cc_id_to_cooridinate(
            src_cc_id, current_compute_cell->shape, current_compute_cell->dim_y);

        Coordinates const dst_cc_coordinates = Cell::cc_id_to_cooridinate(
            dst_cc_id, current_compute_cell->shape, current_compute_cell->dim_y);

        bool const use_mesh_network =
            current_compute_cell->should_I_use_mesh(src_cc_coordinates, dst_cc_coordinates);

        if (use_mesh_network) {
            return dst_cc_id;
        } /* std::cout << "CC: " << current_compute_cell->id << " Sending (" << src_cc_id << ", "
          << dst_cc_id << " in Mesh\n"; */
        if constexpr (std::is_same_v<SomeCellType, ComputeCell>) {

            // Route it in second layer netowrk
            return Cell::cc_cooridinate_to_id(current_compute_cell->sink_cell.value(),
                                              current_compute_cell->shape,
                                              current_compute_cell->dim_y);
        } else {
            // This means that SinkCell returns std::nullptr, which means it will send the
            // operon  to the low latency Htree network.s
            return std::nullopt;
        }

        // This means that SinkCell returns std::nullptr, which means it will send the operon to
        // the low latency Htree network. And if by any programming bug the ComputeCell returns this
        // then program will crash on bad access to optional.
        return std::nullopt;
    }

    template<class SomeCellType>
    static auto get_next_move(std::vector<std::shared_ptr<Cell>>& CCA_chip,
                              Operon& operon,
                              u_int32_t current_cc_id,
                              u_int32_t routing_algorithm_id) -> std::optional<u_int32_t>
    {
        // std::cout << "routing_algorithm_id = " << routing_algorithm_id << "\n";

        switch (routing_algorithm_id) {
            case 0:
                // Routing 0: Aggressively use the Htree.
                return routing_0_aggressively_use_htree<SomeCellType>(
                    CCA_chip, operon, current_cc_id);
                break;

            case 1:
                // Routing 1: Aggresively use the mesh. This is a static routing algrithm.
                return routing_1_use_mesh_more_often<SomeCellType>(CCA_chip, operon, current_cc_id);
                break;

            default:
                // Default is Routing 0: Aggressively use the Htree
                return routing_0_aggressively_use_htree<SomeCellType>(
                    CCA_chip, operon, current_cc_id);
                break;
        }
    }
};

#endif // ROUTING_HPP

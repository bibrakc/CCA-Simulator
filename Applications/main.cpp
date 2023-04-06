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

#include "Action.hpp"
#include "Address.hpp"
#include "ComputeCell.hpp"
#include "Constants.hpp"
#include "Enums.hpp"
#include "Operon.hpp"

#include "cmdparser.hpp"
#include "memory_management.hpp"

#include <chrono>
#include <iostream>
#include <map>
#include <queue>
#include <stdlib.h>

#include <omp.h>

// TODO: Curretly this SSSPAction class has nothing different than its base class Action. See if
// this inheritence makes sense later when the project matures.
class SSSPAction : public Action
{
  public:
    SSSPAction(const Address vertex_addr_in,
               actionType type,
               const bool ready,
               const int nargs_in,
               const std::shared_ptr<int[]>& args_in,
               eventId predicate_in,
               eventId work_in,
               eventId diffuse_in)
    {
        // std::cout << "sssp action constructor\n";

        this->obj_addr = vertex_addr_in;

        this->action_type = type;
        this->is_ready = ready;

        this->nargs = nargs_in;
        this->args = args_in;

        this->predicate = predicate_in;
        this->work = work_in;
        this->diffuse = diffuse_in;
    }
};

// Configuration related to the input data graph
u_int32_t total_vertices;

// Note: We could have put this function as part of the ComputeCell class but then it would have
// introduced application specific functionality into the compute cell. Perhaps, if we really want
// to introduce some special `insert_edge` instruction then we can rethink this. In anycase it makes
// no difference on the simulation. This is just a software engineering decision.
inline bool
insert_edge_by_address(std::vector<std::shared_ptr<ComputeCell>>& CCA_chip,
                       Address src_vertex_addr,
                       Address dst_vertex_addr)
{
    SimpleVertex* vertex =
        static_cast<SimpleVertex*>(CCA_chip[src_vertex_addr.cc_id]->get_object(src_vertex_addr));

    // Check if edges are not full
    // TODO: Later implement the hierarical parallel vertex object
    if (vertex->number_of_edges >= edges_max)
        return false;

    vertex->edges[vertex->number_of_edges] = dst_vertex_addr;
    vertex->number_of_edges++;

    return true;
}

inline bool
insert_edge_by_vertex_id(std::vector<std::shared_ptr<ComputeCell>>& CCA_chip,
                         u_int32_t src_vertex_id,
                         u_int32_t dst_vertex_id)
{

    std::cout << "Inserting " << src_vertex_id << " --> " << dst_vertex_id << "\n";
    Address src_vertex_addr = get_vertex_address_cyclic(
        src_vertex_id, total_vertices, sizeof(SimpleVertex), CCA_chip.size());

    Address dst_vertex_addr = get_vertex_address_cyclic(
        dst_vertex_id, total_vertices, sizeof(SimpleVertex), CCA_chip.size());

    return insert_edge_by_address(CCA_chip, src_vertex_addr, dst_vertex_addr);
}

void
configure_parser(cli::Parser& parser)
{
    parser.set_required<u_int32_t>("v", "vertices", "Number of vertices");
    parser.set_required<u_int32_t>(
        "d",
        "dimension",
        "Dimnesion of the shape. For example: A square is x^2. Only provide d (not d x d)");
    parser.set_required<std::string>("s", "shape", "Shape of the compute cell");
    // parser.set_required<u_int32_t>("cc", "computecells", "Number of compute cells");
}

int
main(int argc, char** argv)
{
    // Parse the commandline input
    cli::Parser parser(argc, argv);
    configure_parser(parser);
    parser.run_and_exit_if_error();

    // Configuration related to the input data graph
    total_vertices = parser.get<u_int32_t>("v");

    // Configuration related to the CCA Chip
    std::string shape_arg = parser.get<std::string>("s");
    computeCellShape shape_of_compute_cells;
    u_int32_t CCA_dim;
    u_int32_t total_compute_cells;

    if (shape_arg == "square") {
        shape_of_compute_cells = computeCellShape::sqaure;
        CCA_dim = parser.get<u_int32_t>("d");
        total_compute_cells = CCA_dim * CCA_dim;
    } else {
        std::cerr << "Error: Compute cell shape type " << shape_arg << " not supported.\n";
        exit(0);
    }

    std::cout << "CCA Chip Details:\n\tShape: "
              << ComputeCell::get_compute_cell_shape_name(shape_of_compute_cells) << "\n\tDim: " << CCA_dim
              << " x " << CCA_dim << "\n\tTotal Compute Cells: " << total_compute_cells << "\n";

    // Declare the CCA Chip that is composed of ComputeCell(s)
    std::vector<std::shared_ptr<ComputeCell>> CCA_chip;

    std::cout << "Populating the CCA Chip: \n";
    // Cannot simply openmp parallelize this. It is very atomic.
    for (int i = 0; i < total_compute_cells; i++) {

        // Create individual compute cells of shape computeCellShape::block_1D
        CCA_chip.push_back(std::make_shared<ComputeCell>(i, computeCellShape::block_1D));

        u_int32_t right_neighbor = (i == total_compute_cells - 1) ? 0 : i + 1;
        CCA_chip.back()->add_neighbor(right_neighbor);
        u_int32_t left_neighbor = (i == 0) ? total_compute_cells - 1 : i - 1;
        CCA_chip.back()->add_neighbor(left_neighbor);
    }

    std::cout << "Allocating vertices cyclically on the CCA Chip: \n";

    for (int i = 0; i < total_vertices; i++) {

        // put a vertex in memory
        SimpleVertex vertex_;
        vertex_.id = i;
        vertex_.number_of_edges = 0;

        Address vertex_addr_cyclic = get_vertex_address_cyclic(
            vertex_.id, total_vertices, sizeof(SimpleVertex), total_compute_cells);

        u_int32_t cc_id = vertex_addr_cyclic.cc_id;

        std::optional<Address> vertex_addr =
            CCA_chip[cc_id]->create_object_in_memory<SimpleVertex>(vertex_);

        if (!vertex_addr) {
            std::cout << "Memory not allocated! Vertex ID: " << vertex_.id << "\n";
            continue;
        }

        /* std::cout << "vertex_.id = " << vertex_.id
                  << ", CCA_chip[cc_id]->id = " << CCA_chip[cc_id]->id
                  << ", vertex_addr = " << vertex_addr.value() << ", get_vertex_address = " <<
        vertex_addr_cyclic
                  << "\n"; */

        std::shared_ptr<int[]> args_x = std::make_shared<int[]>(2);
        args_x[0] = 1;
        args_x[1] = 7;

        CCA_chip[cc_id]->insert_action(std::make_shared<SSSPAction>(vertex_addr.value(),
                                                                    actionType::application_action,
                                                                    true,
                                                                    2,
                                                                    args_x,
                                                                    eventId::sssp_predicate,
                                                                    eventId::sssp_work,
                                                                    eventId::sssp_diffuse));
    }

    std::cout << "Populating vertices by inserting edges: \n";
    for (int i = 0; i < total_vertices; i++) {

        if (!insert_edge_by_vertex_id(CCA_chip, i, i + 1)) {
            std::cout << "Error! Edge not inserted successfully.\n";
        }
    }

    bool global_active_cc = true;
    u_long total_cycles = 0;
    std::cout << "Starting Execution on the CCA Chip: \n";
    auto start = std::chrono::steady_clock::now();

    while (global_active_cc) {
        global_active_cc = false;
        // for (auto& cc : CCA_chip) {
#pragma omp parallel for reduction(| : global_active_cc)
        for (int i = 0; i < CCA_chip.size(); i++) {
            // std::cout << "Running CC : " << cc->id << "\n\n";
            if (CCA_chip[i]->is_compute_cell_active()) {
                global_active_cc |= CCA_chip[i]->run_a_cycle();
            }
        }
        total_cycles++;
    }
    auto end = std::chrono::steady_clock::now();

    std::cout << "Total Cycles: " << total_cycles << "\n";
    std::cout << "Elapsed time in nanoseconds: "
              << std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() << " ns"
              << std::endl;

    std::cout << "Elapsed time in microseconds: "
              << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << " µs"
              << std::endl;

    std::cout << "Elapsed time in milliseconds: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms"
              << std::endl;

    std::cout << "Elapsed time in seconds: "
              << std::chrono::duration_cast<std::chrono::seconds>(end - start).count() << " sec\n";
    /*   args_x = nullptr;
      std::cout << "in main(): args_x.use_count() == " << args_x.use_count() << " (object @ "
                << args_x << ")\n"; */

    return 0;
}
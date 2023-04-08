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
#include "CCASimulator.hpp"
#include "ComputeCell.hpp"
#include "Constants.hpp"
#include "Enums.hpp"
#include "Function.hpp"
#include "Operon.hpp"

#include "cmdparser.hpp"
#include "memory_management.hpp"

// Datastructures
#include "SimpleVertex.hpp"

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
    ~SSSPAction() override
    { /* std::cout << "SSSPAction destructor\n"; */
    }
};

inline Operon
construct_operon(const u_int32_t cc_id, const Action& action)
{
    return std::pair<u_int32_t, Action>(cc_id, action);
}

int
sssp_predicate_func(ComputeCell& cc,
                    const Address& addr,
                    int nargs,
                    const std::shared_ptr<int[]>& args)
{
    std::cout << "in sssp_predicate" << std::endl;
    return 0;
}

int
sssp_work_func(ComputeCell& cc, const Address& addr, int nargs, const std::shared_ptr<int[]>& args)
{
    int x = args[0];
    int y = args[1];
    std::cout << "in sssp_work: x = " << x << ", y = " << y << std::endl;

    int result = x + y;
    return 0;
}
// TODO: try to move such code to ComputeCell or somewhere
inline Task
send_operon(ComputeCell& cc, Operon operon_in)
{

    return std::pair<taskType, Task_func>(
        taskType::send_operon_task_type, Task_func([&cc, operon_in]() {
            if (cc.staging_operon_from_logic != std::nullopt) {
                std::cerr
                    << "Bug! cc: " << cc.id
                    << " staging_operon_from_logic buffer is full! The program shouldn't have come "
                       "to send_operon\n";
                exit(0);
            }
            std::cout << "Sending operon from cc " << cc.id << " to cc << " << operon_in.first
                      << "\n";
            cc.staging_operon_from_logic = operon_in;
        }));
}

int
sssp_diffuse_func(ComputeCell& cc,
                  const Address& addr,
                  int nargs,
                  const std::shared_ptr<int[]>& args)
{
    // std::cout << "in sssp_diffuse: " << std::endl;
    // std::cout << "(" << addr.cc_id << ", " << addr.addr << ")" << std::endl;
    // std::cout << addr  << std::endl;

    // cc.print_SimpleVertex(addr);
    SimpleVertex<Address>* v = static_cast<SimpleVertex<Address>*>(cc.get_object(addr));
    for (int i = 0; i < v->number_of_edges; i++) {
        std::string message = "Send from ";
        message += std::to_string(v->id) + " --> (" + std::to_string(v->edges[i].edge.cc_id) +
                   ", " + std::to_string(v->edges[i].edge.addr) + ")\n";

        std::shared_ptr<int[]> args_x = std::make_shared<int[]>(2);
        args_x[0] = static_cast<int>(v->id);
        args_x[1] = static_cast<int>(cc.id);

        /*       std::shared_ptr action = std::make_shared<SSSPAction>(v->edges[i],
                                                                    actionType::application_action,
                                                                    true,
                                                                    2,
                                                                    args_x,
                                                                    eventId::sssp_predicate,
                                                                    eventId::sssp_work,
                                                                    eventId::sssp_diffuse); */

        SSSPAction action(v->edges[i].edge,
                          actionType::application_action,
                          true,
                          2,
                          args_x,
                          eventId::sssp_predicate,
                          eventId::sssp_work,
                          eventId::sssp_diffuse);
        Operon operon_to_send = construct_operon(cc.id, action);
        cc.task_queue.push(send_operon(cc, operon_to_send));
    }

    return 0;
}

// Later create a register function to do these from the main. Help with using the simulator in more
// of an API style
std::map<eventId, handler_func> event_handlers = { { eventId::sssp_predicate, sssp_predicate_func },
                                                   { eventId::sssp_work, sssp_work_func },
                                                   { eventId::sssp_diffuse, sssp_diffuse_func } };

// Configuration related to the input data graph
u_int32_t total_vertices;

// Note: We could have put this function as part of the ComputeCell class but then it would have
// introduced application specific functionality into the compute cell. Perhaps, if we really want
// to introduce some special `insert_edge` instruction then we can rethink this. In anycase it makes
// no difference on the simulation. This is just a software engineering decision.
inline bool
insert_edge_by_address(std::vector<std::shared_ptr<ComputeCell>>& CCA_chip,
                       Address src_vertex_addr,
                       Address dst_vertex_addr,
                       u_int32_t edge_weight)
{
    SimpleVertex<Address>* vertex = static_cast<SimpleVertex<Address>*>(
        CCA_chip[src_vertex_addr.cc_id]->get_object(src_vertex_addr));

    // Check if edges are not full
    // TODO: Later implement the hierarical parallel vertex object
    if (vertex->number_of_edges >= edges_max)
        return false;

    vertex->edges[vertex->number_of_edges].edge = dst_vertex_addr;
    vertex->edges[vertex->number_of_edges].weight = edge_weight;
    vertex->number_of_edges++;

    return true;
}

inline bool
insert_edge_by_vertex_id(std::vector<std::shared_ptr<ComputeCell>>& CCA_chip,
                         u_int32_t src_vertex_id,
                         u_int32_t dst_vertex_id,
                         u_int32_t edge_weight)
{

    /*  std::cout << "Inserting " << src_vertex_id << " --> " << dst_vertex_id << "\n"; */
    Address src_vertex_addr =
        get_object_address_cyclic(src_vertex_id, sizeof(SimpleVertex<Address>), CCA_chip.size());

    Address dst_vertex_addr =
        get_object_address_cyclic(dst_vertex_id, sizeof(SimpleVertex<Address>), CCA_chip.size());

    return insert_edge_by_address(CCA_chip, src_vertex_addr, dst_vertex_addr, edge_weight);
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

class Graph
{
  public:
    u_int32_t total_vertices;
    std::shared_ptr<SimpleVertex<u_int32_t>[]> vertices;

    // TODO: Read this from file
    Graph(u_int32_t total_vertices_in)
    {
        /* std::cout << "In Graph Constructor\n"; */
        this->total_vertices = total_vertices_in;
        this->vertices = std::make_shared<SimpleVertex<u_int32_t>[]>(this->total_vertices);

        for (int i = 0; i < this->total_vertices; i++) {
            /* std::cout << "In Graph Constructor -- Adding edge in vertex " << i << "\n"; */
            this->vertices[i].id = i;

            // Check if edges are not full
            if (this->vertices[i].number_of_edges >= edges_max) {
                std::cerr << "Cannot add more edges to Vertex: " << i << "\n";
                continue;
            }

            this->vertices[i].edges[this->vertices[i].number_of_edges].edge = i + 1;
            this->vertices[i].edges[this->vertices[i].number_of_edges].weight = 5;
            this->vertices[i].number_of_edges++;
        }
        /* std::cout << "Leaving Graph Constructor\n"; */
    }
    ~Graph()
    { /*  std::cout << "In Graph Destructor\n";  */
    }
};

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
        shape_of_compute_cells = computeCellShape::square;
        CCA_dim = parser.get<u_int32_t>("d");
        total_compute_cells = CCA_dim * CCA_dim;
    } else {
        std::cerr << "Error: Compute cell shape type " << shape_arg << " not supported.\n";
        exit(0);
    }

    std::cout << "Create the simulation environment that includes the CCA Chip: \n";
    // Create the simulation environment
    CCASimulator cca_sqaure_simulator(shape_of_compute_cells, CCA_dim, total_compute_cells);

    std::cout << "CCA Chip Details:\n\tShape: "
              << ComputeCell::get_compute_cell_shape_name(
                     cca_sqaure_simulator.shape_of_compute_cells)
              << "\n\tDim: " << cca_sqaure_simulator.dim << " x " << cca_sqaure_simulator.dim
              << "\n\tTotal Compute Cells: " << cca_sqaure_simulator.total_compute_cells << "\n";

    // Generate or read the input data graph
    Graph input_graph(total_vertices);

    std::cout << "Allocating vertices cyclically on the CCA Chip: \n";
    { // Block so that the vertex_ object is contained in this scope. It is reused in the for loop
      // and we don't want it's constructor to be called everytime.

        SimpleVertex<Address> vertex_(0);
        for (int i = 0; i < input_graph.total_vertices; i++) {

            // Put a vertex in memory
            vertex_.id = i;

            // Get the Address of this vertex if it were to be cyclically allocated across the CCA
            // chip Note here we use SimpleVertex<Address> since the object is now going to be sent
            // to the CCA chip and there the address type is Address (not u_int32_t ID)
            Address vertex_addr_cyclic =
                get_object_address_cyclic(vertex_.id,
                                          sizeof(SimpleVertex<Address>),
                                          cca_sqaure_simulator.total_compute_cells);

            // Get the ID of the compute cell where this vertex is to be allocated
            u_int32_t cc_id = vertex_addr_cyclic.cc_id;

            std::optional<Address> vertex_addr =
                cca_sqaure_simulator.allocate_and_insert_object_on_cc(
                    cc_id, &vertex_, sizeof(SimpleVertex<Address>));

            if (!vertex_addr) {
                std::cout << "Memory not allocated! Vertex ID: " << input_graph.vertices[i].id
                          << "\n";
                continue;
            }

            /*  std::cout << "input_graph.vertices[i].id = " << input_graph.vertices[i].id
                       << ", cca_sqaure_simulator.CCA_chip[cc_id]->id = "
                       << cca_sqaure_simulator.CCA_chip[cc_id]->id
                       << ", vertex_addr = " << vertex_addr.value()
                       << ", get_vertex_address = " << vertex_addr_cyclic << "\n"; */

            std::shared_ptr<int[]> args_x = std::make_shared<int[]>(2);
            args_x[0] = 1;
            args_x[1] = 7;

            cca_sqaure_simulator.CCA_chip[cc_id]->insert_action(
                std::make_shared<SSSPAction>(vertex_addr.value(),
                                             actionType::application_action,
                                             true,
                                             2,
                                             args_x,
                                             eventId::sssp_predicate,
                                             eventId::sssp_work,
                                             eventId::sssp_diffuse));
        }
    }

    std::cout << "Populating vertices by inserting edges: \n";
    for (int i = 0; i < input_graph.total_vertices; i++) {
        u_int32_t src_vertex_id = input_graph.vertices[i].id;
        for (int j = 0; j < input_graph.vertices[i].number_of_edges; j++) {

            u_int32_t dst_vertex_id = input_graph.vertices[i].edges[j].edge;
            u_int32_t edge_weight = input_graph.vertices[i].edges[j].weight;
            if (!insert_edge_by_vertex_id(
                    cca_sqaure_simulator.CCA_chip, src_vertex_id, dst_vertex_id, edge_weight)) {
                std::cout << "Error! Edge (" << src_vertex_id << ", " << dst_vertex_id << ", "
                          << edge_weight << ") not inserted successfully.\n";
            }
        }
    }

    std::cout << "Starting Execution on the CCA Chip: \n";
    auto start = std::chrono::steady_clock::now();

    cca_sqaure_simulator.run_simulation();
    auto end = std::chrono::steady_clock::now();

    std::cout << "Total Cycles: " << cca_sqaure_simulator.total_cycles << "\n";
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
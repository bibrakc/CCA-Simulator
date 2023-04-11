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

static u_int32_t test_vertex;

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

    /*     // Copy constructor
        SSSPAction(const SSSPAction& ssspaction_) : Action(ssspaction_){

        } */
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
    SimpleVertex<Address>* v = static_cast<SimpleVertex<Address>*>(cc.get_object(addr));
    int incoming_distance = args[0];
    int origin_vertex = args[1];

    if (v->id == 35) {
        std::cout << "vertex ID : " << v->id
                  << " sssp_predicate | origin vertex: " << origin_vertex << " | incoming_distance = " << incoming_distance
                  << " v->sssp_distance = " << v->sssp_distance << std::endl;
    }

    if constexpr (debug_code) {
        std::cout << "vertex ID : " << v->id
                  << " in sssp_predicate | incoming_distance = " << incoming_distance
                  << " v->sssp_distance = " << v->sssp_distance << std::endl;
    }
    if (v->sssp_distance > incoming_distance) {
        return 1;
    }
    return 0;
}

int
sssp_work_func(ComputeCell& cc, const Address& addr, int nargs, const std::shared_ptr<int[]>& args)
{
    SimpleVertex<Address>* v = static_cast<SimpleVertex<Address>*>(cc.get_object(addr));
    int incoming_distance = args[0];

    // Update distance with the new distance
    v->sssp_distance = incoming_distance;
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
            if constexpr (debug_code) {
                std::cout << "Sending operon from cc: " << cc.id << " to cc: " << operon_in.first
                          << "\n";
            }

            cc.staging_operon_from_logic = operon_in;
        }));
}

// Return the number of diffusions created, i.e. v->number_of_edges. These are used for performance
// measurements
int
sssp_diffuse_func(ComputeCell& cc,
                  const Address& addr,
                  int nargs,
                  const std::shared_ptr<int[]>& args)
{
    SimpleVertex<Address>* v = static_cast<SimpleVertex<Address>*>(cc.get_object(addr));
    for (int i = 0; i < v->number_of_edges; i++) {

        if constexpr (debug_code) {
            std::string message = "Send from ";
            message += std::to_string(v->id) + " --> (" + std::to_string(v->edges[i].edge.cc_id) +
                       ", " + std::to_string(v->edges[i].edge.addr) + ")\n";

            std::cout << message;
        }

        // TODO: later convert this type int[] to something generic, perhaps std::forward args&& ...
        std::shared_ptr<int[]> args_x = std::make_shared<int[]>(2);
        args_x[0] = static_cast<int>(v->sssp_distance + v->edges[i].weight);
        args_x[1] = static_cast<int>(v->id);

        SSSPAction action(v->edges[i].edge,
                          actionType::application_action,
                          true,
                          2,
                          args_x,
                          eventId::sssp_predicate,
                          eventId::sssp_work,
                          eventId::sssp_diffuse);
        Operon operon_to_send = construct_operon(v->edges[i].edge.cc_id, action);
        cc.task_queue.push(send_operon(cc, operon_to_send));
    }
    // These many new actions were created
    cc.statistics.actions_created += v->number_of_edges;

    return 0;
}

// Later create a register function to do these from the main. Help with using the simulator in more
// of an API style
std::map<eventId, handler_func> event_handlers = { { eventId::sssp_predicate, sssp_predicate_func },
                                                   { eventId::sssp_work, sssp_work_func },
                                                   { eventId::sssp_diffuse, sssp_diffuse_func } };

// Configuration related to the input data graph
/* u_int32_t total_vertices; */

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
    // parser.set_required<u_int32_t>("v", "vertices", "Number of vertices");
    parser.set_required<std::string>("f", "graphfile", "Path to the input data graph file");
    parser.set_required<u_int32_t>("dx",
                                   "dimensionx",
                                   "Dimnesion of the shape in x direction. For example: A rectange "
                                   "chip is x*y. Provide dx such that dx x dy)");
    parser.set_required<u_int32_t>("dy",
                                   "dimensiony",
                                   "Dimnesion of the shape in y direction. For example: A rectange "
                                   "chip is x*y. Provide dy such that dx x dy)");
    parser.set_required<std::string>("s", "shape", "Shape of the compute cell");
    // parser.set_required<u_int32_t>("cc", "computecells", "Number of compute cells");

    parser.set_required<u_int32_t>("tv", "testvertex", "test vertex to print its sssp distance");
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
            this->vertices[i].id = i;
            this->vertices[i].number_of_edges = 0;
            // this->vertices[i]
        }
        /*         for (int i = 0; i < this->total_vertices; i++) {

                    this->vertices[i].id = i;

                    // Check if edges are not full
                    if (this->vertices[i].number_of_edges >= edges_max) {
                        std::cerr << "Cannot add more edges to Vertex: " << i << "\n";
                        continue;
                    }
                    / * // Creating a ring graph.
                    u_int32_t dst_vertex_id = (i == this->total_vertices - 1) ? 0 : i + 1;
                    this->add_edge(this->vertices[i], dst_vertex_id, 5); * /

                    // Creating a list graph.
                    u_int32_t dst_vertex_id = i + 1;
                    if (i != this->total_vertices - 1)
                        this->add_edge(this->vertices[i], dst_vertex_id, 5);
                } */

        /* std::cout << "Leaving Graph Constructor\n"; */
    }
    void add_edge(SimpleVertex<u_int32_t>& vertex, u_int32_t dst_vertex_id, u_int32_t weight)
    {
        vertex.edges[vertex.number_of_edges].edge = dst_vertex_id;
        vertex.edges[vertex.number_of_edges].weight = weight;
        vertex.number_of_edges++;
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
    // total_vertices = parser.get<u_int32_t>("v");
    std::string input_graph_path = parser.get<std::string>("f");

    // Configuration related to the CCA Chip
    std::string shape_arg = parser.get<std::string>("s");
    computeCellShape shape_of_compute_cells;
    u_int32_t CCA_dim_x, CCA_dim_y;
    u_int32_t total_compute_cells;

    if (shape_arg == "square") {
        shape_of_compute_cells = computeCellShape::square;
        CCA_dim_x = parser.get<u_int32_t>("dx");
        CCA_dim_y = parser.get<u_int32_t>("dy");
        total_compute_cells = CCA_dim_x * CCA_dim_y;
    } else {
        std::cerr << "Error: Compute cell shape type " << shape_arg << " not supported.\n";
        exit(0);
    }

    std::cout << "Create the simulation environment that includes the CCA Chip: \n";
    // Create the simulation environment
    CCASimulator cca_sqaure_simulator(
        shape_of_compute_cells, CCA_dim_x, CCA_dim_y, total_compute_cells);

    std::cout << "CCA Chip Details:\n\tShape: "
              << ComputeCell::get_compute_cell_shape_name(
                     cca_sqaure_simulator.shape_of_compute_cells)
              << "\n\tDim: " << cca_sqaure_simulator.dim_x << " x " << cca_sqaure_simulator.dim_y
              << "\n\tTotal Compute Cells: " << cca_sqaure_simulator.total_compute_cells << "\n";

    // Generate or read the input data graph
    FILE* input_graph_file_handler = NULL;

    if ((input_graph_file_handler = fopen(input_graph_path.c_str(), "r")) == NULL)
        return -1;

    u_int32_t total_vertices = 0;
    u_int32_t total_edges = 0;

    fscanf(input_graph_file_handler, "%d\t%d", &total_vertices, &total_vertices);
    fscanf(input_graph_file_handler, "%d", &total_edges);

    std::cout << "The graph: " << input_graph_path << " has total_vertices: " << total_vertices
              << " with " << total_edges << " egdes.\n";

    Graph input_graph(total_vertices);

    // Read from file and insert edges
    u_int32_t vertex_from;
    u_int32_t vertex_to;
    u_int32_t weight;
    for (int i = 0; i < total_edges; i++) {
        fscanf(input_graph_file_handler, "%d\t%d\t%d", &vertex_from, &vertex_to, &weight);
        input_graph.add_edge(input_graph.vertices[vertex_from], vertex_to, weight);
    }

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

            // only put action on a single vertex
            if (vertex_.id == 0) {

                std::shared_ptr<int[]> args_x = std::make_shared<int[]>(2);
                // Set distance to 0
                args_x[0] = 0;
                // origin
                args_x[1] = 0;
                
                cca_sqaure_simulator.CCA_chip[cc_id]->insert_action(
                    SSSPAction(vertex_addr.value(),
                               actionType::application_action,
                               true,
                               2,
                               args_x,
                               eventId::sssp_predicate,
                               eventId::sssp_work,
                               eventId::sssp_diffuse));
                cca_sqaure_simulator.CCA_chip[cc_id]->statistics.actions_created++;
            }
        }
    }

    std::cout << "Populating vertices by inserting edges: \n";
    for (int i = 0; i < input_graph.total_vertices; i++) {
        u_int32_t src_vertex_id = input_graph.vertices[i].id;
        // std::cout << "vertex id: " << src_vertex_id << " has edge: " <<
        // input_graph.vertices[i].number_of_edges << "\n";
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

    ///////////////
    test_vertex = parser.get<u_int32_t>("tv");

    Address test_vertex_addr = get_object_address_cyclic(
        test_vertex, sizeof(SimpleVertex<Address>), cca_sqaure_simulator.CCA_chip.size());

    SimpleVertex<Address>* v_test =
        (SimpleVertex<Address>*)cca_sqaure_simulator.CCA_chip[test_vertex_addr.cc_id]->get_object(
            test_vertex_addr);
    std::cout << "test_vertex_addr cc id " << test_vertex_addr.cc_id << " test vertex id "
              << v_test->id << " sssp distance = " << v_test->sssp_distance << "\n";

    ComputeCellStatistics simulation_statistics;
    for (auto& cc : cca_sqaure_simulator.CCA_chip) {
        simulation_statistics.actions_created += cc->statistics.actions_created;

        simulation_statistics.actions_pushed += cc->statistics.actions_pushed;

        simulation_statistics.actions_performed_work += cc->statistics.actions_performed_work;

        simulation_statistics.actions_false_on_predicate +=
            cc->statistics.actions_false_on_predicate;
        simulation_statistics.actions_invoked += cc->statistics.actions_invoked;
        simulation_statistics.stall_logic_on_network += cc->statistics.stall_logic_on_network;
        simulation_statistics.stall_network_on_recv += cc->statistics.stall_network_on_recv;
        simulation_statistics.stall_network_on_send += cc->statistics.stall_network_on_send;
    }

    std::cout << simulation_statistics;

    return 0;
}

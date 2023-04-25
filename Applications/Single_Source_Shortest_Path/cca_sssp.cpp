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
#include "Cell.hpp"
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

// std::ofstream
#include <fstream>
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
        this->obj_addr = vertex_addr_in;

        this->action_type = type;
        this->is_ready = ready;

        this->nargs = nargs_in;
        this->args = args_in;

        this->predicate = predicate_in;
        this->work = work_in;
        this->diffuse = diffuse_in;
    }

    ~SSSPAction() override {}
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

    if constexpr (debug_code) {
        std::cout << "vertex ID : " << v->id << " sssp_predicate | origin vertex: " << origin_vertex
                  << " | incoming_distance = " << incoming_distance
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
    Address src_vertex_addr =
        get_object_address_cyclic(src_vertex_id, sizeof(SimpleVertex<Address>), CCA_chip.size());

    Address dst_vertex_addr =
        get_object_address_cyclic(dst_vertex_id, sizeof(SimpleVertex<Address>), CCA_chip.size());

    return insert_edge_by_address(CCA_chip, src_vertex_addr, dst_vertex_addr, edge_weight);
}

void
configure_parser(cli::Parser& parser)
{
    parser.set_required<std::string>("f", "graphfile", "Path to the input data graph file");
    parser.set_required<std::string>("g",
                                     "graphname",
                                     "Name of the input graph used to set the name of the output "
                                     "file. Example: Erdos or anything");
    parser.set_required<u_int32_t>("dx",
                                   "dimensionx",
                                   "Dimnesion of the shape in x direction. For example: A rectange "
                                   "chip is x*y. Provide dx such that dx x dy)");
    parser.set_required<u_int32_t>("dy",
                                   "dimensiony",
                                   "Dimnesion of the shape in y direction. For example: A rectange "
                                   "chip is x*y. Provide dy such that dx x dy)");
    parser.set_required<std::string>("s", "shape", "Shape of the compute cell");
    parser.set_required<u_int32_t>("tv", "testvertex", "test vertex to print its sssp distance");
    parser.set_required<u_int32_t>(
        "root", "sssproot", "Root vertex for Single Source Shortest Path (SSSP)");
    parser.set_optional<u_int32_t>("m",
                                   "memory_per_cc",
                                   1 * 512 * 1024,
                                   "Memory per compute cell in bytes. Default is 0.5 MB");
    parser.set_optional<std::string>(
        "od", "outputdirectory", "./", "Path to the output file directory. Default: ./");
}

class Graph
{
  public:
    u_int32_t total_vertices;
    std::shared_ptr<SimpleVertex<u_int32_t>[]> vertices;

    Graph(u_int32_t total_vertices_in)
    {
        this->total_vertices = total_vertices_in;

        this->vertices = std::make_shared<SimpleVertex<u_int32_t>[]>(this->total_vertices);
        for (int i = 0; i < this->total_vertices; i++) {
            this->vertices[i].id = i;
            this->vertices[i].number_of_edges = 0;
        }
    }
    void add_edge(SimpleVertex<u_int32_t>& vertex, u_int32_t dst_vertex_id, u_int32_t weight)
    {
        vertex.edges[vertex.number_of_edges].edge = dst_vertex_id;
        vertex.edges[vertex.number_of_edges].weight = weight;
        vertex.number_of_edges++;
    }
    ~Graph() {}
};

int
main(int argc, char** argv)
{
    // Parse the commandline input
    cli::Parser parser(argc, argv);
    configure_parser(parser);
    parser.run_and_exit_if_error();

    // SSSP root vertex
    u_int32_t root_vertex = parser.get<u_int32_t>("root");

    // Test vertex to print its distance from the root
    test_vertex = parser.get<u_int32_t>("tv");

    // Configuration related to the input data graph
    std::string input_graph_path = parser.get<std::string>("f");
    std::string graph_name = parser.get<std::string>("g");

    // Optional output directory path
    std::string output_file_directory = parser.get<std::string>("od");

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

    // Get the memory per cc or use the default
    u_int32_t memory_per_cc = parser.get<u_int32_t>("m");

    std::cout << "Creating the simulation environment that includes the CCA Chip: \n";
    // Create the simulation environment
    CCASimulator cca_square_simulator(
        shape_of_compute_cells, CCA_dim_x, CCA_dim_y, total_compute_cells, memory_per_cc);

    std::cout << "\nCCA Chip Details:\n\tShape: "
              << ComputeCell::get_compute_cell_shape_name(
                     cca_square_simulator.shape_of_compute_cells)
              << "\n\tDim: " << cca_square_simulator.dim_x << " x " << cca_square_simulator.dim_y
              << "\n\tTotal Compute Cells: " << cca_square_simulator.total_compute_cells
              << "\n\tMemory Per Compute Cell: "
              << cca_square_simulator.memory_per_cc / static_cast<double>(1024) << " KB"
              << "\n\tTotal Chip Memory: "
              << cca_square_simulator.total_chip_memory / static_cast<double>(1024 * 1024) << " MB"
              << "\n\n";

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

    // Memory allocator for vertices allocation. Here we use cyclic allocator, which allocates
    // vertices (or objects) one per compute cell in round-robin fashion.
    std::unique_ptr<MemoryAlloctor> allocator = std::make_unique<CyclicMemoryAllocator>();

    // Store the address of vertices in a map so as to retrieve easily for edge insertion and other
    // tasks
    std::map<u_int32_t, Address> vertex_addresses;

    std::cout << "Allocating vertices cyclically on the CCA Chip: \n";
    { // Block so that the vertex_ object is contained in this scope. It is reused in the for loop
      // and we don't want it's constructor to be called everytime.

        SimpleVertex<Address> vertex_(0);
        for (int i = 0; i < input_graph.total_vertices; i++) {

            // Put a vertex in memory with id = i
            vertex_.id = i;

            // Get the ID of the compute cell where this vertex is to be allocated
            u_int32_t cc_id = allocator->get_next_available_cc(cca_square_simulator);

            // Get the Address of this vertex allocated on the CCA chip. Note here we use
            // SimpleVertex<Address> since the object is now going to be sent to the CCA chip and
            // there the address type is Address (not u_int32_t ID)
            std::optional<Address> vertex_addr =
                cca_square_simulator.allocate_and_insert_object_on_cc(
                    cc_id, &vertex_, sizeof(SimpleVertex<Address>));

            if (!vertex_addr) {
                std::cerr << "Error! Memory not allocated for Vertex ID: "
                          << input_graph.vertices[i].id << "\n";
                exit(0);
            }

            // Insert into the vertex_addresses map
            vertex_addresses[i] = vertex_addr.value();

            // Only put the SSSP seed action on a single vertex.
            // In this case SSSP root = root_vertex
            if (vertex_.id == root_vertex) {

                std::shared_ptr<int[]> args_x = std::make_shared<int[]>(2);
                // Set distance to 0
                args_x[0] = 0;
                // Origin vertex from where this action came
                args_x[1] = root_vertex;

                cca_square_simulator.CCA_chip[cc_id]->insert_action(
                    SSSPAction(vertex_addr.value(),
                               actionType::application_action,
                               true,
                               2,
                               args_x,
                               eventId::sssp_predicate,
                               eventId::sssp_work,
                               eventId::sssp_diffuse));
                cca_square_simulator.CCA_chip[cc_id]->statistics.actions_created++;
            }
        }
    }

    std::cout << "Populating vertices by inserting edges: \n";
    for (int i = 0; i < input_graph.total_vertices; i++) {
        u_int32_t src_vertex_id = input_graph.vertices[i].id;

        for (int j = 0; j < input_graph.vertices[i].number_of_edges; j++) {

            u_int32_t dst_vertex_id = input_graph.vertices[i].edges[j].edge;
            u_int32_t edge_weight = input_graph.vertices[i].edges[j].weight;

            if (!insert_edge_by_address(cca_square_simulator.CCA_chip,
                                        vertex_addresses[src_vertex_id],
                                        vertex_addresses[dst_vertex_id],
                                        edge_weight)) {
                std::cerr << "Error! Edge (" << src_vertex_id << ", " << dst_vertex_id << ", "
                          << edge_weight << ") not inserted successfully.\n";
                exit(0);
            }
        }
    }

    std::cout << "\nStarting Execution on the CCA Chip:\n\n";
    auto start = std::chrono::steady_clock::now();
    cca_square_simulator.run_simulation();
    auto end = std::chrono::steady_clock::now();

    std::cout << "Total Cycles: " << cca_square_simulator.total_cycles << "\n";

    std::cout << "Program elapsed time in milliseconds (This has nothing to do with the simulation "
                 "itself): "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms"
              << std::endl;

    Address test_vertex_addr = get_object_address_cyclic(
        test_vertex, sizeof(SimpleVertex<Address>), cca_square_simulator.CCA_chip.size());

    SimpleVertex<Address>* v_test =
        (SimpleVertex<Address>*)cca_square_simulator.CCA_chip[test_vertex_addr.cc_id]->get_object(
            test_vertex_addr);
    std::cout << "\nSSSP distance from vertex: " << root_vertex << " to vertex: " << v_test->id
              << " is: " << v_test->sssp_distance << "\n";

    ComputeCellStatistics simulation_statistics;
    for (auto& cc : cca_square_simulator.CCA_chip) {

        simulation_statistics += cc->statistics;
    }

    std::cout << simulation_statistics;

    // Write results to a file
    std::string output_file_name = "square_x_" + std::to_string(cca_square_simulator.dim_x) +
                                   "_y_" + std::to_string(cca_square_simulator.dim_y) + "_graph_" +
                                   graph_name + "_v_" + std::to_string(total_vertices) + "_e_" +
                                   std::to_string(total_edges);
    std::string output_file_path = output_file_directory + "/" + output_file_name;
    std::cout << "\nWriting results to output file: " << output_file_path << "\n";
    std::ofstream output_file(output_file_path);

    // Output CCA Chip details
    cca_square_simulator.generate_label(output_file);
    cca_square_simulator.output_description_in_a_single_line(output_file);

    // Output input graph details
    output_file << "graph_file\tvertices\tedges\troot_vertex\n"
                << input_graph_path << "\t" << total_vertices << "\t" << total_edges << "\t"
                << root_vertex << "\n";

    // Output total cycles, total actions, total actions performed work, total actions false on
    // predicate. TODO: Somehow put the resource usage as a percentage...?
    output_file
        << "total_cycles\ttotal_actions_invoked\ttotal_actions_performed_work\ttotal_actions_"
           "false_on_predicate\n"
        << cca_square_simulator.total_cycles << "\t" << simulation_statistics.actions_invoked
        << "\t" << simulation_statistics.actions_performed_work << "\t"
        << simulation_statistics.actions_false_on_predicate << "\n";

    // Output statistics for each compute cell
    simulation_statistics.generate_label(output_file);
    for (auto& cc : cca_square_simulator.CCA_chip) {
        cc->statistics.output_results_in_a_single_line(output_file, cc->id, cc->cooridates);
        if (&cc != &cca_square_simulator.CCA_chip.back()) {
            output_file << "\n";
        }
    }

    // Close the output file
    output_file.close();

    return 0;
}

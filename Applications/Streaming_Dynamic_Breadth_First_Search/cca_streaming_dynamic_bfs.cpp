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

#include "cca_streaming_dynamic_bfs.hpp"

// Datastructures
#include "CyclicMemoryAllocator.hpp"
#include "Graph.hpp"

#include <chrono>
#include <fstream>

// Declare the function event ids for the BFS action functions of predicate, work, and diffuse.
// In the main register the functions and get their ids
CCAFunctionEvent dynamic_bfs_predicate;
CCAFunctionEvent dynamic_bfs_work;
CCAFunctionEvent dynamic_bfs_diffuse_predicate;
CCAFunctionEvent dynamic_bfs_diffuse;

CCAFunctionEvent dynamic_bfs_insert_edge_predicate;
CCAFunctionEvent dynamic_bfs_insert_edge_work;
CCAFunctionEvent dynamic_bfs_insert_edge_diffuse_predicate;
CCAFunctionEvent dynamic_bfs_insert_edge_diffuse;

CCAFunctionEvent allocate;
CCAFunctionEvent dynamic_bfs_insert_edge_continuation_ghost_allocate_return;

auto
main(int argc, char** argv) -> int
{
    // Parse the commandline input
    cli::Parser parser(argc, argv);
    configure_parser(parser);
    parser.run_and_exit_if_error();

    std::cout << "Parsing Commandline Arguments: \n";
    BFSCommandLineArguments cmd_args(parser);

    std::cout << "Creating the simulation environment that includes the CCA Chip: \n";

    // Create the simulation environment
    CCASimulator cca_square_simulator(cmd_args.shape_of_compute_cells,
                                      cmd_args.hx,
                                      cmd_args.hy,
                                      cmd_args.hdepth,
                                      cmd_args.hbandwidth_max,
                                      cmd_args.memory_per_cc,
                                      cmd_args.mesh_type,
                                      cmd_args.routing_policy);

    // Print details of the CCA Chip.
    cca_square_simulator.print_discription(std::cout);

    // Read the input data graph.
    std::string input_graph_inc_1_path = cmd_args.input_graph_path + ".edgelist_1";
    Graph<BFSVertex<SimpleVertex<host_edge_type, edges_min>>> input_graph(input_graph_inc_1_path,
                                                                          true);

    std::cout << "Allocating vertices cyclically on the CCA Chip: \n";

    // Memory allocator for vertices allocation. Here we use cyclic allocator, which allocates
    // vertices (or objects) one per compute cell in round-robin fashion. This is different from
    // when the `RecursiveParallelVertex` allocates ghost vertices.
    // To avoid high degree vertex being allocated on the corners of the chip we start the cyclic
    // allocator from the center of the chip and later provide the `root` vertex to the graph
    // initializer in `transfer_graph_host_to_cca`.
    u_int32_t center_of_the_chip = (cca_square_simulator.dim_x * (cca_square_simulator.dim_y / 2)) +
                                   (cca_square_simulator.dim_x / 2);
    center_of_the_chip = 0;
    CyclicMemoryAllocator allocator(center_of_the_chip, cca_square_simulator.total_compute_cells);

    // Note: here we use BFSSimpleVertex<Address> since the vertex object is now going to be sent to
    // the CCA chip and there the address type is Address (not u_int32_t ID).
    input_graph.create_graph_vertices_host_to_cca<BFSVertex<ghost_type_level_1>>(
        cca_square_simulator,
        allocator,
        std::optional<u_int32_t>(cmd_args.root_vertex),
        cmd_args.shuffle_switch);

    // input_graph.print_vertices<BFSVertex<ghost_type_level_1>>(cca_square_simulator);
    //  exit(0);

    /*
    std::vector<u_int32_t> vertices_inbound_degree_zero =
        input_graph.get_vertices_ids_with_zero_in_degree();

    std::cout << "Vertices with in degree value 0: \n";
    for (const auto& vertex_id : vertices_inbound_degree_zero) {
        std::cout << vertex_id
                  << ", out_degree: " << input_graph.vertices[vertex_id].outbound_degree << "\n";
    }
    std::cout << std::endl; */

    // Get the vertices with out degree values equal to 0.
    /* std::vector<u_int32_t> vertices_outbound_degree_zero =
        input_graph.get_vertices_ids_with_zero_out_degree();

    std::cout << "Vertices with out degree value 0: \n";
    for (const auto& vertex_id : vertices_outbound_degree_zero) {
        std::cout << vertex_id
                  << ", out_degree: " << input_graph.vertices[vertex_id].outbound_degree << "\n";
    }
    std::cout << std::endl; */

    // Register the BFS action functions for predicate, work, and diffuse.
    dynamic_bfs_predicate =
        cca_square_simulator.register_function_event(dynamic_bfs_predicate_func);
    dynamic_bfs_work = cca_square_simulator.register_function_event(dynamic_bfs_work_func);
    dynamic_bfs_diffuse_predicate =
        cca_square_simulator.register_function_event(dynamic_bfs_diffuse_predicate_func);
    dynamic_bfs_diffuse = cca_square_simulator.register_function_event(dynamic_bfs_diffuse_func);

    dynamic_bfs_insert_edge_predicate =
        cca_square_simulator.register_function_event(dynamic_bfs_insert_edge_predicate_func);
    dynamic_bfs_insert_edge_work =
        cca_square_simulator.register_function_event(dynamic_bfs_insert_edge_work_func);
    dynamic_bfs_insert_edge_diffuse_predicate = cca_square_simulator.register_function_event(
        dynamic_bfs_insert_edge_diffuse_predicate_func);
    dynamic_bfs_insert_edge_diffuse =
        cca_square_simulator.register_function_event(dynamic_bfs_insert_edge_diffuse_func);

    allocate = cca_square_simulator.register_function_event(allocate_bfs_func);
    dynamic_bfs_insert_edge_continuation_ghost_allocate_return =
        cca_square_simulator.register_function_event(
            dynamic_bfs_insert_edge_continuation_ghost_allocate_return_func);

    std::optional<Address> dynamic_bfs_terminator = cca_square_simulator.create_terminator();
    if (!dynamic_bfs_terminator) {
        std::cerr << "Error! Memory not allocated for dynamic_bfs_terminator \n";
        exit(0);
    }

    for (u_int32_t dynamic_increment = 1; dynamic_increment <= cmd_args.increments;
         dynamic_increment++) {

        // Read edges from the increament file and then insert then and germinate actions.
        std::string input_graph_inc_path =
            cmd_args.input_graph_path + "_" + std::to_string(dynamic_increment) + ".tsv";
        std::vector<EdgeTuple> new_edges =
            input_graph.read_dnyamic_graph_increment<true>(input_graph_inc_path);
        std::cout << "\n\nRead " << new_edges.size() << " edges from " << input_graph_inc_path
                  << "\n";

        // Transfer the edges to IO channels. The function will create an action reponsible to
        // edge insertion that will contain the edge.
        input_graph.transfer_graph_edges_increment_host_to_io_channel<
            BFSVertex<RecursiveParallelVertex<Address, edges_min>>>(
            cca_square_simulator,
            new_edges,
            dynamic_bfs_terminator.value(),
            dynamic_bfs_insert_edge_predicate,
            dynamic_bfs_insert_edge_work,
            dynamic_bfs_insert_edge_diffuse_predicate,
            dynamic_bfs_insert_edge_diffuse);

        std::cout << "Transfered to the IO Channels\n";

        ///////////

        /* if (dynamic_increment == 1) {
            // Set the root to 0 by hand.

            const auto vertex_addr = input_graph.vertex_addresses[cmd_args.root_vertex];
            auto* vertex = static_cast<BFSVertex<ghost_type_level_1>*>(
                cca_square_simulator.get_object(vertex_addr));
            vertex->bfs_level = 0;
        } */

        ////////////

        std::cout << "\nStarting Execution on the CCA Chip:\n\n";
        auto start = std::chrono::steady_clock::now();
        cca_square_simulator.run_simulation(dynamic_bfs_terminator.value());
        auto end = std::chrono::steady_clock::now();

        std::cout << "Increment Cycles: " << cca_square_simulator.total_current_run_cycles
                  << ", Total Cycles: " << cca_square_simulator.total_cycles << "\n";

        std::cout << "Program elapsed time (This has nothing to do with the simulation "
                     "itself): "
                  << std::chrono::duration_cast<std::chrono::seconds>(end - start).count() << " s"
                  << std::endl;

        if (dynamic_increment == 2) {

            // Only put the BFS seed action on a single vertex.
            // In this case BFS root = root_vertex
            auto vertex_addr = input_graph.get_vertex_address_in_cca(cmd_args.root_vertex);

            BFSArguments root_level_to_send;
            root_level_to_send.level = 0;
            // Origin vertex from where this action came. Host not used. Put any value;
            root_level_to_send.src_vertex_id = 99999;

            ActionArgumentType const args_x =
                cca_create_action_argument<BFSArguments>(root_level_to_send);

            // Insert a seed action into the CCA chip that will help start the diffusion.
            cca_square_simulator.germinate_action(Action(vertex_addr,
                                                         dynamic_bfs_terminator.value(),
                                                         actionType::germinate_action,
                                                         true,
                                                         args_x,
                                                         dynamic_bfs_predicate,
                                                         dynamic_bfs_work,
                                                         dynamic_bfs_diffuse_predicate,
                                                         dynamic_bfs_diffuse));

            cca_square_simulator.run_simulation(dynamic_bfs_terminator.value());
            // Verify results.
            if (cmd_args.verify_results) {
                verify_results<BFSVertex<SimpleVertex<host_edge_type, edges_min>>>(
                    cmd_args, input_graph, cca_square_simulator, dynamic_increment);
            }
        }
    }

    // input_graph.validate_vertices_sent_to_cca<BFSVertex<ghost_type_level_1>>(cca_square_simulator);

    write_results<BFSVertex<SimpleVertex<host_edge_type, edges_min>>>(
        cmd_args, input_graph, cca_square_simulator);

    return 0;
}

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

#include "cca_bfs.hpp"

// Datastructures
#include "CyclicMemoryAllocator.hpp"
#include "Graph.hpp"

#include <chrono>
#include <fstream>

// Declare the function event ids for the BFS action functions of predicate, work, and diffuse.
// In the main register the functions and get their ids
CCAFunctionEvent bfs_predicate;
CCAFunctionEvent bfs_work;
CCAFunctionEvent bfs_diffuse;

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
    Graph<BFSVertex<SimpleVertex<host_edge_type>>> input_graph(cmd_args.input_graph_path);

    std::cout << "Allocating vertices cyclically on the CCA Chip: \n";

    // Memory allocator for vertices allocation. Here we use cyclic allocator, which allocates
    // vertices (or objects) one per compute cell in round-robin fashion. This is different from
    // when the `RecursiveParallelVertex` allocates ghost vertices.
    // To avoid high degree vertex being allocated on the corners of the chip we start the cyclic
    // allocator from the center of the chip and later provide the `root` vertex to the graph
    // initializer in `transfer_graph_host_to_cca`.
    u_int32_t center_of_the_chip = (cca_square_simulator.dim_x * (cca_square_simulator.dim_y / 2)) +
                                   (cca_square_simulator.dim_y / 2);
    // center_of_the_chip = 0;
    CyclicMemoryAllocator allocator(center_of_the_chip, cca_square_simulator.total_compute_cells);

    // Note: here we use BFSSimpleVertex<Address> since the vertex object is now going to be sent to
    // the CCA chip and there the address type is Address (not u_int32_t ID).
    input_graph.transfer_graph_host_to_cca<BFSVertex<RecursiveParallelVertex<Address>>>(
        cca_square_simulator,
        allocator,
        std::optional<u_int32_t>(cmd_args.root_vertex),
        cmd_args.shuffle_switch);

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

    // Only put the BFS seed action on a single vertex.
    // In this case BFS root = root_vertex
    auto vertex_addr = input_graph.get_vertex_address_in_cca(cmd_args.root_vertex);

    // Register the BFS action functions for predicate, work, and diffuse.
    bfs_predicate = cca_square_simulator.register_function_event(bfs_predicate_func);
    bfs_work = cca_square_simulator.register_function_event(bfs_work_func);
    bfs_diffuse = cca_square_simulator.register_function_event(bfs_diffuse_func);

    BFSArguments root_level_to_send;
    root_level_to_send.level = 0;
    // Origin vertex from where this action came. Host not used. Put any value;
    root_level_to_send.src_vertex_id = 99999;

    ActionArgumentType const args_x = cca_create_action_argument<BFSArguments>(root_level_to_send);

    std::optional<Address> bfs_terminator = cca_square_simulator.create_terminator();
    if (!bfs_terminator) {
        std::cerr << "Error! Memory not allocated for bfs_terminator \n";
        exit(0);
    }

    // Insert a seed action into the CCA chip that will help start the diffusion.
    cca_square_simulator.germinate_action(Action(vertex_addr,
                                                 bfs_terminator.value(),
                                                 actionType::germinate_action,
                                                 true,
                                                 args_x,
                                                 bfs_predicate,
                                                 bfs_work,
                                                 bfs_diffuse));

    ///////////

    /* auto vertex_addr_to_dst = input_graph.get_vertex_address_in_cca(0);

    BFSArguments root_level_to_send_dst;
    root_level_to_send_dst.level = 0;
    // Origin vertex from where this action came. Host not used. Put any value;
    root_level_to_send_dst.src_vertex_id = 99999;

    ActionArgumentType const args_x_dst =
        cca_create_action_argument<BFSArguments>(root_level_to_send_dst);

    // Insert a seed action into the CCA chip that will help start the diffusion.
    cca_square_simulator.germinate_action(Action(vertex_addr_to_dst,
                                                 bfs_terminator.value(),
                                                 actionType::germinate_action,
                                                 true,
                                                 args_x_dst,
                                                 bfs_predicate,
                                                 bfs_work,
                                                 bfs_diffuse)); */

    ////////////

    std::cout << "\nStarting Execution on the CCA Chip:\n\n";
    auto start = std::chrono::steady_clock::now();
    cca_square_simulator.run_simulation(bfs_terminator.value());
    auto end = std::chrono::steady_clock::now();

    std::cout << "Total Cycles: " << cca_square_simulator.total_cycles << "\n";

    std::cout << "Program elapsed time (This has nothing to do with the simulation "
                 "itself): "
              << std::chrono::duration_cast<std::chrono::seconds>(end - start).count() << " s"
              << std::endl;

    // Verify results.
    if (cmd_args.verify_results) {
        verify_results<BFSVertex<SimpleVertex<host_edge_type>>>(
            cmd_args, input_graph, cca_square_simulator);
    }

    write_results<BFSVertex<SimpleVertex<host_edge_type>>>(
        cmd_args, input_graph, cca_square_simulator);

    return 0;
}

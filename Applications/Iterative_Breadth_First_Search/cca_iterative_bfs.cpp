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

#include "cca_iterative_bfs.hpp"

// Datastructures
#include "CyclicMemoryAllocator.hpp"

#include <chrono>

// Declare the function event ids for the BFS action functions of predicate, work, and diffuse.
// In the main register the functions and get their ids
CCAFunctionEvent bfs_iterative_predicate;
CCAFunctionEvent bfs_iterative_work;
CCAFunctionEvent bfs_iterative_diffuse;

auto
main(int argc, char** argv) -> int
{
    // Parse the commandline input
    cli::Parser parser(argc, argv);
    configure_parser(parser);
    parser.run_and_exit_if_error();

    std::cout << "Parsing Commandline Arguments: \n";
    BFSIterativeCommandLineArguments cmd_args(parser);

    std::cout << "Creating the simulation environment that includes the CCA Chip: \n";

    // Create the simulation environment
    CCASimulator cca_square_simulator(cmd_args.shape_of_compute_cells,
                                      cmd_args.hx,
                                      cmd_args.hy,
                                      cmd_args.hdepth,
                                      cmd_args.hbandwidth_max,
                                      cmd_args.memory_per_cc,
                                      cmd_args.routing_policy);

    // Print details of the CCA Chip.
    cca_square_simulator.print_discription(std::cout);

    // Read the input data graph.
    Graph<BFSIterativeVertex<SimpleVertex<host_edge_type>>> input_graph(cmd_args.input_graph_path);

    std::cout << "Allocating vertices cyclically on the CCA Chip: \n";

    // Memory allocator for vertices allocation. Here we use cyclic allocator, which allocates
    // vertices (or objects) one per compute cell in round-robin fashion. This is different from
    // when the `RecursiveParallelVertex` allocates ghost vertices.
    CyclicMemoryAllocator allocator;

    // Note: here we use BFSSimpleVertex<Address> since the vertex object is now going to be sent
    // to the CCA chip and there the address type is Address (not u_int32_t ID).
    input_graph.transfer_graph_host_to_cca<BFSIterativeVertex<RecursiveParallelVertex<Address>>>(
        cca_square_simulator, allocator);

    // Only put the BFS seed action on a single vertex.
    // In this case BFS root = root_vertex
    auto vertex_addr = input_graph.get_vertex_address_in_cca(cmd_args.root_vertex);

    // Register the BFS action functions for predicate, work, and diffuse.
    bfs_iterative_predicate =
        cca_square_simulator.register_function_event(bfs_iterative_predicate_func);
    bfs_iterative_work = cca_square_simulator.register_function_event(bfs_iterative_work_func);
    bfs_iterative_diffuse =
        cca_square_simulator.register_function_event(bfs_iterative_diffuse_func);

    std::optional<Address> bfs_iterative_terminator = cca_square_simulator.create_terminator();
    if (!bfs_iterative_terminator) {
        std::cerr << "Error! Memory not allocated for bfs_iterative_terminator \n";
        exit(0);
    }

    // Create the iteration space by eponentially increasing depth.
    std::vector<u_int32_t> iteration_deeepening_space;
    for (u_int32_t i = 1; i <= cmd_args.bfs_iterative_deepening_max; i *= 2) {
        iteration_deeepening_space.push_back(i);
    }
    // Check if bfs_iterative_deepening_max is not already in the sequence. If not then add it.
    if (iteration_deeepening_space.back() != cmd_args.bfs_iterative_deepening_max) {
        iteration_deeepening_space.push_back(cmd_args.bfs_iterative_deepening_max);
    }

    u_int32_t total_program_cycles = 0;
    auto start = std::chrono::steady_clock::now();
    for (const auto& iterations : iteration_deeepening_space) {

        BFSIterativeArguments root_distance_to_send;
        root_distance_to_send.level = 0;
        root_distance_to_send.src_vertex_id = 99999; // host not used. Put any value;
        root_distance_to_send.depth_max = iterations;
        root_distance_to_send.depth_current = 0;
        root_distance_to_send.src_vertex_addr = Address(0, 0, adressType::host_address);

        ActionArgumentType const args_x =
            cca_create_action_argument<BFSIterativeArguments>(root_distance_to_send);

        // Insert a seed action into the CCA chip that will help start the diffusion.
        cca_square_simulator.germinate_action(Action(vertex_addr,
                                                     bfs_iterative_terminator.value(),
                                                     actionType::germinate_action,
                                                     true,
                                                     args_x,
                                                     bfs_iterative_predicate,
                                                     bfs_iterative_work,
                                                     bfs_iterative_diffuse));

        std::cout << "\nIteration: " << iterations << ", Starting Execution on the CCA Chip\n\n";

        cca_square_simulator.run_simulation(bfs_iterative_terminator.value());
        std::cout << "\nIteration: " << iterations
                  << ", Total Cycles: " << cca_square_simulator.total_current_run_cycles << "\n";
        total_program_cycles += cca_square_simulator.total_current_run_cycles;
        // Reset the terminator for the next iteration.
        cca_square_simulator.reset_terminator(bfs_iterative_terminator.value());
    }

    auto end = std::chrono::steady_clock::now();
    std::cout << "Program elapsed time in milliseconds (This has nothing to do with the simulation "
                 "itself): "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms"
              << std::endl;

    std::cout << "\nTotal Iterations: " << iteration_deeepening_space.size()
              << ", Total Program Cycles: " << total_program_cycles << "\n";

    // Verify results.
    if (cmd_args.verify_results) {
        verify_results<BFSIterativeVertex<SimpleVertex<host_edge_type>>>(
            cmd_args, input_graph, cca_square_simulator);
    }

    write_results<BFSIterativeVertex<SimpleVertex<host_edge_type>>>(
        cmd_args, input_graph, cca_square_simulator);

    return 0;
}

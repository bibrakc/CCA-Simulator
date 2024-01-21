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

#include "cca_sssp.hpp"

// Datastructures
#include "CyclicMemoryAllocator.hpp"

#include <chrono>

// Declare the function event ids for the SSSP action functions of predicate, work, and diffuse.
// In the main register the functions and get their ids
CCAFunctionEvent sssp_predicate;
CCAFunctionEvent sssp_work;
CCAFunctionEvent sssp_diffuse_predicate;
CCAFunctionEvent sssp_diffuse;

auto
main(int argc, char** argv) -> int
{
    // Parse the commandline input
    cli::Parser parser(argc, argv);
    configure_parser(parser);
    parser.run_and_exit_if_error();

    std::cout << "Parsing Commandline Arguments: \n";
    SSSPCommandLineArguments cmd_args(parser);

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
    Graph<SSSPVertex<SimpleVertex<host_edge_type>>> input_graph(cmd_args.input_graph_path);

    std::cout << "Allocating vertices cyclically on the CCA Chip: \n";

    // Memory allocator for vertices allocation. Here we use cyclic allocator, which allocates
    // vertices (or objects) one per compute cell in round-robin fashion. This is different from
    // when the `RecursiveParallelVertex` allocates ghost vertices.
    // To avoid high degree vertex being allocated on the corners of the chip we start the cyclic
    // allocator from the center of the chip and later provide the `root` vertex to the graph
    // initializer in `transfer_graph_host_to_cca`.
    u_int32_t center_of_the_chip = (cca_square_simulator.dim_x * (cca_square_simulator.dim_y / 2)) +
                                   (cca_square_simulator.dim_y / 2);
    CyclicMemoryAllocator allocator(center_of_the_chip, cca_square_simulator.total_compute_cells);

    // Note: here we use SSSPSimpleVertex<Address> since the vertex object is now going to be sent
    // to the CCA chip and there the address type is Address (not u_int32_t ID).
    input_graph.transfer_graph_host_to_cca<SSSPVertex<RecursiveParallelVertex<Address>>>(
        cca_square_simulator,
        allocator,
        std::optional<u_int32_t>(cmd_args.root_vertex),
        cmd_args.shuffle_switch);

    // Only put the SSSP seed action on a single vertex.
    // In this case SSSP root = root_vertex
    auto vertex_addr = input_graph.get_vertex_address_in_cca(cmd_args.root_vertex);

    // Register the SSSP action functions for predicate, work, and diffuse.
    sssp_predicate = cca_square_simulator.register_function_event(sssp_predicate_func);
    sssp_work = cca_square_simulator.register_function_event(sssp_work_func);
    sssp_diffuse_predicate =
        cca_square_simulator.register_function_event(sssp_diffuse_predicate_func);
    sssp_diffuse = cca_square_simulator.register_function_event(sssp_diffuse_func);

    SSSPArguments root_distance_to_send;
    root_distance_to_send.distance = 0;
    root_distance_to_send.src_vertex_id = 99999; // host not used. Put any value;

    ActionArgumentType const args_x =
        cca_create_action_argument<SSSPArguments>(root_distance_to_send);

    std::optional<Address> sssp_terminator = cca_square_simulator.create_terminator();
    if (!sssp_terminator) {
        std::cerr << "Error! Memory not allocated for sssp_terminator \n";
        exit(0);
    }

    // Insert a seed action into the CCA chip that will help start the diffusion.
    cca_square_simulator.germinate_action(Action(vertex_addr,
                                                 sssp_terminator.value(),
                                                 actionType::germinate_action,
                                                 true,
                                                 args_x,
                                                 sssp_predicate,
                                                 sssp_work,
                                                 sssp_diffuse_predicate,
                                                 sssp_diffuse));

    std::cout << "\nStarting Execution on the CCA Chip:\n\n";
    auto start = std::chrono::steady_clock::now();
    cca_square_simulator.run_simulation(sssp_terminator.value());
    auto end = std::chrono::steady_clock::now();

    std::cout << "Total Cycles: " << cca_square_simulator.total_cycles << "\n";

    std::cout << "Program elapsed time in milliseconds (This has nothing to do with the simulation "
                 "itself): "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms"
              << std::endl;

    // Verify results.
    if (cmd_args.verify_results) {
        verify_results<SSSPVertex<SimpleVertex<host_edge_type>>>(
            cmd_args, input_graph, cca_square_simulator);
    }

    write_results<SSSPVertex<SimpleVertex<host_edge_type>>>(
        cmd_args, input_graph, cca_square_simulator);

    return 0;
}

/*
BSD 3-Clause License

Copyright (c) 2023-2024, Bibrak Qamar

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

#include "cca_page_rank_fixed_iterations_rhizome.hpp"

// Datastructures
#include "CyclicMemoryAllocator.hpp"
// #include "Graph.hpp"

#include <chrono>
// #include <fstream>

// Declare the function event ids for the Page Rank Fixed Iterations action functions of predicate,
// work, and diffuse. In the main register the functions and get their ids
CCAFunctionEvent page_rank_fixed_iterations_predicate;
CCAFunctionEvent page_rank_fixed_iterations_germinate_work;
CCAFunctionEvent page_rank_fixed_iterations_work;
CCAFunctionEvent page_rank_fixed_iterations_diffuse_predicate;
CCAFunctionEvent page_rank_fixed_iterations_diffuse;

CCAFunctionEvent page_rank_fixed_iterations_rhizome_collapse;
CCAFunctionEvent page_rank_fixed_iterations_rhizome_collapse_diffuse;

#define PRINT_RESULTS_FOR_ALL_VERTICES false

auto
main(int argc, char** argv) -> int
{
    // Parse the commandline input.
    cli::Parser parser(argc, argv);
    configure_parser(parser);
    parser.run_and_exit_if_error();

    std::cout << "Parsing Commandline Arguments: \n";
    PageRankFixedIterationsCommandLineArguments cmd_args(parser);

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
    Graph<PageRankFixedIterationsVertex<SimpleVertex<host_edge_type, edges_min>>> input_graph(
        cmd_args.input_graph_path, false);

    // Get the vertices with degree values equal to 0.
    std::vector<u_int32_t> vertices_inbound_degree_zero =
        input_graph.get_vertices_ids_with_zero_in_degree();

    std::cout << "Allocating vertices cyclically on the CCA Chip: \n";

    // Memory allocator for vertices allocation. Here we use cyclic allocator, which allocates
    // vertices (or objects) one per compute cell in round-robin fashion. This is different from
    // when the `RecursiveParallelVertex` allocates ghost vertices.
    // To avoid high degree vertex being allocated on the corners of the chip we can start the
    // cyclic allocator from the center of the chip and later provide the `root` vertex to the graph
    // initializer in `transfer_graph_host_to_cca`. If the root is a non high-degree node then the
    // high-degree node won't be at the center. In most experiments it is not and therefore we
    // really dont do any such "preprocessing".
    u_int32_t center_of_the_chip = (cca_square_simulator.dim_x * (cca_square_simulator.dim_y / 2)) +
                                   (cca_square_simulator.dim_y / 2);
    // center_of_the_chip = 0;
    CyclicMemoryAllocator allocator(center_of_the_chip, cca_square_simulator.total_compute_cells);

    // random allocator for creating rhizomes. There will be placed anywhere on the chip randomly.
    // The hope is to have them be somewhat apart.
    VicinityMemoryAllocator random_allocator(
        Coordinates(cca_square_simulator.dim_x / 2, cca_square_simulator.dim_y / 2),
        cca_square_simulator.dim_x, // vicinity_rows
        cca_square_simulator.dim_y, // vicinity_cols
        cca_square_simulator.dim_x,
        cca_square_simulator.dim_y,
        cca_square_simulator.shape_of_compute_cells);

    // Note: here we use PageRankFixedIterationsSimpleVertex<Address> since the vertex object is now
    // going to be sent to the CCA chip and there the address type is Address (not u_int32_t ID).
    input_graph
        .transfer_graph_host_to_cca_rhizome<PageRankFixedIterationsVertex<ghost_type_level_1>>(
            cca_square_simulator,
            allocator,
            random_allocator,
            std::optional<u_int32_t>(cmd_args.root_vertex),
            cmd_args.shuffle_switch);

    // Only put the PageRankFixedIterationsAction seed action on a single vertex.
    // In this case Page Rank Fixed Iterations root = root_vertex
    auto vertex_addr = input_graph.get_vertex_address_in_cca_rhizome(cmd_args.root_vertex);

    // Register the Page Rank Fixed Iterations action functions for predicate, work, and diffuse.
    page_rank_fixed_iterations_predicate =
        cca_square_simulator.register_function_event(page_rank_fixed_iterations_predicate_func);
    page_rank_fixed_iterations_germinate_work = cca_square_simulator.register_function_event(
        page_rank_fixed_iterations_germinate_work_func);
    page_rank_fixed_iterations_work =
        cca_square_simulator.register_function_event(page_rank_fixed_iterations_work_func);
    page_rank_fixed_iterations_diffuse_predicate = cca_square_simulator.register_function_event(
        page_rank_fixed_iterations_diffuse_predicate_func);
    page_rank_fixed_iterations_diffuse =
        cca_square_simulator.register_function_event(page_rank_fixed_iterations_diffuse_func);

    page_rank_fixed_iterations_rhizome_collapse = cca_square_simulator.register_function_event(
        page_rank_fixed_iterations_rhizome_collapse_func);

    page_rank_fixed_iterations_rhizome_collapse_diffuse =
        cca_square_simulator.register_function_event(
            page_rank_fixed_iterations_rhizome_collapse_diffuse_func);

    // Create the terminator for this app.
    std::optional<Address> page_rank_fixed_iterations_terminator =
        cca_square_simulator.create_terminator();
    if (!page_rank_fixed_iterations_terminator) {
        std::cerr << "Error! Memory not allocated for page_rank_fixed_iterations_terminator \n";
        exit(0);
    }

    u_int32_t total_program_cycles = 0;
    auto start = std::chrono::steady_clock::now();
    for (u_int32_t iterations = 0; iterations < cmd_args.iter; iterations++) {

        {
            std::cout << "Germinating " << vertices_inbound_degree_zero.size()
                      << " vertices who have indegree of 0" << std::endl;
            /* std::cout << "[";
            // Print the contents of vertices_inbound_degree_zero
            for (size_t i = 0; i < vertices_inbound_degree_zero.size(); ++i) {
                std::cout << vertices_inbound_degree_zero[i];
                if (i != vertices_inbound_degree_zero.size() - 1) {
                    std::cout << ", ";
                }
            }
            std::cout << "]" << std::endl; */
        }

        // Prepare the arguments (payload) for the actions.
        PageRankFixedIterationsArguments root_score_to_send;
        root_score_to_send.score = -10;
        root_score_to_send.src_vertex_id = 999999;
        root_score_to_send.iteration = iterations;

        ActionArgumentType const args_x =
            cca_create_action_argument<PageRankFixedIterationsArguments>(root_score_to_send);

        // Insert a seed action into the CCA chip that will help start the diffusion.
        cca_square_simulator.germinate_action(Action(vertex_addr,
                                                     page_rank_fixed_iterations_terminator.value(),
                                                     actionType::germinate_action,
                                                     true,
                                                     args_x,
                                                     page_rank_fixed_iterations_predicate,
                                                     page_rank_fixed_iterations_germinate_work,
                                                     page_rank_fixed_iterations_diffuse_predicate,
                                                     page_rank_fixed_iterations_diffuse));

        // Germinate seed action on the vertices with inbound_degree zero.
        // This is needed since otherwise they will never be activated and therefore in turn cannot
        // send their score to other vertices that will be waiting on them.
        for (const auto vertex_id : vertices_inbound_degree_zero) {

            auto vertex_addr_to_zero_in_degree =
                input_graph.get_vertex_address_in_cca_rhizome(vertex_id);

            // Prepare the arguments (payload) for the actions.
            PageRankFixedIterationsArguments germinate_arg_to_zero_in_degree;
            germinate_arg_to_zero_in_degree.score = -10;
            germinate_arg_to_zero_in_degree.src_vertex_id = 99999;
            germinate_arg_to_zero_in_degree.iteration = iterations;

            ActionArgumentType const args_to_zero_in_degree =
                cca_create_action_argument<PageRankFixedIterationsArguments>(
                    germinate_arg_to_zero_in_degree);

            // Insert a seed action into the CCA chip that will help start the diffusion.
            cca_square_simulator.germinate_action(
                Action(vertex_addr_to_zero_in_degree,
                       page_rank_fixed_iterations_terminator.value(),
                       actionType::germinate_action,
                       true,
                       args_to_zero_in_degree,
                       page_rank_fixed_iterations_predicate,
                       page_rank_fixed_iterations_germinate_work,
                       page_rank_fixed_iterations_diffuse_predicate,
                       page_rank_fixed_iterations_diffuse));

            // std::cout << "Germinated Vertices with degree value 0: " << vertex_id << "\n";
        }

        std::cout << "\nIteration: " << iterations << ", Starting Execution on the CCA Chip\n\n";

        cca_square_simulator.run_simulation(page_rank_fixed_iterations_terminator.value());

        std::cout << "\nIteration: " << iterations
                  << ", Total Cycles: " << cca_square_simulator.total_current_run_cycles << "\n";
        total_program_cycles += cca_square_simulator.total_current_run_cycles;
        // Reset the terminator for the next iteration.
        cca_square_simulator.reset_terminator(page_rank_fixed_iterations_terminator.value());
    }

    auto end = std::chrono::steady_clock::now();
    std::cout << "Program elapsed time in milliseconds (This has nothing to do with the simulation "
                 "itself): "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms"
              << std::endl;

    std::cout << "\nTotal Iterations: " << cmd_args.iter
              << ", Total Program Cycles: " << total_program_cycles << "\n";

    if (cmd_args.verify_results) {
        verify_results<PageRankFixedIterationsVertex<SimpleVertex<host_edge_type, edges_min>>>(
            cmd_args, input_graph, cca_square_simulator);
    }

    write_results<PageRankFixedIterationsVertex<SimpleVertex<host_edge_type, edges_min>>>(
        cmd_args, input_graph, cca_square_simulator);

    return 0;
}

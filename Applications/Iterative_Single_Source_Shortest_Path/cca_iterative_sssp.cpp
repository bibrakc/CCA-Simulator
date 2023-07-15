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

#include "cca_iterative_sssp.hpp"

// Datastructures
#include "CyclicMemoryAllocator.hpp"
#include "Graph.hpp"

#include <chrono>
#include <fstream>
#include <omp.h>

// Declare the function event ids for the SSSP action functions of predicate, work, and diffuse.
// In the main register the functions and get their ids
CCAFunctionEvent sssp_iterative_predicate;
CCAFunctionEvent sssp_iterative_work;
CCAFunctionEvent sssp_iterative_diffuse;

auto
main(int argc, char** argv) -> int
{
    // Parse the commandline input
    cli::Parser parser(argc, argv);
    configure_parser(parser);
    parser.run_and_exit_if_error();

    // Total max depth iterations to perform.
    auto sssp_iterative_deepening_max = parser.get<u_int32_t>("iter");

    // SSSP root vertex
    auto root_vertex = parser.get<u_int32_t>("root");

    // Cross check the calculated sssp lengths from root with the lengths provided in .sssp file
    auto verify_results = parser.get<bool>("verify");

    // Configuration related to the input data graph
    auto input_graph_path = parser.get<std::string>("f");
    auto graph_name = parser.get<std::string>("g");

    // Optional output directory path
    auto output_file_directory = parser.get<std::string>("od");

    // Get the depth of Htree
    auto hdepth = parser.get<u_int32_t>("hdepth");

    // Get the rows and columbs of cells that are served by a single end Htree node. This will help
    // in construction of the CCA chip, Htree, and routing
    auto hx = parser.get<u_int32_t>("hx");
    auto hy = parser.get<u_int32_t>("hy");
    if (hdepth != 0) {
        if (!(hx % 2)) {
            std::cerr << "Invalid Input: hx must be odd! Provided value: " << hx << "\n";
            exit(0);
        }
        if (!(hy % 2)) {
            std::cerr << "Invalid Input: hy must be odd! Provided value: " << hy << "\n";
            exit(0);
        }
    }

    // Get the max bandwidth of Htree
    auto hbandwidth_max = parser.get<u_int32_t>("hb");
    if (hdepth == 0) {
        hbandwidth_max = 0;
    }
    // Configuration related to the CCA Chip
    auto shape_arg = parser.get<std::string>("s");
    computeCellShape shape_of_compute_cells = computeCellShape::computeCellShape_invalid;

    if (shape_arg == "square") {
        shape_of_compute_cells = computeCellShape::square;
    } else {
        std::cerr << "Error: Compute cell shape type " << shape_arg << " not supported.\n";
        exit(0);
    }

    // Get the memory per cc or use the default
    auto memory_per_cc = parser.get<u_int32_t>("m");

    // Get the routing policy to use
    auto routing_policy = parser.get<u_int32_t>("route");

    std::cout << "Creating the simulation environment that includes the CCA Chip: \n";

    // Create the simulation environment
    CCASimulator cca_square_simulator(
        shape_of_compute_cells, hx, hy, hdepth, hbandwidth_max, memory_per_cc, routing_policy);

    // Print details of the CCA Chip.
    cca_square_simulator.print_discription(std::cout);

    // Read the input data graph.
    Graph<SSSPVertex<SimpleVertex<host_edge_type>>> input_graph(input_graph_path);

    std::cout << "Allocating vertices cyclically on the CCA Chip: \n";

    // Memory allocator for vertices allocation. Here we use cyclic allocator, which allocates
    // vertices (or objects) one per compute cell in round-robin fashion.
    std::unique_ptr<MemoryAllocator> allocator = std::make_unique<CyclicMemoryAllocator>();

    // Note: here we use SSSPSimpleVertex<Address> since the vertex object is now going to be sent
    // to the CCA chip and there the address type is Address (not u_int32_t ID).
    input_graph.transfer_graph_host_to_cca<SSSPVertex<RecursiveParallelVertex<Address>>>(
        cca_square_simulator, allocator);

    // Only put the SSSP seed action on a single vertex.
    // In this case SSSP root = root_vertex
    auto vertex_addr = input_graph.get_vertex_address_in_cca(root_vertex);

    // Register the SSSP action functions for predicate, work, and diffuse.
    sssp_iterative_predicate =
        cca_square_simulator.register_function_event(sssp_iterative_predicate_func);
    sssp_iterative_work = cca_square_simulator.register_function_event(sssp_iterative_work_func);
    sssp_iterative_diffuse =
        cca_square_simulator.register_function_event(sssp_iterative_diffuse_func);

    SSSPIterativeArguments root_distance_to_send;
    root_distance_to_send.distance = 0;
    root_distance_to_send.src_vertex_id = 99999; // host not used. Put any value;
    root_distance_to_send.depth_max = sssp_iterative_deepening_max;
    root_distance_to_send.depth_current = 0;
    root_distance_to_send.src_vertex_addr = Address(0, 0, adressType::host_address);

    ActionArgumentType const args_x =
        cca_create_action_argument<SSSPIterativeArguments>(root_distance_to_send);

    std::optional<Address> sssp_iterative_terminator = cca_square_simulator.create_terminator();
    if (!sssp_iterative_terminator) {
        std::cerr << "Error! Memory not allocated for sssp_iterative_terminator \n";
        exit(0);
    }

    u_int32_t total_program_cycles = 0;
    auto start = std::chrono::steady_clock::now();
    for (u_int32_t iterations = 0; iterations < sssp_iterative_deepening_max; iterations++) {

        // Insert a seed action into the CCA chip that will help start the diffusion.
        cca_square_simulator.germinate_action(Action(vertex_addr,
                                                     sssp_iterative_terminator.value(),
                                                     actionType::germinate_action,
                                                     true,
                                                     args_x,
                                                     sssp_iterative_predicate,
                                                     sssp_iterative_work,
                                                     sssp_iterative_diffuse));

        std::cout << "\nIteration: " << iterations << ", Starting Execution on the CCA Chip\n\n";

        cca_square_simulator.run_simulation(sssp_iterative_terminator.value());
        std::cout << "\nIteration: " << iterations
                  << ", Total Cycles: " << cca_square_simulator.total_current_run_cycles << "\n";
        total_program_cycles += cca_square_simulator.total_current_run_cycles;
        // Reset the terminator for the next iteration.
        cca_square_simulator.reset_terminator(sssp_iterative_terminator.value());
    }

    auto end = std::chrono::steady_clock::now();
    std::cout << "Program elapsed time in milliseconds (This has nothing to do with the simulation "
                 "itself): "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms"
              << std::endl;

    std::cout << "\nTotal Iterations: " << sssp_iterative_deepening_max
              << ", Total Program Cycles: " << total_program_cycles << "\n";

    if (verify_results) {
        std::cout << "\nSingle Source Shortest Path Verification: \n";

        // Open the file containing sssp results for verification.
        std::string verfication_file_path = input_graph_path + ".sssp";
        std::ifstream file(verfication_file_path);

        if (!file.is_open()) {
            std::cout << "Failed to open the verification file: " << verfication_file_path << "\n";

        } else {

            std::vector<u_int32_t> control_results;
            std::string line;
            // Read the header.
            std::getline(file, line);
            // Read the root (source) of sssp that was used for results in the .sssp file.
            // Initialize it to an invalid value first.
            u_int32_t root_in_file = input_graph.total_vertices + 1;
            std::getline(file, line);
            if (!(std::istringstream(line) >> root_in_file)) {
                std::cerr << "Invalid root (source) value.\n";
                exit(0);
            }

            if (root_in_file != root_vertex) {
                std::cerr
                    << "root vertex in file and root vertex used to run the program miss match. "
                       "Please use the same root in both for verification. Failed!\n";
                exit(0);
            }

            u_int32_t node_id;
            u_int32_t sssp_iterative_value;
            while (std::getline(file, line)) {

                if (std::sscanf(line.c_str(), "%zu\t%zu", &node_id, &sssp_iterative_value) == 2) {
                    control_results.emplace_back(sssp_iterative_value);
                }
            }

            file.close();

            u_int32_t total_errors = 0;
            for (u_int32_t i = 0; i < control_results.size(); i++) {

                // Check for correctness. Print the distance to a target test vertex. test_vertex

                Address const test_vertex_addr = input_graph.get_vertex_address_in_cca(i);

                auto* v_test = static_cast<SSSPVertex<RecursiveParallelVertex<Address>>*>(
                    cca_square_simulator.get_object(test_vertex_addr));

                // Assumes the result .sssp file is sorted.
                bool equal = control_results[i] == v_test->sssp_distance;
                if (!equal) {
                    std::cout << "Vertex: " << i << ", Computed SSSP: " << v_test->sssp_distance
                              << ", Control Value: " << control_results[i]
                              << ", Not equal! Error\n";
                    total_errors++;
                }
            }

            if (total_errors > 0) {
                std::cout << "Total number values error: " << total_errors
                          << ", Verification Failed\n";
            } else {
                std::cout << "All values were correct. Verification Successful.\n";
            }
        }
    }

    // Write simulation statistics to a file
    std::string const output_file_name =
        "sssp_iterative_square_x_" + std::to_string(cca_square_simulator.dim_x) + "_y_" +
        std::to_string(cca_square_simulator.dim_y) + "_graph_" + graph_name + "_v_" +
        std::to_string(input_graph.total_vertices) + "_e_" +
        std::to_string(input_graph.total_edges) + "_hb_" + std::to_string(hbandwidth_max);

    std::string const output_file_path = output_file_directory + "/" + output_file_name;
    std::cout << "\nWriting results to output file: " << output_file_path << "\n";

    std::ofstream output_file(output_file_path);
    if (!output_file) {
        std::cerr << "Error! Output file not created\n";
    }

    // Output input graph details in the header of the statistics for us to know which input graph
    // it operated on.
    output_file << "graph_file\tvertices\tedges\troot_vertex\n"
                << input_graph_path << "\t" << input_graph.total_vertices << "\t"
                << input_graph.total_edges << "\t" << root_vertex << "\n";

    // Ask the simulator to print its statistics to the `output_file`.
    cca_square_simulator.print_statistics(output_file);

    // Close the output file
    output_file.close();

    // Write the active status animation data in a separate file.
    std::string const output_file_path_animation = output_file_path + "_active_animation";
    std::cout << "\nWriting active status animation data to output file: "
              << output_file_path_animation << "\n";

    std::ofstream output_file_animation(output_file_path_animation);
    if (!output_file_animation) {
        std::cerr << "Error! Output file not created\n";
    }

    // Ask the simulator to print cell active status information per cycle to the
    // `output_file_animation`. This will be used mostly for animation purposes.
    cca_square_simulator.output_CCA_active_status_per_cell_cycle(output_file_animation);

    // Close the output file
    output_file_animation.close();

    return 0;
}

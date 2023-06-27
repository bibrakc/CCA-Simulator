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
#include "Graph.hpp"
#include "SimpleVertex.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>

// std::ofstream
#include <fstream>
#include <map>
#include <queue>
#include <stdlib.h>

#include <omp.h>

// Declare the function event ids for the SSSP action functions of predicate, work, and diffuse.
// In the main register the functions and get their ids
CCAFunctionEvent sssp_predicate;
CCAFunctionEvent sssp_work;
CCAFunctionEvent sssp_diffuse;

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
    u_int32_t test_vertex = parser.get<u_int32_t>("tv");

    // Configuration related to the input data graph
    std::string input_graph_path = parser.get<std::string>("f");
    std::string graph_name = parser.get<std::string>("g");

    // Optional output directory path
    std::string output_file_directory = parser.get<std::string>("od");

    // Get the depth of Htree
    u_int32_t hdepth = parser.get<u_int32_t>("hdepth");

    // Get the rows and columbs of cells that are served by a single end Htree node. This will help
    // in construction of the CCA chip, Htree, and routing
    u_int32_t hx = parser.get<u_int32_t>("hx");
    u_int32_t hy = parser.get<u_int32_t>("hy");
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
    u_int32_t hbandwidth_max = parser.get<u_int32_t>("hb");
    if (hdepth == 0) {
        hbandwidth_max = 0;
    }
    // Configuration related to the CCA Chip
    std::string shape_arg = parser.get<std::string>("s");
    computeCellShape shape_of_compute_cells;

    if (shape_arg == "square") {
        shape_of_compute_cells = computeCellShape::square;
    } else {
        std::cerr << "Error: Compute cell shape type " << shape_arg << " not supported.\n";
        exit(0);
    }

    // Get the memory per cc or use the default
    u_int32_t memory_per_cc = parser.get<u_int32_t>("m");

    // Get the routing policy to use
    u_int32_t routing_policy = parser.get<u_int32_t>("route");

    std::cout << "Creating the simulation environment that includes the CCA Chip: \n";

    // Create the simulation environment
    CCASimulator cca_square_simulator(
        shape_of_compute_cells, hx, hy, hdepth, hbandwidth_max, memory_per_cc, routing_policy);

    // Print details of the CCA Chip.
    cca_square_simulator.print_discription(std::cout);

    // Read the input data graph.
    Graph<SimpleVertex<u_int32_t>> input_graph(input_graph_path);

    std::cout << "Allocating vertices cyclically on the CCA Chip: \n";

    // Memory allocator for vertices allocation. Here we use cyclic allocator, which allocates
    // vertices (or objects) one per compute cell in round-robin fashion.
    std::unique_ptr<MemoryAlloctor> allocator = std::make_unique<CyclicMemoryAllocator>();

    // Note: here we use SimpleVertex<Address> since the vertex object is now going to be sent to
    // the CCA chip and there the address type is Address (not u_int32_t ID).
    input_graph.transfer_graph_host_to_cca<SimpleVertex<Address>>(cca_square_simulator, allocator);

    // Only put the SSSP seed action on a single vertex.
    // In this case SSSP root = root_vertex
    auto vertex_addr = input_graph.get_vertex_address_in_cca(root_vertex);

    // Register the SSSP action functions for predicate, work, and diffuse.
    sssp_predicate = cca_square_simulator.register_function_event(sssp_predicate_func);
    sssp_work = cca_square_simulator.register_function_event(sssp_work_func);
    sssp_diffuse = cca_square_simulator.register_function_event(sssp_diffuse_func);

    // std::shared_ptr<int[]> args_x = std::make_shared<int[]>(2);
    std::shared_ptr<int[]> args_x(new int[2], std::default_delete<int[]>());
    // Set distance to 0
    args_x[0] = 0;
    // Origin vertex from where this action came
    args_x[1] = root_vertex;

    std::optional<Address> sssp_terminator = cca_square_simulator.create_terminator();
    if (!sssp_terminator) {
        std::cerr << "Error! Memory not allocated for sssp_terminator \n";
        exit(0);
    }

    // Insert a seed action into the CCA chip that will help start the diffusion.
    cca_square_simulator.germinate_action(SSSPAction(vertex_addr,
                                                     sssp_terminator.value(),
                                                     actionType::application_action,
                                                     true,
                                                     2,
                                                     args_x,
                                                     sssp_predicate,
                                                     sssp_work,
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

    // Check for correctness. Print the distance to a target test vertex.
    Address test_vertex_addr = input_graph.get_vertex_address_in_cca(test_vertex);

    SimpleVertex<Address>* v_test =
        static_cast<SimpleVertex<Address>*>(cca_square_simulator.get_object(test_vertex_addr));

    std::cout << "\nSSSP distance from vertex: " << root_vertex << " to vertex: " << v_test->id
              << " is: " << v_test->sssp_distance << "\n";

    // Write simulation statistics to a file
    std::string output_file_name = "square_x_" + std::to_string(cca_square_simulator.dim_x) +
                                   "_y_" + std::to_string(cca_square_simulator.dim_y) + "_graph_" +
                                   graph_name + "_v_" + std::to_string(input_graph.total_vertices) +
                                   "_e_" + std::to_string(input_graph.total_edges) + "_hb_" +
                                   std::to_string(hbandwidth_max);

    std::string output_file_path = output_file_directory + "/" + output_file_name;
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
    std::string output_file_path_animation = output_file_path + "_active_animation";
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

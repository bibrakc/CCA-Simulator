/*
BSD 3-Clause License

Copyright (c) 2024, Bibrak Qamar

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

#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

// Structure to represent an edge
struct Edge
{
    int target; // End vertex of the edge
    int weight; // Weight of the edge

    // Constructor
    Edge(int target, int weight)
        : target(target)
        , weight(weight)
    {
    }
};

int
main(int argc, char* argv[])
{

    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <input_file> <weighted>" << std::endl;
        return 1;
    }

    std::cout << "Remember the input file is assumed to be weighted. If it is not then the "
                 "converted output file will not be correct!\n";

    const std::string inputFileName = argv[1];
    const bool weighted = (std::stoi(argv[2]) != 0);
    std::string outputfile_extension = "Adj";
    if (weighted) {
        outputfile_extension = "_weightedAdj";
    }

    const std::string outputFileName = inputFileName + outputfile_extension;

    std::ifstream inputFile(inputFileName);
    if (!inputFile.is_open()) {
        std::cerr << "Failed to open input file: " << inputFileName << std::endl;
        return 1;
    }

    std::string line;
    bool dataStarted = false;

    std::vector<std::vector<Edge>> adj_graph;
    int num_vertices, num_verticesx, num_edges;

    {
        inputFile >> num_vertices >> num_verticesx;
        {
            std::cout << "num_vertices: " << num_vertices << ", num_verticesx: " << num_verticesx
                      << "\n";
            // Resize the adj_graph vector to the number of vertices.
            adj_graph.resize(num_vertices);
        }
        inputFile >> num_edges;
        {
            std::cout << "num_edges: " << num_edges << "\n";
        }
    }

    for (int i = 0; i < num_edges; ++i) {
        int vertex_from, vertex_to, weight;
        inputFile >> vertex_from >> vertex_to >> weight;
        adj_graph[vertex_from].emplace_back(Edge(vertex_to, weight));
    }

    inputFile.close();

    std::ofstream outputFile(outputFileName);
    if (!outputFile.is_open()) {
        std::cerr << "Failed to create output file: " << outputFileName << std::endl;
        inputFile.close();
        return 1;
    }

    std::cout << "Writing to the output file: " << outputFileName << std::endl;

    // Write number of vertices and the number of edges of the graph to the adj output file.
    if (weighted) {
        outputFile << "WeightedAdjacencyGraph\n";
    } else {
        outputFile << "AdjacencyGraph\n";
    }
    outputFile << adj_graph.size() << "\n" << num_edges << "\n";

    int offset = 0;
    // Write the first offset.
    outputFile << offset << "\n";

    for (int i = 0; i < adj_graph.size() - 1; i++) {
        offset += adj_graph[i].size();
        outputFile << offset << "\n";
    }

    for (const auto& vertex : adj_graph) {
        for (const auto& edge : vertex) {
            outputFile << edge.target << "\n";
        }
    }

    if (weighted) {
        for (const auto& vertex : adj_graph) {
            for (const auto& edge : vertex) {
                outputFile << edge.weight << "\n";
            }
        }
    }

    outputFile.close();

    std::cout << "Graph data processed and saved to " << outputFileName << std::endl;

    return 0;
}

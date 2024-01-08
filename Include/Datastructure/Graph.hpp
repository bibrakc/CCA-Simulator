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

#ifndef Graph_HPP
#define Graph_HPP

#include <random>

template<class VertexType>
class Graph
{
  public:
    // Checking to see whether the user has mistakenly used this for device side allocation. If they
    // donot provide VertexType `SimpleVertex` type then it means they are intenting to allocate on
    // the device. This Graph class is used as a utility to load and store the graph from file at
    // the host side. Then it is used to transfer the loaded graph to the device.
    // static_assert(std::is_same_v<VertexType, SimpleVertex<u_int32_t>>);

    u_int32_t total_vertices;
    u_int32_t total_edges;
    std::shared_ptr<VertexType[]> vertices;

    // Store the CCA address of vertices in a map so as to retrieve easily for edge insertion and
    // other tasks like printing for debuging and solution checking.
    std::map<u_int32_t, Address> vertex_addresses;

    auto get_vertex_address_in_cca(u_int32_t vertex_id) -> Address
    {
        return this->vertex_addresses[vertex_id];
    }

    void add_edge(VertexType& vertex, u_int32_t dst_vertex_id, u_int32_t weight)
    {
        // Add the edge on the host allocated graph.
        if (!vertex.insert_edge(dst_vertex_id, weight)) {
            std::cerr << "Error! add_edge() Edge (" << vertex.id << ", " << dst_vertex_id
                      << ") cannot be inserted\n";
            exit(0);
        }
        // Increment the in degree of the dst vertex.
        this->vertices[dst_vertex_id].inbound_degree++;
    }

    // Insert edge by `Address` type src and dst
    template<class VertexTypeOfAddress>
    inline auto insert_edge_by_address(CCASimulator& cca_simulator,
                                       /* MemoryAllocator& allocator, */
                                       Address src_vertex_addr,
                                       Address dst_vertex_addr,
                                       u_int32_t edge_weight) -> bool
    {

        auto* vertex = static_cast<VertexTypeOfAddress*>(cca_simulator.get_object(src_vertex_addr));
        bool success =
            vertex->insert_edge(cca_simulator, src_vertex_addr.cc_id, dst_vertex_addr, edge_weight);

        // Increament the `inbound_degree` of the destination vertex
        if (success) {
            auto* vertex =
                static_cast<VertexTypeOfAddress*>(cca_simulator.get_object(dst_vertex_addr));

            vertex->inbound_degree++;
        }

        return success;
    }

    auto get_vertices_ids_with_zero_in_degree() -> std::vector<u_int32_t>
    {
        // Find vertices with inbound_degree equal to 0.
        std::vector<u_int32_t> vertices_inbound_degree_zero;
        for (u_int32_t i = 0; i < this->total_vertices; i++) {
            // std::cout << this->vertices[i].inbound_degree << "\n";
            if (this->vertices[i].inbound_degree == 0) {
                vertices_inbound_degree_zero.push_back(this->vertices[i].id);
            }
        }
        // std::cout << "\n";
        return vertices_inbound_degree_zero;
    }

    auto get_vertices_ids_with_zero_out_degree() -> std::vector<u_int32_t>
    {
        // Find vertices with inbound_degree equal to 0.
        std::vector<u_int32_t> vertices_outbound_degree_zero;
        for (u_int32_t i = 0; i < this->total_vertices; i++) {
            // std::cout << this->vertices[i].outbound_degree << "\n";
            if (this->vertices[i].outbound_degree == 0) {
                vertices_outbound_degree_zero.push_back(this->vertices[i].id);
            }
        }
        // std::cout << "\n";
        return vertices_outbound_degree_zero;
    }

    // Initialize a newly created vertex in the CCA memory.
    // This is used for things like initializing the MemoryAllocator of the RecurssiveParallelVertex
    template<class VertexTypeOfAddress>
    inline auto init_vertex(CCASimulator& cca_simulator, Address src_vertex_addr) -> bool
    {
        auto* vertex = static_cast<VertexTypeOfAddress*>(cca_simulator.get_object(src_vertex_addr));
        return vertex->init(cca_simulator, src_vertex_addr.cc_id);
    }

    template<class VertexTypeOfAddress>
    void transfer_graph_host_to_cca(CCASimulator& cca_simulator,
                                    MemoryAllocator& allocator,
                                    std::optional<u_int32_t> start_vertex_id,
                                    bool shuffle_enabled)
    {

        // The vertex object that exists on the CCA needs to have edges of type `Address`.
        static_assert(std::is_same_v<decltype(VertexTypeOfAddress::edges[0].edge), Address>,
                      "edge type must be of type Address");

        u_int32_t starting_vertex_id = 0;
        if (start_vertex_id.has_value()) {
            starting_vertex_id = start_vertex_id.value();
            std::cout << "Vertex id " << starting_vertex_id
                      << " will be allocated first by the allocator. Then the rest of the vertices "
                         "will follow based on the allocator type."
                      << std::endl;
        }

        // Create a vector of ids of all vertices values from root to N cyclically
        std::vector<int> vertex_ids;
        for (int i = starting_vertex_id; i <= this->total_vertices; ++i) {
            vertex_ids.push_back(i);
        }
        for (int i = 0; i < starting_vertex_id; ++i) {
            vertex_ids.push_back(i);
        }

        if (shuffle_enabled) {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::shuffle(vertex_ids.begin(), vertex_ids.end(), gen);

            // Find the position of the root (starting_vertex_id) value in the shuffled vector
            auto rootPos = std::find(vertex_ids.begin(), vertex_ids.end(), starting_vertex_id);

            // If the root (starting_vertex_id) value is found, swap it with the first element
            if (rootPos != vertex_ids.end()) {
                std::swap(*rootPos, vertex_ids[0]);
            } else {
                std::cout << "Root value not found in the shuffled array. How is that possible?"
                          << std::endl;
                exit(0);
            }
            std::cout << "Shuffled the vertex id list for random allocation of vertices. May help "
                         "with synthetic graphs where the graph generator didn't do a good job."
                      << std::endl;
        }

        // Putting `vertex_` in a scope so as to not have it in the for loop and avoid calling the
        // constructor everytime.
        VertexTypeOfAddress vertex_(0, this->total_vertices);
        //  for (int i = 0; i < this->total_vertices; i++) {
        for (int i = 0; i < vertex_ids.size(); i++) {
            int current_vertex_id = vertex_ids[i];

            // Put a vertex in memory with id = current_vertex_id
            vertex_.id = current_vertex_id;

            // Get the Address of this vertex allocated on the CCA chip. Note here we use
            // VertexType<Address> since the object is now going to be sent to the CCA chip and
            // there the address type is Address (not u_int32_t ID)
            std::optional<Address> vertex_addr = cca_simulator.allocate_and_insert_object_on_cc(
                allocator, &vertex_, sizeof(VertexTypeOfAddress));

            if (!vertex_addr) {
                std::cerr << "Error! Memory not allocated for Vertex ID: "
                          << this->vertices[current_vertex_id].id << "\n";
                exit(0);
            }

            if (!this->init_vertex<VertexTypeOfAddress>(cca_simulator, vertex_addr.value())) {
                std::cerr << "Error! Vertex initialization failed for Vertex ID: "
                          << this->vertices[current_vertex_id].id << "\n";
                exit(0);
            }

            // Insert into the vertex_addresses map
            vertex_addresses[current_vertex_id] = vertex_addr.value();
        }

        std::cout << "Populating vertices by inserting edges: " << std::endl;

// TODO: Can we OpenMP parallelize this?
#pragma omp parallel for
        for (int i = 0; i < this->total_vertices; i++) {
            u_int32_t const src_vertex_id = this->vertices[i].id;

            for (int j = 0; j < this->vertices[i].number_of_edges; j++) {

                u_int32_t const dst_vertex_id = this->vertices[i].edges[j].edge;
                u_int32_t const edge_weight = this->vertices[i].edges[j].weight;

                if (!this->insert_edge_by_address<VertexTypeOfAddress>(
                        cca_simulator,
                        /*  allocator, */
                        this->vertex_addresses[src_vertex_id],
                        this->vertex_addresses[dst_vertex_id],
                        edge_weight)) {
                    std::cerr << "Error! Edge (" << src_vertex_id << ", " << dst_vertex_id << ", "
                              << edge_weight << ") not inserted successfully.\n";
                    exit(0);
                }
            }
        }
    }

    Graph(const std::string& input_graph_path)
    {

        // Generate or read the input data graph
        FILE* input_graph_file_handler = nullptr;

        if ((input_graph_file_handler = fopen(input_graph_path.c_str(), "r")) == nullptr) {
            std::cout << "The graph: " << input_graph_path << " failed to openn\n";
            exit(0);
        }

        fscanf(input_graph_file_handler, "%d\t%d", &this->total_vertices, &this->total_vertices);
        fscanf(input_graph_file_handler, "%d", &this->total_edges);

        std::cout << "The graph: " << input_graph_path
                  << " has total_vertices: " << this->total_vertices << " with "
                  << this->total_edges << " egdes." << std::endl;

        // this->vertices = std::make_shared<VertexType[]>(this->total_vertices);
        std::shared_ptr<VertexType[]> const vertices_(new VertexType[this->total_vertices],
                                                      std::default_delete<VertexType[]>());
        this->vertices = vertices_;

        for (int i = 0; i < this->total_vertices; i++) {
            this->vertices[i].id = i;
            this->vertices[i].number_of_edges = 0;
        }

        // Read from file and insert edges
        u_int32_t vertex_from = 0;
        u_int32_t vertex_to = 0;
        u_int32_t weight = 0;
        for (int i = 0; i < this->total_edges; i++) {
            fscanf(input_graph_file_handler, "%d\t%d\t%d", &vertex_from, &vertex_to, &weight);
            this->add_edge(this->vertices[vertex_from], vertex_to, weight);
        }
        fclose(input_graph_file_handler);
    }

    ~Graph() = default;
};

#endif // Graph_HPP

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

#ifndef Graph_HPP
#define Graph_HPP

#include "Constants.hpp"

#include <fstream>
#include <random>

// Used for dynamic graphs when reading the increment edgelist file.
struct EdgeTuple
{
    u_int32_t from;
    u_int32_t to;
    u_int32_t weight;
};

struct VertexInfo
{
    std::optional<Address> addresses[rhizome_size];
    u_int32_t inbound_degree[rhizome_size] = { 0 };

    // Start pointing the inbound edges to rhizome 0 but then as the `rhizome_inbound_degree_cutoff`
    // is reached then move to the next rhizome i.e. 1, 2, 3, until rhizome_size then back to 0.
    u_int32_t current_rhizome{};

    auto get_current_rhizome_address() -> std::optional<Address>
    {
        return this->addresses[this->current_rhizome];
    }

    void set_current_rhizome_address(std::optional<Address> addr)
    {
        this->addresses[this->current_rhizome] = addr;
    }

    void increment_inbound_degree()
    {
        this->inbound_degree[this->current_rhizome]++;

        // When the cutoff is reached then switch to the other rhizome.
        u_int32_t current_chunk =
            this->inbound_degree[this->current_rhizome] % rhizome_inbound_degree_cutoff;

        if (current_chunk == 0) {
            this->current_rhizome = (this->current_rhizome + 1) % rhizome_size;
        }
    }
};

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
    // TODO: Why is this a map? It can easily be a vector.
    std::map<u_int32_t, Address> vertex_addresses;

    // Holds information about the inbound degree for rhizomes and their addresses.
    // TODO: Investigate: strange that it would occasionally throw memory errors in destructor when
    // this was a vector and we .resize() in the transfer function. With a map it is ok now...
    // std::vector<VertexInfo> vertices_info;
    std::map<u_int32_t, VertexInfo> vertices_info;

    auto get_vertex_address_in_cca(u_int32_t vertex_id) -> Address
    {
        return this->vertex_addresses[vertex_id];
    }

    auto get_vertex_address_in_cca_rhizome(u_int32_t vertex_id) -> Address
    {
        return this->vertices_info[vertex_id].addresses[0].value();
    }

    // This is used to populate the host side graph when it is read from a file for example.
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

            // TODO: This might not be thread-safe!
            vertex->inbound_degree++;
        }

        return success;
    }

    // Insert edge by `Address` type src and dst with continuation.
    template<class VertexTypeOfAddress>
    inline auto insert_edge_by_address_with_continuation(CCASimulator& cca_simulator,
                                                         /* MemoryAllocator& allocator, */
                                                         Address src_vertex_addr,
                                                         Address dst_vertex_addr,
                                                         u_int32_t edge_weight,
                                                         u_int32_t root_vertex,
                                                         Address terminator,
                                                         CCAFunctionEvent continuation) -> bool
    {

        auto* vertex = static_cast<VertexTypeOfAddress*>(cca_simulator.get_object(src_vertex_addr));

        ActionArgumentType args_for_continuation =
            vertex->edge_insert_continuation_argument(dst_vertex_addr, edge_weight, root_vertex);

        /* auto* dvertex =
            static_cast<VertexTypeOfAddress*>(cca_simulator.get_object(dst_vertex_addr));

        std::cout << "svertex: " << vertex->id << ", src_vertex_addr: " << src_vertex_addr
                  << ", dvertex: " << dvertex->id << ", dst_vertex_addr: " << dst_vertex_addr
                  << "\n"; */

        bool success = vertex->insert_edge(cca_simulator,
                                           src_vertex_addr.cc_id,
                                           src_vertex_addr,
                                           dst_vertex_addr,
                                           edge_weight,
                                           args_for_continuation,
                                           terminator,
                                           continuation);

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
    // NOTE: Only to be used for RecurssiveParallelVertex
    template<class VertexTypeOfAddress>
    inline auto init_vertex(CCASimulator& cca_simulator, Address src_vertex_addr) -> bool
    {
        auto* vertex = static_cast<VertexTypeOfAddress*>(cca_simulator.get_object(src_vertex_addr));
        return vertex->init(cca_simulator, src_vertex_addr.cc_id);
    }

    // Initialize a newly created vertex in the CCA memory.
    // This is used for things like initializing the MemoryAllocator of the RecurssiveParallelVertex
    // NOTE: Only to be used for RhizomeRecurssiveParallelVertex
    template<class VertexTypeOfAddress>
    inline auto init_rhizome_vertex(CCASimulator& cca_simulator, Address src_vertex_addr) -> bool
    {
        auto* vertex = static_cast<VertexTypeOfAddress*>(cca_simulator.get_object(src_vertex_addr));
        return vertex->init(
            cca_simulator, src_vertex_addr.cc_id, true); // true: meaning it is rhizome
    }

    std::vector<u_int32_t> make_vertices_list(std::optional<u_int32_t> start_vertex_id,
                                              bool shuffle_enabled,
                                              int total_vertices)
    {
        u_int32_t starting_vertex_id = 0;
        if (start_vertex_id.has_value()) {
            starting_vertex_id = start_vertex_id.value();
            std::cout << "Vertex id " << starting_vertex_id
                      << " will be allocated first by the allocator. Then the rest of the vertices "
                         "will follow based on the allocator type."
                      << std::endl;
        }

        // Create a vector of ids of all vertices values from root to N cyclically
        std::vector<u_int32_t> vertex_ids;
        for (u_int32_t i = starting_vertex_id; i <= this->total_vertices; ++i) {
            vertex_ids.push_back(i);
        }
        for (u_int32_t i = 0; i < starting_vertex_id; ++i) {
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

        return vertex_ids;
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

        std::vector<u_int32_t> vertex_ids =
            make_vertices_list(start_vertex_id, shuffle_enabled, this->total_vertices);

        // Putting `vertex_` in a scope so as to not have it in the for loop and avoid calling the
        // constructor everytime.
        VertexTypeOfAddress vertex_(0, this->total_vertices);
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
            this->vertex_addresses[current_vertex_id] = vertex_addr.value();
        }

        std::cout << "Populating vertices by inserting edges: " << std::endl;

        // #pragma omp parallel for
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

    template<class VertexTypeOfAddress>
    void transfer_graph_host_to_cca_rhizome(CCASimulator& cca_simulator,
                                            MemoryAllocator& allocator,
                                            MemoryAllocator& random_allocator,
                                            std::optional<u_int32_t> start_vertex_id,
                                            bool shuffle_enabled)
    {

        // The vertex object that exists on the CCA needs to have edges of type `Address`.
        static_assert(std::is_same_v<decltype(VertexTypeOfAddress::edges[0].edge), Address>,
                      "edge type must be of type Address");

        // TODO: Perhaps this `int` must become `u_int32_t`
        std::vector<int> vertex_ids =
            make_vertices_list(start_vertex_id, shuffle_enabled, this->total_vertices);

        // Putting `vertex_` in a scope so as to not have it in the for loop and avoid calling the
        // constructor everytime.
        VertexTypeOfAddress vertex_(0, this->total_vertices);

        // Set the size of the vertices information vector before inserting into it.
        // this->vertices_info.resize(this->total_vertices);

        for (int i = 0; i < vertex_ids.size(); i++) {
            int current_vertex_id = vertex_ids[i];

            // Put a vertex in memory with id = current_vertex_id
            vertex_.id = current_vertex_id;

            // Get the Address of this vertex allocated on the CCA chip. Note here we use
            // VertexType<Address> since the object is now going to be sent to the CCA chip and
            // there the address type is Address (not u_int32_t ID).
            std::optional<Address> vertex_addr = cca_simulator.allocate_and_insert_object_on_cc(
                allocator, &vertex_, sizeof(VertexTypeOfAddress));

            if (!vertex_addr) {
                std::cerr << "Error! Memory not allocated for Vertex ID: "
                          << this->vertices[current_vertex_id].id << "\n";
                exit(0);
            }

            if (!this->init_rhizome_vertex<VertexTypeOfAddress>(cca_simulator,
                                                                vertex_addr.value())) {
                std::cerr << "Error! Vertex initialization failed for Vertex ID: "
                          << this->vertices[current_vertex_id].id << "\n";
                exit(0);
            }

            // Insert the address into the vertices_info vector.
            this->vertices_info[current_vertex_id].addresses[0] = vertex_addr.value();
        }

        std::cout << "Populating vertices by inserting edges: " << std::endl;

        // #pragma omp parallel for
        for (int i = 0; i < this->total_vertices; i++) {
            u_int32_t const src_vertex_id = this->vertices[i].id;

            for (int j = 0; j < this->vertices[i].number_of_edges; j++) {

                u_int32_t const dst_vertex_id = this->vertices[i].edges[j].edge;
                u_int32_t const edge_weight = this->vertices[i].edges[j].weight;

                // Create the Rhizome if needed. 
                if (!this->vertices_info[dst_vertex_id].get_current_rhizome_address()) {

                    std::optional<Address> vertex_addr =
                        cca_simulator.allocate_and_insert_object_on_cc(
                            random_allocator, &vertex_, sizeof(VertexTypeOfAddress));

                    if (!vertex_addr) {
                        std::cerr << "Error! Memory not allocated for the Rhizome of Vertex ID: "
                                  << this->vertices[dst_vertex_id].id << "\n";
                        exit(0);
                    }
                    /* std::cout << "Rhizome Created for dst_vertex_id: " << dst_vertex_id
                              << ", Total inbound: " << this->vertices[dst_vertex_id].inbound_degree
                              << "\n"; */

                    if (!this->init_rhizome_vertex<VertexTypeOfAddress>(cca_simulator,
                                                                        vertex_addr.value())) {
                        std::cerr
                            << "Error! Vertex initialization failed for the Rhizome of Vertex ID: "
                            << this->vertices[dst_vertex_id].id << "\n";
                        exit(0);
                    }

                    // Exchange the rhizome addresses between the rhizomes
                    // i.e. link them...
                    auto* new_rhizome_vertex = static_cast<VertexTypeOfAddress*>(
                        cca_simulator.get_object(vertex_addr.value()));

                    // 1. New adds all in self.
                    for (u_int32_t rhizome_iterator = 0;
                         rhizome_iterator < this->vertices_info[dst_vertex_id].current_rhizome;
                         rhizome_iterator++) {

                        if (!new_rhizome_vertex->set_rhizome(
                                this->vertices_info[dst_vertex_id].addresses[rhizome_iterator])) {
                            std::cerr
                                << "Error! new set_rhizome failed for the Rhizome of Vertex ID: "
                                << this->vertices[dst_vertex_id].id << "\n";
                            exit(0);
                        }
                    }

                    // 2. All other rhizomes add/link this new rhizome
                    // Exchange the rhizome addresses between the rhizomes i.e. link them ...
                    for (u_int32_t rhizome_iterator = 0;
                         rhizome_iterator < this->vertices_info[dst_vertex_id].current_rhizome;
                         rhizome_iterator++) {

                        auto* other_rhizome_vertex = static_cast<VertexTypeOfAddress*>(
                            cca_simulator.get_object(this->vertices_info[dst_vertex_id]
                                                         .addresses[rhizome_iterator]
                                                         .value()));

                        if (!other_rhizome_vertex->set_rhizome(vertex_addr)) {
                            std::cerr << "Error! other_rhizome_vertex set_rhizome failed for the "
                                         "Rhizome of Vertex ID: "
                                      << this->vertices[dst_vertex_id].id << "\n";
                            exit(0);
                        }
                    }

                    // 3. Insert the address into the Graph::vertices_info map.
                    this->vertices_info[dst_vertex_id].set_current_rhizome_address(vertex_addr);
                }

                if (!this->insert_edge_by_address<VertexTypeOfAddress>(
                        cca_simulator,
                        /*  allocator, */
                        this->vertices_info[src_vertex_id].addresses[0].value(),
                        this->vertices_info[dst_vertex_id].get_current_rhizome_address().value(),
                        edge_weight)) {
                    std::cerr << "Error! Edge (" << src_vertex_id << ", " << dst_vertex_id << ", "
                              << edge_weight << ") not inserted successfully.\n";
                    exit(0);
                } else {
                    // Insertion was successful. Now increment the local count for inbound edges
                    // for dst vertex to be used to form Rhizomes.
                    this->vertices_info[dst_vertex_id].increment_inbound_degree();
                }
            }
        }
    }

    template<bool ZERO_INDEX>
    auto read_dnyamic_graph_increment(const std::string& input_graph_path) -> std::vector<EdgeTuple>
    {

        // Read the input data graph
        std::ifstream input_graph_file_handler(input_graph_path);

        // Check if the file is open
        if (!input_graph_file_handler.is_open()) {
            std::cerr << "The graph: " << input_graph_path << " failed to open\n";
            exit(0);
        }

        std::vector<EdgeTuple> new_edges;

        // Read from file and insert edges
        EdgeTuple reader_edge;
        while (input_graph_file_handler >> reader_edge.from >> reader_edge.to >>
               reader_edge.weight) {
            // Insert the edge in `this` graph i.e. host side store
            if constexpr (ZERO_INDEX) {
                reader_edge.from--;
                reader_edge.to--;
            }
            this->add_edge(this->vertices[reader_edge.from], reader_edge.to, reader_edge.weight);

            // Insert the edge in the vector to be returned to caller so that it can then call the
            // insert_edge to transfer this new edgelist to the device.
            new_edges.emplace_back(reader_edge);
            this->total_edges++;
        }

        // Close the file
        input_graph_file_handler.close();

        return new_edges;
    }

    // For dynamic graphs transfer the new edgelist. Note: this function does not accept creation of
    // new vertices, although its not an issue and can be done easily in the future.
    template<class VertexTypeOfAddress>
    void transfer_graph_edges_increment_host_to_cca(CCASimulator& cca_simulator,
                                                    std::vector<EdgeTuple>& new_edges,
                                                    u_int32_t root_vertex,
                                                    Address terminator,
                                                    CCAFunctionEvent continuation)
    {

        // The vertex object that exists on the CCA needs to have edges of type `Address`.
        static_assert(std::is_same_v<decltype(VertexTypeOfAddress::edges[0].edge), Address>,
                      "edge type must be of type Address");

        std::cout << "Inserting increment of new edges: " << std::endl;

        // Not sure if this is thread safe anymore... #pragma omp parallel for
        for (u_int32_t i = 0; i < new_edges.size(); i++) {

            u_int32_t const src_vertex_id = new_edges[i].from;
            u_int32_t const dst_vertex_id = new_edges[i].to;
            u_int32_t const edge_weight = new_edges[i].weight;

            if (!this->insert_edge_by_address_with_continuation<VertexTypeOfAddress>(
                    cca_simulator,
                    /*  allocator, */
                    this->vertex_addresses[src_vertex_id],
                    this->vertex_addresses[dst_vertex_id],
                    edge_weight,
                    root_vertex,
                    terminator,
                    continuation)) {
                std::cerr << "Error! Edge (" << src_vertex_id << ", " << dst_vertex_id << ", "
                          << edge_weight << ") not inserted successfully.\n";
                exit(0);
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

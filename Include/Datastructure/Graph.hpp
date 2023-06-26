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

template<class VertexType>
class Graph
{
  public:
    u_int32_t total_vertices;
    u_int32_t total_edges;
    std::shared_ptr<VertexType[]> vertices;

    Graph(u_int32_t total_vertices_in)
    {
        this->total_vertices = total_vertices_in;

        // this->vertices = std::make_shared<VertexType[]>(this->total_vertices);
        std::shared_ptr<VertexType[]> vertices(new VertexType[total_vertices],
                                               std::default_delete<VertexType[]>());
        this->vertices = vertices;

        for (int i = 0; i < this->total_vertices; i++) {
            this->vertices[i].id = i;
            this->vertices[i].number_of_edges = 0;
        }
    }
    void add_edge(VertexType& vertex, u_int32_t dst_vertex_id, u_int32_t weight)
    {
        if (!vertex.insert_edge(dst_vertex_id, weight)) {
            std::cerr << "Error! add_edge() Edge (" << vertex.id << ", " << dst_vertex_id
                      << ") cannot be inserted\n";
            exit(0);
        }
    }

    Graph(std::string input_graph_path)
    {

        // Generate or read the input data graph
        FILE* input_graph_file_handler = NULL;

        if ((input_graph_file_handler = fopen(input_graph_path.c_str(), "r")) == NULL) {
            exit(0);
        }

        fscanf(input_graph_file_handler, "%d\t%d", &this->total_vertices, &this->total_vertices);
        fscanf(input_graph_file_handler, "%d", &this->total_edges);

        std::cout << "The graph: " << input_graph_path
                  << " has total_vertices: " << this->total_vertices << " with "
                  << this->total_edges << " egdes.\n";

        // this->vertices = std::make_shared<VertexType[]>(this->total_vertices);
        std::shared_ptr<VertexType[]> vertices_(new VertexType[this->total_vertices],
                                                std::default_delete<VertexType[]>());
        this->vertices = vertices_;

        for (int i = 0; i < this->total_vertices; i++) {
            this->vertices[i].id = i;
            this->vertices[i].number_of_edges = 0;
        }

        // Read from file and insert edges
        u_int32_t vertex_from;
        u_int32_t vertex_to;
        u_int32_t weight;
        for (int i = 0; i < this->total_edges; i++) {
            fscanf(input_graph_file_handler, "%d\t%d\t%d", &vertex_from, &vertex_to, &weight);
            this->add_edge(this->vertices[vertex_from], vertex_to, weight);
        }
        fclose(input_graph_file_handler);
    }

    ~Graph() {}
};

#endif // Graph_HPP

# The file is adapted from a class assignment in Graph Analytics
# during graduate studies at Indiana University Bloomington.
# Bibrak Qamar

import random
import networkx as nx
import sys

import time
from collections import deque

# For statistics
import pandas as pd
pd.options.display.float_format = '{:.2f}'.format


args = sys.argv
# Read input data from file
filename = args[1]

Output_filename = filename+".output"


def write_to_file(filename, content):
    with open(filename, 'a') as f:
        f.write(content+"\n")


# Input: A graph
# Find shortest paths in the largest 5 componets and plot distribution
def ShortestPaths_Analysis(G):
    cc_sorted = sorted(nx.connected_components(G), key=len, reverse=True)
    """ for i in cc_sorted:
        print("Length of component: ", len(i)) """
    # find shortest paths in top 1 components
    topcc = min(len(cc_sorted), 1)
    for i in range(topcc):
        cc = cc_sorted[i]
        cc_graph = G.subgraph(cc)

        if (len(cc) > 100):
            print(
                "This component is too large. Using 100 single-source shortest paths.")
            cc = list(cc)
            cc_graph = G.subgraph(cc)
            shortest_path_lens = []

            """ one_way_sample_size = 50
            begining_vertices = list(range(0, one_way_sample_size))
            last_vertices = list(
                range(len(cc)-1, len(cc) - one_way_sample_size - 1, -1))
            sample_vertices = begining_vertices + last_vertices """
            sample_vertices = random.sample(list(cc_graph.nodes()), 99)
            sample_vertices = [0] + sample_vertices
            print("Random vertices for SSSP: ", sample_vertices)
            # print(sample_vertices)

            for i in sample_vertices:
                print('Finding SSSP for ', i)
                length = nx.single_source_shortest_path_length(cc_graph, i)
                shortest_path_lens += [v for v in length.values()]
        else:
            all_shortest_path_dict = dict(
                nx.all_pairs_shortest_path_length(cc_graph))
            shortest_path_lens = []
            for val1 in all_shortest_path_dict.values():
                for val in val1.values():
                    shortest_path_lens.append(val)

        # print(shortest_path_lens)
        avg_shortest_path_lens = sum(
            shortest_path_lens)/len(shortest_path_lens)
        # print("Average shortest_path_lens: ", avg_shortest_path_lens)
        # print('Len of shortest_path_lens: ', len(shortest_path_lens))

        """ plot_distribution(shortest_path_lens, xlabel='Shortest path lengths (hops)',
                          ylabel='Number of paths', title='Shortest path lengths distributions '+A,
                          xlog=False, ylog=False, showLine=True, intAxis=True)

        plt.savefig("ShortestPaths_Analysis"+A+".png") """

        print("Shortest Path Lengths Statistics")
        s = pd.Series(shortest_path_lens)
        print(s.describe())
        write_to_file(Output_filename, "ShortestPaths_Analysis")
        write_to_file(Output_filename, str(s.describe()))


def In_Degree_Distribution(G):

    degree = G.in_degree()
    degree = [deg for (v, deg) in degree]
    avg_degree = sum(degree)/len(degree)

    print("In Degree Distribution Statistics")
    s = pd.Series(degree)
    print(s.describe(percentiles=[0.10, 0.20, 0.30,
                                  0.40, 0.50, 0.60, 0.70, 0.80, 0.84, 0.86, 0.90, 0.92, 0.94, 0.96, 0.98, 0.99]))
    write_to_file(Output_filename, "In Degree Distribution Statistics")
    write_to_file(Output_filename, str(s.describe(percentiles=[0.10, 0.20, 0.30,
                                                               0.40, 0.50, 0.60, 0.70, 0.80, 0.84, 0.86, 0.90, 0.92, 0.94, 0.96, 0.98, 0.99])))

    sorted_degrees = sorted(G.in_degree, key=lambda x: x[1], reverse=True)
    print('highest in degree vertex: ', sorted_degrees[0][0])
    
    # Print the first 20 entries
    for i, (node, degree) in enumerate(sorted_degrees[:30], start=1):
        print(f"Node {node}: In-degree {degree}")
    
    write_to_file(Output_filename, "highest in degree vertex: " +
                  str(sorted_degrees[0][0]))


def Out_Degree_Distribution(G):

    degree = G.out_degree()
    degree = [deg for (v, deg) in degree]
    # avg_degree = sum(degree)/len(degree)

    print("Out Degree Distribution Statistics")
    s = pd.Series(degree)
    print(s.describe(percentiles=[0.10, 0.20, 0.30,
                                  0.40, 0.50, 0.60, 0.70, 0.80, 0.84, 0.86, 0.90, 0.92, 0.94, 0.96, 0.98, 0.99]))

    write_to_file(Output_filename, "Out Degree Distribution Statistics")
    write_to_file(Output_filename, str(s.describe(percentiles=[0.10, 0.20, 0.30,
                                                               0.40, 0.50, 0.60, 0.70, 0.80, 0.84, 0.86, 0.90, 0.92, 0.94, 0.96, 0.98, 0.99])))

    sorted_degrees = sorted(G.out_degree, key=lambda x: x[1], reverse=True)
    print('highest out degree vertex: ', sorted_degrees[0][0])
    
    # Print the first 20 entries
    for i, (node, degree) in enumerate(sorted_degrees[:30], start=1):
        print(f"Node {node}: Out-degree {degree}")
    
    
    write_to_file(Output_filename, "highest out degree vertex: " +
                  str(sorted_degrees[0][0]))
    return sorted_degrees[0][0]

# Input: A graph
# Find the sizes of all connected components and plot the distribution


def CC_Distribution(G):
    cc_sorted = sorted(nx.connected_components(G), key=len, reverse=True)
    # print statistics of the top 5 components (if exist)
    topcc = min(len(cc_sorted), 5)
    for i in range(topcc):
        cc = cc_sorted[i]
        cc_graph = G.subgraph(cc)
        n = cc_graph.number_of_nodes()
        m = cc_graph.number_of_edges()
        n_percent = (n/G.number_of_nodes()) * 100
        print("Largest component #", i+1)
        print("Number of vertices:", n, " (", n_percent, ")",
              "\nNumber of edges: ", m, "\n")

# Perform BFS using nx.bfs_tree


def bfs(graph, source, Output_filename):
    print("Running BFS with Source: ", source)
    bfs_tree = nx.bfs_tree(graph, source)
    level = {source: 0}  # Dictionary to store the level of each node
    # Queue to keep track of nodes and their levels
    queue = deque([(source, 0)])

    while queue:
        node, node_level = queue.popleft()
        for neighbor in bfs_tree.neighbors(node):
            if neighbor not in level:
                level[neighbor] = node_level + 1
                queue.append((neighbor, node_level + 1))

    # Sort the level dictionary based on keys
    sorted_level = sorted(level.items(), key=lambda x: x[0])

    # Open a file for writing the SSSP values
    with open(Output_filename, "w") as file:
        file.write(f"# Source\n")
        file.write(f"{source}\n")
        # Write the BFS values to the file
        for vertex, level in sorted_level:
            file.write(f"{vertex}\t{level}\n")


# Perform SSSP using nx.single_source_dijkstra
def sssp(graph, source, Output_filename):
    print("Running SSSP with Source: ", source)
    sssp = nx.single_source_dijkstra(graph, source=source)

    # Sort the vertices based on their keys
    sorted_vertices = sorted(sssp[0].items(), key=lambda x: x[0])

    write_to_file(Output_filename, "# Source")
    write_to_file(Output_filename, str(source))

    # Open a file for writing the SSSP values
    with open(Output_filename, "w") as file:
        file.write(f"# Source\n")
        file.write(f"{source}\n")
        # Write the SSSP values to the file
        for vertex, length in sorted_vertices:
            file.write(f"{vertex}\t{length}\n")

# Perform pagerank using nx.pagerank ignore weights.


def pagerank(graph, Output_filename):
    # Calculate PageRank
    pagerank = nx.pagerank(graph, alpha=0.85, max_iter=100,
                           weight=None, dangling=None)

    # Open a file for writing the PageRank values
    with open(Output_filename, "w") as file:
        # Write the PageRank values to the file
        for vertex, pagerank_value in pagerank.items():
            file.write(f"{vertex}\t{pagerank_value}\n")


# Read graph from file
G = nx.DiGraph()
with open(filename, 'r') as file:
    for _ in range(2):  # Skip the first two lines
        next(file)
    for line in file:
        src, dest, weight = line.split()
        G.add_edge(int(src), int(dest), weight=float(weight))

n = G.number_of_nodes()
m = G.number_of_edges()
print('Is directed: ', nx.is_directed(G))
print("Directed : Number of vertices: ", n, ", Number of edges: ", m)
write_to_file(Output_filename, "Directed : Number of vertices: " +
              str(n) + ", Number of edges: "+str(m))

# Get all vertex IDs in the graph
""" vertex_ids = list(G.nodes())

# Find the maximum vertex ID
max_vertex_id = max(vertex_ids) if vertex_ids else -1

# Check for missing vertices from 0 to max_vertex_id
missing_vertices = [v for v in range(max_vertex_id + 1) if v not in vertex_ids]

# print("All vertex IDs:", vertex_ids)
print("max_vertex_id: ", max_vertex_id)
print("Missing vertices:", missing_vertices) """


""" start = time.time()
ShortestPaths_Analysis(G.to_undirected())
end = time.time()
print("Time in SSSP: ", end-start, "\n") """

start = time.time()
In_Degree_Distribution(G)
end = time.time()
print("Time in In_Degree_Distribution: ", end-start, "\n")


start = time.time()
max_out_degree_vertex = Out_Degree_Distribution(G)
end = time.time()
print("Time in Out_Degree_Distribution: ", end-start, "\n")


""" # Call BFS starting from node 0. Write the levels in a file.
bfs(G, max_out_degree_vertex, filename+".bfs")

# Call SSSP starting from node 0. Write the levels in a file.
sssp(G, max_out_degree_vertex, filename+".sssp")

# Perform pagerank using nx.pagerank. Write the values in a file.
pagerank(G, filename+".pagerank") """


# G2 = G.to_undirected()
# CC_Distribution(G2)

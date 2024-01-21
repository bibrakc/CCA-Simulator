# Refine Input Data Graphs

## Convert from other formats to .edgelist format
Compile the `.cpp` files and use the executables to convert from other formats to the `.edgelist` format to be used by the CCASimulator.

## Generate stats of the input graph
`$ python3 generate_graph_reader.py <input_graph.edgelist>`

## Output
The python script will create files: 
1. `.output`: containing statistics about the generated graph.
2. `.bfs`: containing breadth first search levels from source `0`.
3. `.sssp`: containing single source shortest path lengths from source `0`.
4. `.pagerank`: containing pagerank values.

`.bfs`, `.sssp`, and `.pagerank` can be used by applications for verification of results.

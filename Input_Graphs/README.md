# Generate Synthetic Graphs Using NetworkX
`$ python3 generate_input_graphs.py <Powerlaw|Scalefree|Smallworld|Erdos|RMAT> <directed|not_directed> <edge-factor> <scale-factor>`

Here:
1. `[Powerlaw, Scalefree, Smallworld, Erdos, RMAT]` are the graph types.
2. `directed` or `not_directed`:  If `directed` then use `directed=True` parameter whenever available.
3. `edge-factor` is the number of edges per vertex (approximately).
4. `scale-factor` is the 2^n number of vertices, where n is the `scale-factor`.

## Output
The python script will create files: 
1. `.edgelist`: containting the edges of the grath. The first two lines in the file contain the meta-data about the grapth, which includes the number of vertices and number of edges.
2. `.output`: containing statistics about the generated graph.
3. `.bfs`: containing breadth first search levels from source `0`.
4. `.sssp`: containing single source shortest path lengths from source `0`.
5. `.pagerank`: containing pagerank values.

`.bfs`, `.sssp`, and `.pagerank` can be used by applications for verification of results.

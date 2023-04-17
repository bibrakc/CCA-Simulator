# Generate Synthetic Graphs Using NetworkX
`$ generate_input_graphs.py <Powerlaw|Scalefree|Smallworld|Erdos> <edge-factor> <scale-factor>`

Here:
1. `[Powerlaw, Scalefree, Smallworld, Erdos]` are the graph types.
2. `edge-factor` is the number of edges per vertex (approximately).
3. `scale-factor` is the 2^n number of vertices, where n is the `scale-factor`.

## Output
The python script will create the `.edgelist` file containting the edges of the grath. The first two lines in the file contain the meta-data about the grapth, which includes the number of vertices and number of edges.

It will also output statistics about the graph in a `.output.txt` file.

It will also generate plots of the statistics.

# Page Rank Fixed Iterations

## Notes
Seems like over diffusiveness degrades the performance. Worst performance even if `nested_iterations = 2`. Non-nested iterations from the host seems to have great performance. This can be verified by implementing iterative versions of sssp and bfs.

## Limitations
For a directed graph there must not be any vertex with zero inbound degree. If such a case arises, the vertex will remain inactive and consequently not contribute to the score. Moreover, this inactive vertex might hinder the update of scores for other vertices, as they depend on the activation of this vertex.

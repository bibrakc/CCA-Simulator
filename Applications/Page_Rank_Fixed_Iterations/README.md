# Page Rank Fixed Iterations

## Limitations
For a directed graph there much not be any vertex with zero inbound degree. If this occurs then that vertex will not get activated and therefore will not contribute to the score. It might actually not allow other vertices to finally update their scores since they are waiting on this vertex.
# Page Rank Fixed Iterations

## Limitations
For a directed graph there must not be any vertex with zero inbound degree. If such a case arises, the vertex will remain inactive and consequently not contribute to the score. Moreover, this inactive vertex might hinder the update of scores for other vertices, as they depend on the activation of this vertex.

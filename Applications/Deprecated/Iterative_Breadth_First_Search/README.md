# Iterative_Breadth_First_Search
`Iterative_Breadth_First_Search` application implements asynchronous `breadth first search` using the CCASimulator. In particular it uses Iterative Deepening to traverse the graph.

## Building Using CMake
To compile the application, execute the following `cmake` commands to generate the executable.
> `$ CC=gcc-13 CXX=g++-13 cmake -S . -B build -D THROTTLE=true`

> `$ cmake --build build`

- `THROTTLE=true/false`: for enabling throttle of diffusion to mitigate congestion.
- `-D ANIMATION=true/false`: for recording and writing the simulation animation data.
- `-D MAXEDGESPERVERTEX=<int value>`: sets the max edges per vertex object before creating a new ghost vertex.
- `-D TERMINATION=true/false`: for running the termination detection algorithm or not. When it is false there won't be any ack messages for each action recieved and that way the overheads of termination can be calculated. This is for benchmarking purposes normally the termination detection will be on.

## Executing
Assuming the current directory is `/Applications/Iterative_Breadth_First_Search`
### Using Low-Latency Network (Htree)
> `$ ./build/Iterative_BFS_CCASimulator -f ../../Input_Graphs/Erdos-Renyi_directed_ef_16_v_11.edgelist -g Erdos -od ./Output -s square -root 0 -m 90000 -hx 3 -hy 3 -hdepth 4 -hb 128 -route 0 -iter 5 -verify`

### Using Pure Mesh Netowrk
> `$ ./build/Iterative_BFS_CCASimulator -f ../../Input_Graphs/Erdos-Renyi_directed_ef_16_v_11.edgelist -g Erdos -od ./Output -s square -root 0 -m 90000 -hx 48 -hy 48 -hdepth 0 -hb 0 -route 0 -iter 5 -verify`

- Make sure to have the output `-od ./Output` directory created before runing the application.
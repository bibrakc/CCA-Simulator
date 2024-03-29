# Iterative_Single_Source_Shortest_Path
`Iterative_Single_Source_Shortest_Path` application implements asynchronous `single source shortest path` using the CCASimulator. In particular it uses Iterative Deepening to traverse the graph.

## Building Using CMake
To compile the application, execute the following `cmake` commands to generate the executable.
> `$ CC=gcc-13 CXX=g++-13 cmake -S . -B build -D THROTTLE=true -D RECVBUFFSIZE=2 -D ANIMATION=true`

> `$ cmake --build build`

- `THROTTLE=true/false`: for enabling throttle of diffusion to mitigate congestion.
- `-D ANIMATION=true/false`: for recording and writing the simulation animation data.

## Executing
Assuming the current directory is `/Applications/Iterative_Single_Source_Shortest_Path`
### Using Low-Latency Network (Htree)
> `$ ./build/Iterative_SSSP_CCASimulator -f ../../Input_Graphs/Erdos-Renyi_directed_ef_16_v_11.edgelist -g Erdos -od ./Output -s square -root 0 -m 90000 -hx 3 -hy 3 -hdepth 4 -hb 128 -route 0 -iter 8 -verify`

### Using Pure Mesh Netowrk
> `$ ./build/Iterative_SSSP_CCASimulator -f ../../Input_Graphs/Erdos-Renyi_directed_ef_16_v_11.edgelist -g Erdos -od ./Output -s square -root 0 -m 90000 -hx 48 -hy 48 -hdepth 0 -hb 0 -route 0 -iter 8 -verify`

- Make sure to have the output `-od ./Output` directory created before runing the application.
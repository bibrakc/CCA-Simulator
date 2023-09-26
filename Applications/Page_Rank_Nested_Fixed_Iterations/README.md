# Page_Rank_Nested_Fixed_Iterations
`Page_Rank_Nested_Fixed_Iterations` application implements an asynchronous `page rank` with nested fixed number of iterations using the CCASimulator. In this application there is no convergence implemented rather the program runs for a fixed number if page rank iterations provided by the user. These host side iterations invoke the `nested page rank` where it asynchronously goes into the next iteration. Kind of an overlap.


## Building Using CMake
To compile the application, execute the following `cmake` commands to generate the executable.
> `$ CC=gcc-13 CXX=g++-13 cmake -S . -B build -D THROTTLE=true`

> `$ cmake --build build`

- `THROTTLE=true/false`: for enabling throttle of diffusion to mitigate congestion.

## Executing
Assuming the current directory is `/Applications/Page_Rank_Nested_Fixed_Iterations`, and `-iter 5` iterations to perform. `-verify` is optional but when enabled reads from an acompanying `.pagerank` file that contains precomputed pagerank values. So, make sure to have that file.
### Using Low-Latency Network (Htree)
> `$ ./build/PageRank_Nested_Fixed_Iterations_CCASimulator -f ../../Input_Graphs/Erdos-Renyi_directed_ef_16_v_11.edgelist -g Erdos -od ./Output -s square -root 3 -m 90000 -hx 3 -hy 3 -hdepth 4 -hb 128 -route 0 -iter 5 -verify`

### Using Pure Mesh Netowrk
> `$ ./build/PageRank_Nested_Fixed_Iterations_CCASimulator -f ../../Input_Graphs/Erdos-Renyi_directed_ef_16_v_11.edgelist -g Erdos -od ./Output -s square -root 3 -m 90000 -hx 48 -hy 48 -hdepth 0 -hb 0 -route 0 -iter 5 -verify`

- Make sure to have the output `-od ./Output` directory created before runing the application.
- The `root` will only be used if there are no vertices with `in degree` of zero. The application will try to find if there are any vertices with zero `in degre` and if it finds then it will germinate the page rank action on all those vertices.

## Limitations
For a directed graph if there is any vertex with zero outbound degree then the networkx calculated .pagerank score might no match since perhaps networkx uses some form of dandling calculations that this version does not. It needs to be investigated further. Right now it shouldn't be an issue as far as the performance and understanding of the behaviour is concerned.
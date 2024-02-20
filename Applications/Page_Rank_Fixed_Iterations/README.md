# Page_Rank_Fixed_Iterations
`Page_Rank_Fixed_Iterations` application implements asynchronous `page rank` with fixed number of iterations using the CCASimulator. In this application there is no convergence implemented rather the program runs for a fixed number if page rank iterations provided by the user.

## Building Using CMake
To compile the application, execute the following `cmake` commands to generate the executable.
> `$ CC=gcc-13 CXX=g++-13 cmake -S . -B build -D THROTTLE=true`

> `$ cmake --build build`

- `-D THROTTLE=true/false`: for enabling throttle of diffusion to mitigate congestion.
- `-D ANIMATION=true/false`: for recording and writing the simulation animation data. To be used by `../Analytics/Animations/cca_chip_active_status_animation.py`
- `-D ACTIVE_PERCENT=true/false`: for recording and writing the simulation active status as percentage for each cycle. To be used by `../Analytics/Post_Processing/post_processing.py`
- `-D ACTIONQUEUESIZE=<int value>`: side of the action queue. Use `64` or more/less or whatever.
- `-D MAXEDGESPERVERTEX=<int value>`: sets the max edges per vertex object before creating a new ghost vertex.
- `-D VICINITY=<int value>`: sets the radius of allocation for the vicinity allocator.
- `-D TERMINATION=true/false`: for running the termination detection algorithm or not. When it is false there won't be any ack messages for each action recieved and that way the overheads of termination can be calculated. This is for benchmarking purposes normally the termination detection will be on.
- `-D THROTTLE_CONGESTION_THRESHOLD=<int value>`: When there is congestion at a compute cell then that compute cell cools down for a period of cycles before generating new operons. This period is provided in `THROTTLE_CONGESTION_THRESHOLD`.
- `-D RECVBUFFSIZE=<int value>`: sets the size of the buffers at each channel of the compute cell.

## Executing
Assuming the current directory is `/Applications/Page_Rank_Fixed_Iterations`, and `-iter 5` iterations to perform. 
- `-verify` is optional but when enabled reads from an acompanying `.pagerank` file that contains precomputed pagerank values. So, make sure to have that file.
- Make sure to have the output `-od ./Output` directory created before runing the application.
### Using Pure Mesh or Torus-Mesh Netowrk
> `$ ./build/PageRank_Fixed_Iterations_CCASimulator -f ../../Input_Graphs/Erdos-Renyi_directed_ef_16_v_11.edgelist -g Erdos -od ./Output -s square -root 3 -m 90000 -hx 48 -hy 48 -hdepth 0 -hb 0 -route 0 -iter 5 -verify`
### Using Low-Latency Network (Htree) - NOT TESTED!!
> `$ ./build/PageRank_Fixed_Iterations_CCASimulator -f ../../Input_Graphs/Erdos-Renyi_directed_ef_16_v_11.edgelist -g Erdos -od ./Output -s square -root 3 -m 90000 -hx 3 -hy 3 -hdepth 4 -hb 128 -route 0 -iter 5 -verify`

## A Note on Verification
The `netwokx` code used to compute the page rank has a parameter called `tol` for tolerance. If it is low and in our test runs we use a higher number of iterations `-iter` then our solutions might be better and thus verification will throw errors for correctness. Make sure both are meaningful.

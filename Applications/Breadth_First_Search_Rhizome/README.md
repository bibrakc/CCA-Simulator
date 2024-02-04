# Breadth_First_Search_Rhizome
`Breadth_First_Search` application implements asynchronous `breadth first search` using the CCASimulator. In particular, it uses Rhizomes to partion any single large vertex into many Rhizomatic links to improve ingrees (in-degree) load.

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
Assuming the current directory is `/Applications/Breadth_First_Search`
### Using Low-Latency Network (Htree) - Deprecated
> `$ ./build/BFS_Rhizome_CCASimulator -f ../../Input_Graphs/Erdos-Renyi_directed_ef_16_v_11.edgelist -g Erdos -od ./Output -s square -root 0 -m 90000 -hx 3 -hy 3 -hdepth 4 -hb 128 -route 0 -mesh 1 -shuffle -verify`

### Using Only Mesh/Torus Netowrk
> `$ ./build/BFS_Rhizome_CCASimulator -f ../../Input_Graphs/Erdos-Renyi_directed_ef_16_v_11.edgelist -g Erdos -od ./Output -s square -root 0 -m 90000 -hx 48 -hy 48 -hdepth 0 -hb 0 -route 0 -mesh 1 -shuffle -verify`

- `-mesh 1`: represents the Torus mesh. `0`: is pure mesh.
- Make sure to have the output `-od ./Output` directory created before runing the application.
- `-shuffle`: to toggle shuffling of vertices for better load balancing.
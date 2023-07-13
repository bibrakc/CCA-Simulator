# CCA-Simulator
Event-level simulator for [Continuum Computer Architecture (CCA)](https://superfri.org/index.php/superfri/article/view/188) class of designs.

## Summary
The CCA Simulator enables exploring design space of the CCA class of non-Von Neumann intelligent memory systems. These systems are concieved to be highly fine-grain parallel and use event-driven mechanisms to perform computation.

## Building Using CMake
To compile the applications, navigate to the [Applications](/Applications) directory and select a specific application. Once inside the chosen application directory, execute the following `cmake` commands to generate the executable.
> `$ CC=gcc-13 CXX=g++-13 cmake -S . -B build -D THROTTLE=true`

> `$ cmake --build build`

- `THROTTLE=true/false`: for enabling throttle of diffusion to mitigate congestion.

## Executing
### SSSP Application
Assuming the current directory is `/Applications/Single_Source_Shortest_Path`
#### Using Low-Latency Network (Htree)
> `$ ./build/SSSP_CCASimulator -f ./Generated_Graphs/Erdos-Renyi_ef_9_v_12.edgelist -g Erdos -od ./Output -s square -root 0 -m 90000 -hx 3 -hy 3 -hdepth 4 -hb 256 -route 0 -verify`

#### Using Pure Mesh Netowrk
> `$ ./build/SSSP_CCASimulator -f ./Generated_Graphs/Erdos-Renyi_ef_8_v_14.edgelist -g Erdos -od ./Output -s square -root 0 -m 90000 -hx 96 -hy 96 -hdepth 0 -hb 0 -route 0 -verify`

### BFS Application
Assuming the current directory is `/Applications/Breadth_First_Search`
#### Using Low-Latency Network (Htree)
> `$ ./build/BFS_CCASimulator -f ./Generated_Graphs/Erdos-Renyi_ef_9_v_12.edgelist -g Erdos -od ./Output -s square -root 0 -m 90000 -hx 3 -hy 3 -hdepth 4 -hb 256 -route 0 -verify`

#### Using Pure Mesh Netowrk
> `$ ./build/BFS_CCASimulator -f ./Generated_Graphs/Erdos-Renyi_ef_8_v_14.edgelist -g Erdos -od ./Output -s square -root 0 -m 90000 -hx 96 -hy 96 -hdepth 0 -hb 0 -route 0 -verify`

### Page Rank Fixed Iterations Application
Assuming the current directory is `/Applications/Page_Rank_Fixed_Iterations`, and 100 iterations to perform. `-verify` is optional but when enabled reads from an acompanying `.pagerank` file that contains precomputed pagerank values. So, make sure to have that file.
#### Using Low-Latency Network (Htree)
> `$ ./build/PageRank_Fixed_Iterations_CCASimulator -f ./Generated_Graphs/Erdos-Renyi_ef_9_v_12.edgelist -g Erdos -od ./Output -s square -root 3 -m 90000 -hx 3 -hy 3 -hdepth 4 -hb 256 -route 0 -iter 5 -verify`

#### Using Pure Mesh Netowrk
> `$ ./build/PageRank_Fixed_Iterations_CCASimulator -f ../Generated_Graphs/Erdos-Renyi_ef_9_v_12.edgelist -g Erdos -od ../../Output -s square -root 3 -m 90000 -hx 46 -hy 46 -hdepth 0 -hb 0 -route 0 -iter 5 -verify`

### Page Rank Nested Fixed Iterations Application
Assuming the current directory is `/Applications/Page_Rank_Nested_Fixed_Iterations`, and 100 iterations to perform. `-verify` is optional but when enabled reads from an acompanying `.pagerank` file that contains precomputed pagerank values. So, make sure to have that file. To change the number of nested iterations edit `inline constexpr u_int32_t nested_iterations` in the `.hpp`. 
#### Using Low-Latency Network (Htree)
> `$ ./build/PageRank_Nested_Fixed_Iterations_CCASimulator -f ./Generated_Graphs/Erdos-Renyi_ef_9_v_12.edgelist -g Erdos -od ./Output -s square -root 0 -m 90000 -hx 3 -hy 3 -hdepth 4 -hb 256 -route 0 -iter 5 -verify`

#### Using Pure Mesh Netowrk
> `$ ./build/PageRank_Nested_Fixed_Iterations_CCASimulator -f ../Generated_Graphs/Erdos-Renyi_ef_9_v_12.edgelist -g Erdos -od ../../Output -s square -root 3 -m 90000 -hx 46 -hy 46 -hdepth 0 -hb 0 -route 0 -iter 5 -verify`

## Using clang-tidy
Compile with: `CC=gcc-13 CXX=g++-13 cmake -S . -B build -D THROTTLE=true -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`

Inside an application use something like:
> `$ noglob /opt/homebrew/opt/llvm/bin/clang-tidy -p build ../../Source/ComputeCell.cpp -checks=cppcoreguidelines-* -header-filter=.*`

> `$ noglob python3 /opt/homebrew/opt/llvm/bin/run-clang-tidy -p build -header-filter='.*' -checks='-*,modernize-use-nullptr'`

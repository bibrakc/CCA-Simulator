# CCA-Simulator
Event-level simulator for [Continuum Computer Architecture (CCA)](https://superfri.org/index.php/superfri/article/view/188) class of designs.

## Summary
The CCA Simulator enables exploring design space of the CCA class of non-Von Neumann intelligent memory systems. These systems are concieved to be highly fine-grain parallel and use event-driven mechanisms to perform computation.

## Building
### Using CMake
To generate the executable `SSSP_CCASimulator`:

> `$ CC=gcc-13 CXX=g++-13 cmake -S . -B build -D THROTTLE=true`

> `$ cmake --build build`

- `THROTTLE=true/false`: for enabling throttle of diffusion to mitigate congestion.

## Executing
### SSSP Application

#### Using Low-Latency Network (Htree)
> `$ ./build/Applications/Single_Source_Shortest_Path/SSSP_CCASimulator -f ./Generated_Graphs/Erdos-Renyi_ef_9_v_12.edgelist -g Erdos -od ./Output -s square -root 0 -tv 35 -m 90000 -hx 3 -hy 3 -hdepth 4 -hb 256 -route 0`

#### Using Pure Mesh Netowrk
> `$ ./build/Applications/Single_Source_Shortest_Path/SSSP_CCASimulator -f ./Generated_Graphs/Erdos-Renyi_ef_8_v_14.edgelist -g Erdos -od ./Output -s square -root 0 -tv 35 -m 90000 -hx 96 -hy 96 -hdepth 0 -hb 0 -route 0`

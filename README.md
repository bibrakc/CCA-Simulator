# CCA-Simulator
Event-level simulator for [Continuum Computer Architecture (CCA)](https://superfri.org/index.php/superfri/article/view/188) class of designs.

## Summary
The CCA Simulator enables exploring design space of the CCA class of non-Von Neumann intelligent memory systems. These systems are concieved to be highly fine-grain parallel and use event-driven mechanisms to perform computation.

## Building
### Using CMake
To generate the executable `SSSP_CCASimulator`:

- `$ CC=gcc-12 CXX=g++-12 cmake -S . -B build -D DEBUG_CODE=false` (or `DEBUG_CODE=true` for outputing debuging information)
- `$ cmake --build build`

## Executing
### SSSP Application
`$ ./build/Applications/Single_Source_Shortest_Path/SSSP_CCASimulator -f ./Generated_Graphs/Erdos-Renyi_ef_4_v_8.edgelist -g Erdos -od ./Output -s square -root 0 -tv 35 -m 9000 -hx 9 -hy 11 -hdepth 2`

`$ ./build/Applications/Single_Source_Shortest_Path/SSSP_CCASimulator -f ./Generated_Graphs/Erdos-Renyi_ef_4_v_8.edgelist -g Erdos -od ./Output -s square -root 0 -tv 35 -m 9000 -hx 5 -hy 7 -hdepth 1`

`$ ./build/Applications/Single_Source_Shortest_Path/SSSP_CCASimulator -f ./Generated_Graphs/Erdos-Renyi_ef_9_v_12.edgelist -g Erdos -od ./Output -s square -root 0 -tv 35 -m 90000 -hx 3 -hy 3 -hdepth 4 -hb 256 -route 0`

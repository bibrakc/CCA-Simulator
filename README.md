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
`$ ./build/Applications/Single_Source_Shortest_Path/SSSP_CCASimulator -f ./Input_Graphs/Erdos-Renyi_ef_5_v_7.edgelist  -s square -dx 10 -dy 10 -root 0 -tv 35`

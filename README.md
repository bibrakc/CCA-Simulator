# CCA-Simulator
Event level simulator for Continuum Computer Architecture (CCA) class of designs

## Summary
The simulator enables exploring the design space of CCA class of non-Von Neumann intelligent memory systems that use event-driven mechanism to perform computations.

## Building
### Using CMake
To generate the executable `CCASimulator`:

- `$ cmake -S . -B build -D DEBUG_CODE=false` (or `DEBUG_CODE=true` for outputing debuging information)
- `$ cmake --build build`

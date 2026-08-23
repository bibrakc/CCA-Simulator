# CCA-Simulator

Simulator for the Continuum Computer Architecture (CCA) ([paper1](https://superfri.org/index.php/superfri/article/view/188), [paper2](https://doi.org/10.1016/j.future.2026.108394)) class of designs.

## Summary

The CCA-Simulator enables exploring the design space of non-Von Neumann intelligent memory systems based on the Continuum Computer Architecture. CCA systems are highly fine-grain parallel and use event-driven mechanisms to perform computation.

The simulator supports the design and deployment of asynchronous message-driven graph computations. It is high-level enough to be programmed using the diffusive programming model, yet low-level enough to simulate individual operon (message) movements between Compute Cells (CCs).

**Simulation model:** In a single cycle, a message traverses one hop between neighboring CCs. CCA channel links are 256-bit wide, allowing small operons to be sent in a single flit cycle. Simultaneously, a CC can perform one of two operations:
1. A computing instruction (predicate resolution + work in the user application action), or
2. The creation and staging of a new operon.

This means BFS and SSSP actions take 2–3 cycles of compute, Page Rank actions take 3–70 cycles, and diffusions take cycles proportional to the local edge-list size.

## Demo of Message-Driven Execution

### Static Graph BFS
<img src="Analytics/Animations/BFS_32x32_v_1024_e_10240_th_ON_SH_ON.gif" alt="Animation" width="600"/>

### Dynamic Graph BFS
Streaming dynamic graph processing on CCA is documented in [paper3](https://doi.org/10.1145/3677333.3678146).

<img src="Analytics/Animations/Streaming_Dynamic_BFS_32x32_v_1000_egdeSample_SH_ON.gif" alt="Animation" width="600"/>

### Legend
<img src="Analytics/Animations/Legend_Animation.png" alt="Legend" width="600"/>

## Building

Requires a C++20 compiler with OpenMP support (e.g., GCC 13+) and CMake 3.20+.

```bash
CC=gcc-13 CXX=g++-13 cmake -S . -B build -DTHROTTLE=true
cmake --build build -j$(nproc)
```

All executables are placed in `build/`. See the [Applications](/Applications/) directory for per-application details and configuration options.

## Graph Applications

The simulator includes the following asynchronous message-driven graph applications:

- **Breadth-First Search (BFS)** — with Rhizome variant for skewed graphs
- **Single-Source Shortest Path (SSSP)** — with Rhizome variant
- **Page Rank (Fixed Iterations)** — with Rhizome variant
- **Dynamic BFS** — incremental edge insertions with BFS recomputation
- **Streaming Dynamic BFS** — edges streamed into the CCA chip at runtime

## Tests

```bash
cd Tests/Run_All_Apps && zsh run_all_apps.zsh
```

Builds all applications and runs verification against reference solutions. Reports pass/fail status and elapsed time.

## CCA Language and Compiler

The repository includes a Scheme/Racket-style language for expressing graph algorithms that compiles directly to C++ application code for the simulator. See [`Language/README.md`](Language/README.md) for details.

```bash
# Compile a CCA program to C++
racket Language/main.rkt --compile --output /tmp/generated Language/examples/bfs/bfs.cca

# Build the generated application
cmake -S . -B build -DCCA_GENERATED_APPLICATIONS_DIR=/tmp/generated
cmake --build build --target BFS_Generated_CCASimulator -j$(nproc)
```

Requires Racket 9.3+ (`brew install minimal-racket`).

## Publications

- B. Qamar Chandio, M. Brodowicz, T. Sterling, "A message-driven system for processing highly skewed graphs," *Future Generation Computer Systems*, vol. 180, 2026. [DOI](https://doi.org/10.1016/j.future.2026.108394)

- B. Q. Chandio, M. Brodowicz, T. Sterling, "Structures and Techniques for Streaming Dynamic Graph Processing on Decentralized Message-Driven Systems," *ICPP Workshops '24*, 2024. [DOI](https://doi.org/10.1145/3677333.3678146)

- B. Q. Chandio, M. Brodowicz, T. Sterling, "Exploring the Design Space for Message-Driven Systems for Dynamic Graph Processing Using CCA," *Parallel Processing and Applied Mathematics (PPAM)*, Springer, 2025. [DOI](https://doi.org/10.1007/978-3-031-85697-6_6)

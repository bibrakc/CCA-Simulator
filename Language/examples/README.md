# CCA Language Examples

Each example has two files:

- **`*-kernel.cca`** — the algorithm (vertex type, constants, and action definition)
- **`*-driver.cca`** — the host program (simulator setup, graph loading, execution, verification)

The driver `require`s the kernel and compiles into a standalone C++ application.

## Compiling and Running

All commands are run from the CCA-Simulator root directory.

### BFS (Breadth-First Search)

```bash
# 1. Compile CCA source to C++
racket -e '(require "Language/compiler/pipeline.rkt") \
  (run-pipeline "Language/examples/bfs/bfs-driver.cca" \
                #:output "Language/generated-output")'

# 2. Build the generated application
CC=gcc-13 CXX=g++-13 cmake -S . -B build \
  -DCCA_GENERATED_APPLICATIONS_DIR="$PWD/Language/generated-output" \
  -DTHROTTLE=true -DANIMATION=false

cmake --build build --target BFS_Generated_CCASimulator -j 10

# 3. Run with verification
OMP_NUM_THREADS=10 ./build/BFS_Generated_CCASimulator \
  -f Input_Graphs/Erdos-Renyi_ef_5_v_6.edgelist \
  -g Erdos -s square -root 0 -verify
```

### SSSP (Single-Source Shortest Path)

```bash
# 1. Compile CCA source to C++
racket -e '(require "Language/compiler/pipeline.rkt") \
  (run-pipeline "Language/examples/sssp/sssp-driver.cca" \
                #:output "Language/generated-output")'

# 2. Build the generated application
CC=gcc-13 CXX=g++-13 cmake -S . -B build \
  -DCCA_GENERATED_APPLICATIONS_DIR="$PWD/Language/generated-output" \
  -DTHROTTLE=true -DANIMATION=false

cmake --build build --target SSSP_Generated_CCASimulator -j 10

# 3. Run with verification
OMP_NUM_THREADS=10 ./build/SSSP_Generated_CCASimulator \
  -f Input_Graphs/Erdos-Renyi_ef_5_v_6.edgelist \
  -g Erdos -s square -root 0 -verify
```

## Using a Custom Cost Model

```bash
racket -e '(require "Language/compiler/pipeline.rkt") \
  (run-pipeline "Language/examples/bfs/bfs-driver.cca" \
                #:output "Language/generated-output" \
                #:cost-model-path "Language/cost-models/memory-heavy-cost-model.rkt")'
```

## Program Structure

```
examples/
├── bfs/
│   ├── bfs-kernel.cca    # BFS algorithm: BFSVertex + bfs-action
│   └── bfs-driver.cca    # Host: create-simulator, load-graph, germinate, run, verify
└── sssp/
    ├── sssp-kernel.cca   # SSSP algorithm: SSSPVertex + sssp-action
    └── sssp-driver.cca   # Host: same structure, uses edge weights
```

## Available CLI Arguments (generated)

Run with `--help` to see all options:

```bash
./build/BFS_Generated_CCASimulator --help
```

Key arguments:
- `-f` — path to the input graph file (required)
- `-g` — graph name for output file naming (required)
- `-s` — compute cell shape: "square" (required)
- `-root` — root vertex ID (required)
- `-verify` — enable verification against reference file
- `-m` — memory per compute cell (default: 512 KB)
- `-hx`, `-hy` — chip dimensions (default: 3×5)
- `-mesh` — mesh type: 0=Regular, 1=Torus
- `-shuffle` — randomize vertex placement
- `-od` — output directory for results
- `-trail` — trail number for experiments

# CCA Language and Compiler

A Scheme/Racket-style language for expressing diffusive graph algorithms on the
Continuum Computer Architecture. The compiler generates C++ application code that
links directly against the CCA-Simulator.

## Requirements

- Racket v9.3+ (`brew install minimal-racket`)
- rackunit (`raco pkg install --auto rackunit-lib`)

## Quick start

```bash
# Check a program (parse, resolve, type-check)
racket Language/main.rkt --check Language/examples/bfs/bfs.cca

# Compile to generated C++
racket Language/main.rkt --compile --output /tmp/cca-generated \
    Language/examples/bfs/bfs.cca

# Compile with a custom cost model
racket Language/main.rkt --compile --output /tmp/cca-generated \
    --cost-model Language/cost-models/memory-heavy-cost-model.rkt \
    Language/examples/bfs/bfs.cca

# Build the generated application against CCA-Simulator
cmake -S . -B build -DCCA_GENERATED_APPLICATIONS_DIR=/tmp/cca-generated
cmake --build build --target BFS_Generated_CCASimulator -j 10

# Run with verification
./build/BFS_Generated_CCASimulator \
    -f Generated_Graphs/Erdos-Renyi_ef_4_v_5.edgelist \
    -g Erdos -s square -root 0 -verify

# Run tests
racket Language/tests/run-tests.rkt
```

## Features

### Source language
- Scheme/Racket S-expression syntax with `#lang cca`
- `define-constant`, `define-vertex`, `define-action`, `define-application`
- Four-phase action model: predicate, work, diffuse-predicate, diffuse
- Typed fields, parameters, and expressions (`UInt32`, `Boolean`, `Address`)
- Edge iteration with `for-each` and `propagate`
- Paper-compatible syntax sugar (nested predicate/begin/diffuse)

### Compiler passes
- **Reader** — strips `#lang cca`, reads S-expressions as data
- **Parser** — builds typed AST, splits action into four canonical phases
- **Name resolution** — symbol table, C++ name mangling, propagate validation
- **Type checker** — expression types, field access, payload constraints
- **Effect checker** — predicates are pure, work cannot propagate, diffuse cannot
  mutate, ghost-safety of diffuse expressions
- **C++ IR** — structured representation of generated declarations, functions,
  statements, and expressions with a pretty-printer
- **Code generation** — vertex struct, payload struct, four templated handlers with
  RPVO/ghost bypass, `INVOKE_HANDLER_3` dispatch, host main with CLI, graph
  loading, event registration, termination, germination, verification, and CMake

### Cost model
The compiler counts AST operations per action phase and generates accurate
`cc.apply_CPI(N)` calls. The default cost model assigns:

| Operation | Cycles |
|---|---|
| field-read | 1 |
| field-write | 1 |
| comparison | 1 |
| arithmetic | 1 |
| boolean-op | 1 |
| literal/variable access | 0 |
| edge-read | 1 |
| propagate (network) | 0 |

Users can supply a custom cost model file via `--cost-model` to explore different
architectural assumptions without modifying the source program.

### Safety checks
- Payload parameters must be trivially-copyable scalar types
- `propagate` in a work phase is rejected
- Vertex field mutation in a diffuse phase is rejected
- Non-Boolean predicate expressions are rejected
- Type mismatches in field assignments are rejected
- Ghost-unsafe expressions in diffuse propagation arguments are rejected

## Status

| Milestone | Status |
|---|---|
| 1. Reader, parser, canonical AST | ✓ |
| 2. Name resolution, types, effects | ✓ |
| 3. C++ IR and code generation | ✓ |
| 4. Cost model | ✓ |
| 5. Host and CMake generation | ✓ |
| 6. End-to-end BFS conformance | ✓ |
| 7. SSSP | — |
| 8. PageRank | — |
| 9. Dynamic graphs | — |

## Architecture

```
Language/
├── main.rkt                       CLI entry point
├── compiler/
│   ├── pipeline.rkt               Pass orchestration
│   ├── ast.rkt                    AST data types
│   ├── cost-model.rkt             Operation cost model and CPI computation
│   ├── frontend/
│   │   ├── read-source.rkt        S-expression reader
│   │   ├── parse.rkt              Parser (builds canonical four-phase AST)
│   │   ├── resolve.rkt            Name resolution and symbol table
│   │   └── typecheck.rkt          Type and effect checker
│   └── backend/
│       ├── emit-cpp.rkt           C++ code generation orchestration
│       └── cpp/
│           └── cpp-ir.rkt         C++ IR structs and pretty-printer
├── cost-models/                   Configurable cost model files
├── examples/bfs/bfs.cca           Reference BFS program
├── generated-output/              Compiler output (gitignored)
└── tests/
    └── run-tests.rkt              19 passing tests
```

## Generated output

For a 27-line BFS source program, the compiler generates ~250 lines of C++ that:
- Builds against CCA-Simulator with no modifications to existing code
- Correctly implements the four-phase action pipeline
- Handles RPVO ghost vertices automatically
- Uses computed CPI values based on the cost model
- Passes BFS verification on real graphs (tested up to 16K vertices / 425K edges)

The generated application integrates via an optional CMake cache variable:
```cmake
cmake -DCCA_GENERATED_APPLICATIONS_DIR=/path/to/generated ...
```

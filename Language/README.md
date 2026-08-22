# CCA Language and Compiler

A Scheme/Racket-style language for expressing diffusive graph algorithms on the
Continuum Computer Architecture. The compiler generates C++ application code that
links directly against the CCA-Simulator.

## Requirements

- Racket v9.3+ (install via `brew install minimal-racket`)
- rackunit (`raco pkg install --auto rackunit-lib`)

## Quick start

```bash
# Check a program (parse, type-check, effect-check)
racket Language/main.rkt --check Language/examples/bfs/bfs.cca

# Compile to generated C++
racket Language/main.rkt --compile --output build/generated-applications/BFS \
    Language/examples/bfs/bfs.cca

# Run tests
racket Language/tests/run-tests.rkt
```

## Status

| Milestone | Status |
|---|---|
| 0. BFS fixtures and ABI snapshot | ◐ |
| 1. Reader, parser, canonical AST | ✓ |
| 2. Name resolution, types, effects | — |
| 3. Semantic action lowering | — |
| 4. C++ kernel generation | — |
| 5. Host and CMake generation | — |
| 6. End-to-end BFS conformance | — |
| 7. Packaging | — |

## Architecture

```
Language/
├── main.rkt                 CLI entry point
├── compiler/
│   ├── pipeline.rkt         Pass orchestration
│   ├── ast.rkt              AST data types
│   ├── frontend/            Reader, parser, type checker
│   ├── passes/              Action lowering, RPVO lowering
│   └── backend/             C++ code generation
├── examples/bfs/bfs.cca     Reference BFS program
└── tests/                   Compiler test suite
```

The compiler target is the same C++ interface used by hand-written applications
under `Applications/`. Generated output builds through `cca_add_application`
without modifying existing code.

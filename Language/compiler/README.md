# Compiler

The CCA-to-C++ compiler implementation in Racket.

## Structure

```
compiler/
├── pipeline.rkt         Pass orchestration and CLI integration
├── ast.rkt              AST data types (expressions, statements, declarations)
├── cost-model.rkt       Operation cost model and CPI computation
├── frontend/            Analysis passes (no code generation)
│   ├── read-source.rkt  S-expression reader
│   ├── parse.rkt        Parser → canonical four-phase AST
│   ├── resolve.rkt      Name resolution and symbol table
│   └── typecheck.rkt    Type and effect checker
├── passes/              Explicit lowering passes (future)
│   └── README.md
└── backend/             C++ code generation
    ├── emit-cpp.rkt     Orchestrates .hpp, .cpp, and CMake emission
    └── cpp/
        └── cpp-ir.rkt   C++ IR structs and pretty-printer
```

## Pass pipeline

```
source file (.cca)
    → read-source    (file → list of S-expressions)
    → parse          (S-expressions → canonical AST)
    → resolve        (AST → AST with symbol table)
    → typecheck      (types + effects + ghost-safety)
    → emit-cpp       (AST + cost model → .hpp + .cpp + CMakeLists.txt)
```

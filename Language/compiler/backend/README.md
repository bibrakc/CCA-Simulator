# Backend

C++ code generation targeting CCA-Simulator's application interface.

## Structure

| File | Role |
|---|---|
| `emit-cpp.rkt` | Main entry: generates .hpp, .cpp, and CMakeLists.txt |
| `cpp/cpp-ir.rkt` | C++ IR data structures and pretty-printer |

## What gets generated

- Vertex struct (template deriving from `RecursiveParallelVertex`)
- Payload struct (trivially copyable, only source action parameters)
- Four templated handler functions per action (predicate, work, diffuse-pred, diffuse)
- Ghost/RPVO bypass logic (auto-generated, never user-written)
- `INVOKE_HANDLER_3` dispatch wrappers
- Host `main()` with CLI, simulator setup, graph loading, verification
- `CMakeLists.txt` using `cca_add_application`

## Cost model integration

`apply_CPI(N)` values are computed from the AST using the active cost model,
not hardcoded. See `compiler/cost-model.rkt`.

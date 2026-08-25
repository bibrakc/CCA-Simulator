# CCA Compiler Design

**Status:** Draft 0.2  
**Source language:** Scheme/Racket-style CCA DSL  
**Implementation:** Racket 9.3+  
**Target:** C++ application code for CCA-Simulator

## 1. Overview

The CCA compiler translates programs written in a Scheme/Racket-style domain-specific
language into C++ application code that links against CCA-Simulator. The compiler's
value is domain-specific lowering: typed diffusive actions become the exact C++ event
handlers, RPVO ghost-vertex plumbing, and host orchestration expected by the simulator.

The compiler does **not** target x86, LLVM IR, or a standalone runtime. A standard
C++ compiler (GCC 13+) handles machine-code generation from the emitted source.

## 2. Compilation pipeline

```
source (.cca)
    → read-source       File → S-expressions (resolves require imports)
    → parse             S-expressions → canonical AST (four-phase actions)
    → resolve           AST → resolved program with symbol table
    → typecheck         Type checking + effect checking + ghost-safety
    → emit-cpp          AST + cost model → .hpp + .cpp + CMakeLists.txt
```

### 2.1 Reader (`frontend/read-source.rkt`)

- Strips `#lang cca` header
- Reads all top-level forms as pure data (no `eval`)
- Resolves `(require "path.cca")` by recursively reading and inlining the referenced file's forms

### 2.2 Parser (`frontend/parse.rkt`)

- Recognizes `define-constant`, `define-vertex`, `define-action`, `define-program`
- Splits action bodies into the canonical four-phase structure: predicate, work, diffuse-predicate, diffuse
- Handles paper-compatible sugar (nested `begin`/`diffuse` forms)
- Parses `define-program` host-level forms with `let` bindings for CLI arguments
- Produces a `cca-program` AST struct

### 2.3 Name resolution (`frontend/resolve.rkt`)

- Builds a symbol table mapping source names to kind, type, and C++ mangled name
- Validates propagate targets reference declared actions
- Validates application metadata references (vertex-type, root-action, result-field)
- Mangles identifiers: `-` → `_`, `!` → removed, `?` → `_p_0x3f`, C++ keywords prefixed with `cca_`

### 2.4 Type and effect checker (`frontend/typecheck.rkt`)

- Verifies expression type consistency (arithmetic on UInt32, comparisons return Boolean)
- Verifies field access matches declared types
- Enforces payload parameters are trivially-copyable scalar types
- Phase effect enforcement:
  - Predicates must be pure (Boolean, no mutation, no propagate)
  - Work may mutate vertex fields but must NOT propagate
  - Diffuse may propagate but must NOT mutate vertex fields
- Ghost-safety: diffuse propagation arguments must not read vertex fields

### 2.5 Code generation (`backend/emit-cpp.rkt`)

Generates three files:

- **`.hpp`** — vertex struct, payload struct, four templated handlers per action,
  INVOKE_HANDLER_3 dispatch wrappers, event declarations
- **`.cpp`** — `main()` generated from host-level forms: CLI parsing, simulator
  construction, graph loading, event registration, germination, execution,
  verification, and results output
- **`CMakeLists.txt`** — `cca_add_application(NAME ... SOURCE ...)`

Uses a C++ IR layer (`backend/cpp/cpp-ir.rkt`) with structs for declarations,
functions, and expressions, plus a pretty-printer.

## 3. Cost model (`cost-model.rkt`)

The compiler counts AST operations per action phase and generates accurate
`cc.apply_CPI(N)` calls. Each operation has a configurable cycle cost:

| Operation | Default cost |
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

## 4. Generated C++ contract

The emitted code must satisfy:

- Handler signature: `Closure(ComputeCell&, Address, ActionArgumentType)` for templates
- ABI wrapper: `Closure(ComputeCell&, Address, actionType, ActionArgumentType)` for `handler_func`
- Dispatch via `INVOKE_HANDLER_3` macro for `edges_min`/`edges_max` RPVO types
- Payload structs must be trivially copyable (`cca_create_action_argument`/`cca_get_action_argument`)
- `Action` constructor: 9 fields (dest, origin, type, ready, args, pred, work, dpred, diffuse)
- Ghost vertices: predicate=true, work=noop, diffuse_pred=true, diffuse reads from payload
- Ghost fan-out: forwards unchanged payload through RPVO children before processing local edges
- Host uses `cca_add_application` CMake function for build integration

## 5. Host-level code generation

The driver file's `define-program` host forms translate directly to simulator API calls:

| Source form | Generated C++ |
|---|---|
| `(create-simulator ...)` | `CCASimulator` constructor with shape validation |
| `(load-graph ...)` | `Graph<V<SimpleVertex<>>>` constructor + `CyclicMemoryAllocator` |
| `(register-actions ...)` | Four `register_function_event` calls per action |
| `(germinate ...)` | `transfer_graph_host_to_cca` + payload pack + `germinate_action` |
| `(run)` | `run_simulation` with timing |
| `(when v? (verify ...))` | Verification against reference file |
| `(write-results ...)` | Statistics output + animation data |

CLI arguments declared via `(let ([name (cli-arg ...)] ...) ...)` generate
`parser.set_required`/`set_optional` with long names and descriptions, followed
by `parser.get<>` calls using the let-binding name as the C++ variable.

## 6. Testing

- **19 unit tests** covering reader, parser, type/effect checker, and negative cases
- **Compile integration test** in CI: compile `bfs-driver.cca`, build against simulator, run with verification
- **Correctness criterion**: generated application must pass `.bfs`/`.sssp` verification on reference graphs
- **No cycle-count regression**: simulated cycles are research-sensitive and not compared

## 7. Future work

### Near-term
- **PageRank** — floating-point types, iteration, accumulation, multiple vertex fields
- **Explicit lowering passes** (`compiler/passes/`) — extract action-splitting, payload synthesis, and RPVO lowering from the emitter into independent testable passes
- **Source spans** — attach source locations to AST nodes for precise error reporting

### Medium-term
- **Dynamic graphs** — edge insertion actions, remote allocation, futures, continuations
- **Multiple actions per germination** — support programs that germinate different actions
- **General helper functions** — `define` for shared logic across actions

### Long-term
- **Rhizomes** — `#:rhizome-shared` annotation, rhizome-link propagation, AND-gate LCO collapse
- **Actions without all four phases** — optional work, optional diffuse
- **Host-level forms as first-class language** — full programmability replacing metadata

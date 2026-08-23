# CCA Language Compiler — Design and Implementation Plan

**Date:** 2026-08-22  
**Status:** BFS-first implementation plan  
**Source language:** Scheme/Racket-style CCA DSL  
**Compiler implementation:** Racket  
**Target:** C++ application kernels accepted directly by `CCA-Simulator`

## 1. Goal

Build a compiler that turns a CCA language program into the same category of C++ application currently written by hand in CCA-Simulator. The first conformance target is static breadth-first search (BFS): compile `bfs.cca` into a generated vertex type, action payload, four event handlers, host setup, verification support, and CMake metadata; build it with the simulator; and obtain the same BFS levels as the hand-written `BFS_CCASimulator`.

This compiler does **not** target x86, LLVM IR, pthreads, or a standalone CCA runtime. A normal C++ compiler remains responsible for machine-code generation. The CCA compiler's value is domain-specific lowering: typed diffusive actions become the exact C++ event and RPVO plumbing expected by the simulator.

`language-specification.md` is the normative Draft 0.1 definition of BFS syntax, types, phase effects, and semantics. This document explains how to implement it.

## 2. Evidence and constraints

### 2.1 Paper and pseudocode model

The source material establishes these concepts:

- Scheme/Racket-style typed structures and actions;
- a BFS predicate `current-level > incoming-level`;
- work that writes the incoming level;
- propagation of `incoming-level + 1` over outgoing edges;
- an independently guarded, deferred `diffuse` closure;
- RPVO ghost vertices that distribute large outgoing adjacency lists;
- future/continuation-based remote allocation for dynamic graphs; and
- rhizomes and collapse operations for later highly skewed graph support.

Only the first five plus RPVO traversal are in BFS Draft 0.1. Futures, continuations, dynamic insertion, and rhizomes remain later milestones.

### 2.2 Current simulator ABI

The backend must conform to current repository interfaces rather than inventing a parallel runtime API:

- `handler_func` is a function pointer returning `Closure` and accepting
  `ComputeCell&`, `Address`, `actionType`, and `ActionArgumentType`;
- `ActionArgumentType` is `std::shared_ptr<char[]>`;
- payload helpers use `memcpy` and statically require a trivially-copyable C++ type;
- an `Action` has destination, origin, type, readiness, payload, predicate, work, diffuse-predicate, and diffuse fields;
- application functions are registered through `register_function_event`;
- RPVO type selection is performed by `INVOKE_HANDLER_3` based on `edges_min` or `edges_max`;
- graph transfer instantiates the application vertex over `ghost_type_level_1`;
- execution starts with `create_terminator`, `germinate_action`, and `run_simulation`; and
- applications are linked with `cca_add_application` from the top-level CMake build.

The hand-written files under `Applications/Breadth_First_Search/` are the initial executable oracle for code shape and runtime behavior.

### 2.3 Project constraints

- Preserve simulator behavior unless a separate, intentional simulator change is approved.
- Generated C++ must coexist with hand-written applications.
- Use clang-format major version 21 for generated C++ and CI consistency.
- Validate the generated BFS application, not all applications after every compiler edit.
- End-to-end runs on this machine should use `OMP_NUM_THREADS=10`.
- Compare correctness outputs, not simulated cycle counts; cycle counts are research-sensitive and are not regression criteria.

## 3. Lessons adopted from Little Parallel Language

The `Little-Parallel-Language` repository is a useful compiler-organization guide, not a backend template.

### 3.1 Reuse

1. **Small explicit passes.** Each pass has one input IR, one output IR, and one invariant to establish.
2. **Racket pattern matching.** Use `match` for syntax-directed parsing, checking, and lowering.
3. **Environment threading.** Follow the typechecker and `uniquify` pattern for declaration, type, and local-binding environments.
4. **Pass-level testing.** Adapt its pass-test organization using checked IR snapshots and structural invariants before C++ generation.
5. **Negative type tests.** Preserve the convention of source fixtures that intentionally fail with expected diagnostics.
6. **Pipeline visibility.** Make it possible to run or dump the program after any pass.

### 3.2 Replace or omit

- Replace `expose-parallelism` with CCA-specific action splitting and RPVO lowering.
- Omit flattening to three-address code, instruction selection, liveness, interference graphs, register allocation, conditional lowering, instruction patching, and x86 printing.
- Omit LPL's pthread runtime, thread IDs, per-thread heaps, and GC transformations.
- Do not use deeply nested pass calls. Use a named pass sequence with invariant checks and optional dumps.

### 3.3 Improvement over raw list IRs

The source remains S-expressions, but after parsing the compiler should use transparent Racket structs carrying source locations. This retains easy pattern matching while preventing malformed lists from silently crossing pass boundaries. Every IR node should contain or reference a `source-span` so diagnostics survive desugaring.

## 4. Repository layout

The language and compiler are a first-class subsystem inside the `CCA-Simulator` repository. `Language/` is a top-level sibling of `Include/`, `Source/`, `Applications/`, and `Tests/`; it is not placed inside the simulator test directory or an application directory.

```text
CCA-Simulator/
├── CMakeLists.txt
├── Include/                         # simulator C++ headers
├── Source/                          # simulator C++ implementation
├── Applications/                    # hand-written C++ applications
├── Tests/                           # existing simulator tests
│
├── Language/                        # CCA language and Racket compiler
│   ├── README.md
│   ├── info.rkt                     # Racket package metadata
│   ├── main.rkt                     # bootstrap compiler CLI
│   │
│   ├── docs/
│   │   ├── language-specification.md
│   │   ├── compiler-design.md
│   │   ├── runtime-interface.md
│   │   └── decisions/               # short architectural decision records
│   │
│   ├── cca/
│   │   └── lang/
│   │       └── reader.rkt           # implements #lang cca
│   │
│   ├── compiler/
│   │   ├── pipeline.rkt
│   │   ├── diagnostics.rkt
│   │   ├── source-span.rkt
│   │   ├── ast.rkt
│   │   ├── types.rkt
│   │   ├── environments.rkt
│   │   │
│   │   ├── frontend/
│   │   │   ├── read-source.rkt
│   │   │   ├── parse.rkt
│   │   │   ├── desugar.rkt
│   │   │   ├── resolve.rkt
│   │   │   ├── typecheck.rkt
│   │   │   └── check-effects.rkt
│   │   │
│   │   ├── passes/
│   │   │   ├── lower-actions.rkt
│   │   │   ├── synthesize-payloads.rkt
│   │   │   └── lower-rpvo.rkt
│   │   │
│   │   └── backend/
│   │       ├── cca-simulator/
│   │       │   ├── runtime-profile.rkt
│   │       │   ├── synthesize-host.rkt
│   │       │   └── lower-handlers.rkt
│   │       └── cpp/
│   │           ├── cpp-ir.rkt
│   │           ├── lower-cpp.rkt
│   │           ├── emit-cpp.rkt
│   │           └── emit-cmake.rkt
│   │
│   ├── examples/
│   │   └── bfs/
│   │       └── bfs.cca
│   │
│   └── tests/                       # language/compiler tests only
│       ├── unit/
│       ├── passes/
│       ├── negative/
│       ├── golden/
│       ├── integration/
│       └── run-tests.rkt
│
└── build/
    └── generated-applications/      # ignored generated C++ and CMake files
        └── BFS/
```

The repository copy under `Language/docs/` becomes authoritative once development starts. The current materials under `Tasks/cca-language-compiler/` remain external research and planning references; the PDFs, paper pseudocode, and cloned `Little-Parallel-Language` repository should not be copied wholesale into `CCA-Simulator`.

Generated C++ must remain under `build/generated-applications/` initially and must not overwrite hand-written applications. `Language/tests/` is only for compiler tests, while the existing top-level `Tests/` continues to test the simulator.

## 5. Compiler entry points

The bootstrap interface should be executable without installing a Racket package:

```bash
racket Language/main.rkt check Language/examples/bfs/bfs.cca
racket Language/main.rkt compile Language/examples/bfs/bfs.cca \
  --output build/generated-applications/BFS
racket Language/main.rkt dump-ir Language/examples/bfs/bfs.cca \
  --after lower-rpvo
```

After packaging, expose equivalent `raco cca` commands.

Required commands:

- `check`: read, parse, resolve, typecheck, and check effects without emitting files;
- `compile`: run the full pipeline and emit `.hpp`, `.cpp`, and CMake files;
- `dump-ir`: print a deterministic representation after a named pass; and
- `explain`: later show how one source action maps to generated handlers.

Useful options:

- `--output DIR` — required destination for generated artifacts;
- `--simulator-root DIR` — validate includes and optionally build against a checkout;
- `--keep-unformatted` — debugging only; normal output is formatted with clang-format 21;
- `--emit-source-comments` — add source spans to generated C++ comments; and
- `--install` — later copy into an approved generated-applications directory. It must never overwrite a hand-written application silently.

## 6. Intermediate representations

### 6.1 S0: source syntax

Input is a sequence of Racket syntax objects. Keep lexical locations and reject arbitrary Racket evaluation. CCA source is data parsed by the compiler, not trusted Racket code run with `eval`.

### 6.2 S1: core AST

After parsing and desugaring, every action uses one canonical shape:

```text
ActionDecl
  name
  TargetParam(name, Pointer(VertexType))
  PayloadParam*
  Predicate(expr)
  Work(stmt*)
  DiffusePredicate(expr)
  Diffuse(stmt*)
```

Representative Racket structs:

```racket
(struct program (constants vertex actions application span) #:transparent)
(struct vertex-decl (name fields span) #:transparent)
(struct field-decl (name type initial mutable? annotations span) #:transparent)
(struct action-decl (name target params predicate work diffuse-predicate diffuse span)
  #:transparent)
(struct application-decl
  (name binary-name vertex-type root-action root-arguments result-field verification span)
  #:transparent)
```

Expression and statement variants should be explicit structs such as `var-expr`, `literal-expr`, `primitive-expr`, `vertex-read-expr`, `set-field-stmt`, `for-edges-stmt`, and `propagate-stmt`.

**S1 invariant:** paper-compatible nested forms and numeric `eq?` have been normalized; no surface sugar remains.

### 6.3 S2: resolved and typed AST

Each identifier reference points to a symbol record with a stable internal ID. Each expression carries its inferred type and effect set. Action call sites refer directly to the resolved action declaration, not a source string.

```racket
(struct typed-expr (node type effects span) #:transparent)
(struct symbol-info (id source-name cpp-name kind type declaration-span) #:transparent)
```

**S2 invariant:** all names resolve; all types match; phase effects are legal; propagation is arity/type correct; root metadata is valid; and edge propagation expressions are ghost-safe.

### 6.4 A0: split-action IR

`lower-actions` turns each action into four phase records and a registration record:

```text
SplitAction
  name
  target-type
  payload-layout
  PredicatePhase
  WorkPhase
  DiffusePredicatePhase
  DiffusePhase
  EventBundle
```

The four phases still contain CCA expressions and statements, not C++ text. This IR makes the source-to-runtime transformation independently testable.

**A0 invariant:** every action has exactly four phases with legal phase-specific operations.

### 6.5 A1: RPVO-aware action IR

`lower-rpvo` makes generated behavior explicit:

- dispatch between `ghost_type_level_1` and `ghost_type_level_greater_than_1`;
- test `is_ghost_vertex`;
- return true on ghost predicates;
- skip work on ghosts;
- source diffuse values from payload on ghosts;
- fan out unchanged action payloads to populated ghost children; and
- process local real edges.

Do not represent this as duplicated source syntax. Use backend nodes such as `ghost-guard`, `dispatch-rpvo`, `forward-ghost-children`, and `for-local-edges` so C++ lowering is mechanical.

**A1 invariant:** no generated handler needs to infer logical RPVO behavior; all ghost/root paths are explicit.

### 6.6 H0: host-application IR

`synthesize-host` produces a backend-neutral model of:

- CLI configuration required by the BFS executable;
- simulator construction;
- host graph type and CCA graph type;
- cyclic root vertex allocation and graph transfer;
- four event registrations per action;
- root payload construction;
- terminator allocation;
- seed `Action` construction;
- simulation invocation;
- optional `.bfs` verification; and
- result writing.

Draft 0.1 may use a versioned BFS host profile rather than trying to express arbitrary host logic in the source language.

### 6.7 C0: C++ document IR

Do not build all output with ad hoc string concatenation. Define a small document/AST layer for declarations, functions, statements, expressions, includes, templates, and raw versioned ABI snippets. Semantic expressions should lower structurally; fixed simulator boilerplate may use tested templates.

**C0 invariant:** identifiers are legal C++, declarations are ordered, includes are complete, and every emitted event reference has one declaration and one definition.

## 7. Pass pipeline

Implement a list of pass descriptors:

```racket
(struct pass (name transform input-contract output-contract) #:transparent)

(define bfs-passes
  (list read-source
        parse-program
        desugar-paper-forms
        resolve-names
        typecheck-program
        check-phase-effects
        lower-actions
        synthesize-payloads
        lower-rpvo
        synthesize-host
        lower-to-cpp
        emit-artifacts))
```

The actual pipeline should thread a compilation context containing diagnostics, source table, runtime profile, options, and deterministic fresh-name state. Avoid global mutable compiler state.

### Pass 1 — `read-source`

- Read syntax objects without evaluating source code.
- Recognize or bootstrap the `#lang cca` line.
- Preserve source spans.
- Reject malformed reader input with Racket's original location.

### Pass 2 — `parse-program`

- Match top-level declaration shapes.
- Construct S1 nodes.
- Reject unknown top-level forms and duplicate singleton declarations.
- Do not perform type checking here.

### Pass 3 — `desugar-paper-forms`

- Convert nested paper predicates/begin/diffuse into explicit phases.
- Convert `eq?` on Draft 0.1 scalars to `=`.
- Normalize `Integer` to `UInt32` while retaining the source spelling for diagnostics.
- Optionally inline the narrowly defined paper `inform-neighbors` helper when compatibility support is enabled.

### Pass 4 — `resolve-names`

- Build separate namespaces for constants, vertex fields, actions, and locals.
- Allocate stable IDs and C++ names.
- Detect mangling collisions and C++ keyword conflicts.
- Resolve field accessors and propagated action names.

Unlike LPL's `gensym`-heavy `uniquify`, use deterministic IDs based on declaration order so output is reproducible across processes.

### Pass 5 — `typecheck-program`

- Type constants, initializers, expressions, field writes, edge access, and propagation.
- Enforce fixed-width Draft 0.1 numeric rules.
- Check action target and payload parameter restrictions.
- Check application root arguments and BFS result metadata.
- Attach inferred type information to each expression.

Use an explicit type representation, not strings:

```racket
(struct t-unit () #:transparent)
(struct t-bool () #:transparent)
(struct t-u32 () #:transparent)
(struct t-address () #:transparent)
(struct t-edge () #:transparent)
(struct t-pointer (vertex-id) #:transparent)
(struct t-vector (element) #:transparent)
```

### Pass 6 — `check-phase-effects`

- Infer `read-target`, `write-target`, and `propagate` effects.
- Require pure/read-only predicates.
- Restrict work to target mutation.
- Restrict diffuse to communication and local bindings/control flow.
- Run ghost-safety analysis for expressions executed on RPVO ghost nodes.

Keep this pass separate from type checking so diagnostics distinguish “wrong value type” from “legal value in the wrong execution phase.”

### Pass 7 — `lower-actions`

This is the central domain-specific pass, analogous in importance—but not mechanism—to LPL's `expose-parallelism`.

For BFS:

```text
predicate:          current_level > incoming_level
work:               current_level = incoming_level
diffuse predicate:  current_level == incoming_level
diffuse:            propagate BFS(edge.address, incoming_level + 1)
```

becomes four independently named phases plus one event bundle. Preserve phase source spans for generated comments and diagnostics.

### Pass 8 — `synthesize-payloads`

- Remove the contextual target parameter from serialized arguments.
- Lay out remaining fields in declaration order.
- Map each field to a C++ type.
- Generate pack/unpack operations.
- Emit a C++ trivial-copyability assertion even though the source checker already restricts types.
- Do not add `src_vertex_id` merely because the hand-written BFS uses it for debugging; add hidden provenance only when a backend feature needs it.

BFS payload:

```cpp
struct BFSActionArguments {
    u_int32_t incoming_level;
};
```

### Pass 9 — `lower-rpvo`

Generate the root and ghost paths described in Section 6.5. The source edge loop represents logical full adjacency; this pass realizes it over ghost children and local edge chunks.

For BFS, preserve incoming level for ghost forwarding and increment only for real graph edges. Ensure ghost paths never cast a ghost allocation to the application-derived vertex type before checking the base `is_ghost_vertex` flag.

### Pass 10 — `synthesize-host`

Use the application declaration plus BFS runtime profile to synthesize host setup. Keep parser flags and reporting consistent with the current BFS application where practical. The minimum required options are graph file, graph name, shape, root, verification, memory, topology/routing, shuffle, output directory, and trial metadata.

### Pass 11 — `lower-to-cpp`

Map typed operations to C++:

| CCA construct | Generated C++ |
|---|---|
| `UInt32` | `u_int32_t` |
| target field read | `v->field` |
| target field write | `v->field = value;` |
| action parameter read | unpacked payload field |
| `(vertex-id v)` | `v->id` on non-ghost path |
| `(edge-address e)` | `v->edges[i].edge` |
| `(edge-weight e)` | `v->edges[i].weight` |
| `propagate` | payload pack + `cc.diffuse(Action(...))` |
| successful outer predicate | `Closure(cc.null_true_event, nullptr)` |
| rejected outer/diffuse predicate | `Closure(cc.null_false_event, nullptr)` |
| completed work | `Closure(cc.null_true_event, nullptr)` so `ComputeCell` schedules the diffuse predicate |
| completed diffuse | `Closure(cc.null_false_event, nullptr)` because the phase chain is finished |

For non-ghost BFS diffusion, the successful diffuse predicate proves that the stored vertex level equals the payload's incoming level. Code generation may use either value to compute the outgoing level, but the ghost path must use the payload because a ghost allocation has no application level field.

Draft 0.1 should emit the same phase cost calls as the hand-written BFS (`cc.apply_CPI(1)` in predicate, work, and diffuse predicate) through the simulator runtime profile. A formal source-level cost model can be designed later. Correctness tests must not assert exact total cycles.

### Pass 12 — `emit-artifacts`

- Emit deterministic UTF-8 files.
- Include an “auto-generated; do not edit” header and compiler/spec version.
- Write only beneath the requested output directory.
- Format `.hpp` and `.cpp` with clang-format 21 unless disabled for debugging.
- Write files atomically using temporary files and rename.
- Avoid rewriting unchanged output to reduce rebuild churn.

## 8. Generated BFS artifact contract

For application `BFS`, emit:

```text
Generated/BFS/
├── cca_bfs_generated.hpp
├── cca_bfs_generated.cpp
└── CMakeLists.txt
```

### 8.1 Header

The header contains:

1. required simulator and graph includes;
2. `max_level` constant;
3. `BFSVertex<Vertex_T>` deriving from `Vertex_T`;
4. initialized application fields and constructors;
5. `configure_derived_class_LCOs()`;
6. payload struct and static assertion;
7. `extern CCAFunctionEvent` declarations;
8. four templated handler implementations;
9. four ABI wrapper functions using `INVOKE_HANDLER_3`;
10. parser/configuration declarations or inline definitions; and
11. BFS verification/result helpers.

Each ABI wrapper must match `handler_func`, including the unused `actionType` parameter:

```cpp
inline auto bfs_predicate_func(ComputeCell& cc,
                               const Address addr,
                               actionType,
                               const ActionArgumentType args) -> Closure
{
    INVOKE_HANDLER_3(bfs_predicate_T, cc, addr, args);
}
```

### 8.2 Translation unit

The `.cpp` file contains:

1. event variable definitions;
2. CLI parsing;
3. `CCASimulator` construction;
4. host `Graph<BFSVertex<SimpleVertex<host_edge_type, edges_min>>>` loading;
5. cyclic allocator setup;
6. transfer to `BFSVertex<ghost_type_level_1>`;
7. four event registrations;
8. root payload packing;
9. terminator creation with allocation failure handling;
10. nine-field germination `Action` construction;
11. `run_simulation`;
12. optional BFS verification; and
13. result output.

### 8.3 CMake

```cmake
cca_add_application(
    NAME BFS_Generated_CCASimulator
    SOURCE cca_bfs_generated.cpp
)
```

The generated source includes its generated header from the same directory and relies only on include/link settings supplied by `cca_add_application`.

## 9. Simulator build integration

Generated output should not overwrite `Applications/Breadth_First_Search`. Add one small, reviewed integration hook to CCA-Simulator:

```cmake
set(CCA_GENERATED_APPLICATIONS_DIR "" CACHE PATH
    "Optional directory containing generated CCA applications")

if(CCA_GENERATED_APPLICATIONS_DIR)
    add_subdirectory(
        "${CCA_GENERATED_APPLICATIONS_DIR}"
        "${CMAKE_BINARY_DIR}/generated-applications")
endif()
```

The compiler output root supplies a CMake manifest that adds its `BFS` child directory. Then build with:

```bash
CC=gcc-13 CXX=g++-13 cmake -S CCA-Simulator -B build-generated \
  -DCCA_GENERATED_APPLICATIONS_DIR="$PWD/generated"
cmake --build build-generated --target BFS_Generated_CCASimulator -j 10
```

This keeps generated artifacts external, makes simulator acceptance explicit, and avoids editing the top-level application list on every compiler run. An `--install` workflow can be added later if committed generated sources become desirable.

## 10. Testing strategy

### 10.1 Reader/parser tests

- one fixture per top-level form;
- paper-compatible and canonical action forms normalize identically;
- source spans survive parsing;
- malformed forms and duplicate declarations fail at the correct location.

### 10.2 Type and effect tests

Positive tests cover BFS field access, arithmetic, payload construction, edge iteration, and propagation. Negative tests cover:

- signed/out-of-range literals;
- non-Boolean predicates;
- mutation of immutable fields;
- `propagate` in work;
- mutation in diffuse;
- action arity/type mismatch;
- non-payload action parameters;
- C++ name collisions;
- invalid root arguments; and
- target-field reads in a ghost-executed propagation expression.

Store expected diagnostic codes rather than entire prose where possible, for example `CCA-TYPE-004` and `CCA-EFFECT-002`.

### 10.3 Pass contract tests

For each pass, use a compact input fixture and compare against a checked expected IR. Add contract predicates such as `typed-program?`, `split-actions?`, and `rpvo-lowered?` and run them after every pass in debug/test mode.

### 10.4 Generated behavior fixtures

Use tiny deterministic graph fixtures to validate behavior through the real target path:

1. compile the CCA source into generated C++;
2. build only the generated application target against CCA-Simulator;
3. run it with the fixture's graph, root, and verification file; and
4. require the simulator's application verification to pass.

Include path, cycle, disconnected, duplicate-edge, and competing-path graphs so action ordering and predicate pruning are exercised by the simulator itself. Draft 0.1 deliberately has no Racket interpreter or reference evaluator; generated C++ plus CCA-Simulator is the executable semantics used for behavioral validation.

### 10.5 C++ golden tests

Golden tests should focus on stable structural units:

- vertex and payload declarations;
- handler signatures;
- ghost guards;
- `INVOKE_HANDLER_3` wrappers;
- event registration;
- propagation `Action` construction; and
- generated CMake.

Format expected and actual snippets with clang-format 21 before comparison. Do not make the whole generated translation unit the only test; smaller goldens identify the broken lowering more precisely.

### 10.6 Compile integration test

Compile generated BFS against the current simulator with the top-level CMake path. This catches ABI drift that Racket tests cannot:

- wrong handler signatures;
- stale `Action` constructor shape;
- missing includes;
- incorrect template types;
- wrong RPVO macro use; and
- link failures for event variables.

### 10.7 Runtime differential test

For a small existing graph/root pair:

1. run hand-written `BFS_CCASimulator` with verification;
2. run `BFS_Generated_CCASimulator` with the same graph, root, and relevant simulator options;
3. require both `.bfs` verifications to pass; and
4. optionally extract and compare per-vertex level vectors.

Use `OMP_NUM_THREADS=10` on this Mac. Do not require equal total simulated cycles or wall-clock time.

### 10.8 CI layers

Keep CI cost controlled:

- every compiler change: Racket unit, negative, pass-contract, and golden tests;
- backend changes: generated C++ compile test plus one tiny simulator behavior fixture;
- integration branch/PR: end-to-end BFS verification and comparison with the hand-written application;
- no all-application simulator run for documentation-only or frontend-only edits unless shared simulator code changed.

## 11. Implementation milestones and acceptance criteria

### Milestone 0 — Freeze BFS fixtures and ABI snapshot

Tasks:

- copy the Section 9 BFS source into `examples/bfs.cca`;
- record the current handler signature, `Action` constructor fields, RPVO macro use, and CMake helper in runtime-profile tests;
- select one tiny graph/root pair already supported by `.bfs` verification; and
- record the exact compiler/spec version in generated headers.

Acceptance: fixtures are checked in and a test fails clearly if the runtime profile no longer matches the simulator source.

### Milestone 1 — Reader, parser, and canonical AST

Tasks:

- implement source spans and diagnostics;
- parse all Draft 0.1 declarations;
- desugar paper-compatible BFS into canonical phases;
- dump deterministic S1 IR; and
- add parser and malformed-input tests.

Acceptance: `check` can parse `bfs.cca` through S1 and reports precise syntax errors.

### Milestone 2 — Name resolution, types, and effects

Tasks:

- implement deterministic symbols and C++ mangling;
- implement Draft 0.1 types;
- type all BFS expressions and statements;
- check phase effects and ghost safety; and
- add positive/negative suites.

Acceptance: valid BFS reaches S2; each intentionally invalid BFS fixture fails with the intended diagnostic code and source location.

### Milestone 3 — Semantic action lowering

Tasks:

- split actions into four phases;
- synthesize payload layout;
- make RPVO behavior explicit;
- add checked IR fixtures and structural pass contracts; and
- define expected BFS levels for path, cycle, disconnected, duplicate-edge, and competing-path graph fixtures.

Acceptance: each BFS fixture lowers to valid split-action and RPVO-aware IR, with expected generated handler structure ready for C++ emission. Behavioral acceptance occurs through CCA-Simulator in Milestones 5 and 6.

### Milestone 4 — C++ kernel generation

Tasks:

- implement C++ document IR and pretty printer;
- emit vertex, payload, event declarations/definitions, four handlers, wrappers, and propagation;
- generate ghost handling and `INVOKE_HANDLER_3` dispatch;
- format with clang-format 21; and
- add structural golden tests.

Acceptance: generated kernel code compiles in a focused harness or simulator target with warnings enabled.

### Milestone 5 — Host and CMake generation

Tasks:

- generate the BFS host translation unit and verification support;
- emit the child and root generated CMake manifests;
- add the optional generated-applications CMake hook to the simulator in a separate reviewed change; and
- build only `BFS_Generated_CCASimulator` initially.

Acceptance: the generated executable links against the unmodified simulator libraries through `cca_add_application`.

### Milestone 6 — End-to-end BFS conformance

Tasks:

- germinate level 0 at the selected root;
- run generated BFS with `OMP_NUM_THREADS=10`;
- pass `.bfs` verification;
- compare final levels with the hand-written BFS; and
- document the exact reproduction command.

Acceptance: generated and hand-written applications both pass correctness verification on the selected fixture. No cycle-count equality is required.

### Milestone 7 — Harden and package

Tasks:

- add `raco cca` packaging;
- add atomic output and stale-file cleanup;
- add `dump-ir` and compiler version output;
- document simulator compatibility/version checks; and
- run sanitizer/build validation where useful for generated C++.

Acceptance: a new checkout can install dependencies, compile `bfs.cca`, build the generated target, and run the documented BFS verification without manual source edits.

## 12. Later language progression

Only begin these after Milestone 6 is stable:

1. **SSSP:** add weighted edge semantics and distance overflow policy; reuse the BFS four-phase pattern.
2. **PageRank:** add floating-point type(s), iteration/state coordination, accumulation, and potentially multiple actions.
3. **Dynamic graphs:** add insertion actions, remote allocation, futures, and compiler lowering of continuations described in the dynamic-graph paper.
4. **Rhizomes:** add rhizome-aware vertex representations, `#:rhizome-shared`, broadcasts/collapse, and AND-gate LCO integration.
5. **Optimization:** payload minimization, common-expression hoisting, predicate simplification, and generated code specialization—only with semantic differential tests.

Do not implement x86 lowering, register allocation, or a pthread runtime at any stage; C++ and CCA-Simulator remain the backend.

## 13. Key decisions

1. **The source is a restricted DSL, not arbitrary Racket.** Never `eval` source files.
2. **The C++ ABI is a versioned backend profile.** Simulator interface drift should fail profile tests rather than produce obscure C++ errors.
3. **The target parameter is contextual.** It maps to `Action::obj_addr`; only remaining parameters are payload fields.
4. **Phases are explicit in core IR.** Paper syntax is sugar; generated events are first-class compiler artifacts.
5. **RPVO is abstract in source and explicit in lowered IR.** Ghost behavior is generated, not written by users.
6. **Fixed-width types are intentional.** Draft 0.1 `Integer` means `UInt32` for predictable payload and C++ semantics.
7. **Generated output is isolated.** Never overwrite hand-written BFS without an explicit, reviewed install operation.
8. **Correctness is semantic.** Final BFS levels and build/API compatibility matter; generated text and cycle counts need not be identical.
9. **Compiler passes are deterministic.** Stable names and output are required for debugging and golden tests.
10. **BFS comes before generality.** Do not add futures, rhizomes, or user-defined host logic until generated BFS works end to end.

## 14. Risks and mitigations

| Risk | Mitigation |
|---|---|
| Simulator ABI changes during compiler development | Runtime-profile module plus compile integration test |
| Ghost vertices lack application fields | Explicit ghost-safety analysis and RPVO lowering |
| Deferred predicates are treated like ordinary sequential `if` | Separate action phases, inspect lowered IR, and run stale-diffusion fixtures in CCA-Simulator |
| Payload contains non-trivial C++ data | Restricted payload types plus generated `static_assert` |
| Generated host boilerplate dominates implementation | Versioned BFS host template backed by compile/runtime tests |
| Golden files become brittle | Compare formatted structural units; use simulator differential tests for behavior |
| Source names produce invalid/colliding C++ | Deterministic mangling and collision diagnostics |
| Compiler accidentally evaluates untrusted source | Read syntax as data; no `eval` or dynamic module loading |
| Scope expands before BFS works | Milestone gates and Draft 0.1 feature restrictions |
| Research routing changes alter cycle counts | Exclude cycle counts from correctness regression criteria |

## 15. Definition of done for the BFS-first compiler

The BFS-first compiler is complete when all of the following are demonstrated by automated commands:

- `bfs.cca` passes syntax, name, type, effect, and ghost-safety checking;
- paper-compatible BFS syntax normalizes to the same canonical action as the explicit phased syntax;
- pass contracts confirm valid typed, split-action, payload, and RPVO-aware IR;
- generated payloads are trivially copyable;
- generated handlers match `handler_func`, use `INVOKE_HANDLER_3`, and register four events;
- generated diffusion forwards unchanged payload through RPVO ghosts and level+1 over real edges;
- generated host code creates a terminator and germinates the root action;
- generated C++ builds through CCA-Simulator's `cca_add_application` path;
- `BFS_Generated_CCASimulator` passes `.bfs` verification on the selected graph/root; and
- the final levels match the hand-written BFS application without requiring equal cycle counts.

# CCA Language Specification

**Status:** Draft 0.2  
**Date:** 2026-08-23  
**Implementation language:** Racket  
**Compilation target:** C++ application code accepted directly by `CCA-Simulator`

## 1. Purpose and scope

CCA is a statically typed, Scheme/Racket-style language for diffusive graph programs on the Continuum Computer Architecture simulator. A CCA source program does not compile to x86 or to a new runtime. It compiles to the same application-level C++ interface used by the hand-written simulator applications.

Draft 0.2 supports BFS and SSSP, with a kernel/driver program structure, host-level forms that map 1:1 to simulator C++ API calls, a configurable cost model, and a module system via `require`. See `grammar.md` for the complete formal syntax.

The design is grounded in:

- the BFS and data-structure pseudocode in the research papers;
- the predicate, deferred diffusion, RPVO, future, and rhizome descriptions in the publications;
- the current simulator ABI and hand-written application implementations; and
- the pass-oriented Racket implementation style of `Little-Parallel-Language`.

## 2. Execution model

### 2.1 Vertices and actions

A graph vertex is an addressable simulator object. An action is an asynchronous message directed to one vertex. The first action parameter denotes the target vertex and is supplied by the runtime from `Action::obj_addr`; it is not serialized in the action payload. All remaining parameters are copied into a generated, trivially-copyable C++ payload struct.

An action has four logical phases:

1. **predicate** — read-only guard for early pruning;
2. **work** — computation and mutation of application state on the target vertex;
3. **diffuse predicate** — read-only guard evaluated when deferred diffusion is considered; and
4. **diffuse** — communication that propagates actions to other vertices.

This maps directly to the simulator's four `CCAFunctionEvent` fields in `Action`: `predicate`, `work`, `diffuse_predicate`, and `diffuse`.

Actions are fire-and-forget. A program must not assume a global action order or wait synchronously for a propagated action. BFS correctness comes from monotonic level improvement and predicate pruning, not from message ordering.

### 2.2 Logical adjacency and RPVO lowering

At source level, `(vertex-edges v)` denotes the complete logical outgoing adjacency of `v`. The programmer does not mention RPVO ghost vertices.

The backend realizes logical adjacency using the simulator's `RecursiveParallelVertex` hierarchy:

- the action is forwarded to each present ghost child with the payload unchanged;
- ghost vertices always pass the action predicate and diffuse predicate;
- ghost vertices perform no application work;
- each RPVO object propagates over its local edge chunk; and
- expressions used to construct edge-bound action arguments are evaluated from payload values, constants, and edge data available at that RPVO.

For BFS, an incoming `level` is forwarded unchanged through ghost links and `level + 1` is sent over each real graph edge. This behavior is compiler-generated.

### 2.3 Termination

The generated host program creates a simulator terminator, germinates the root action using the terminator address as the action origin, and calls `run_simulation`. Termination is a backend/runtime concern and is not explicit in a BFS kernel.

## 3. Source files and lexical syntax

A source file uses Racket datum syntax and begins with:

```racket
#lang cca
```

CCA uses parentheses and brackets interchangeably where Racket does. Line comments start with `;`. Identifiers follow Racket symbol syntax, subject to these restrictions:

- identifiers must not begin with `$`; that prefix is reserved for compiler-generated names;
- a declaration must not bind the same identifier twice in one scope;
- source names that mangle to the same C++ identifier are an error; and
- keywords beginning with `#:` are reserved in declaration option positions.

The backend mangles `-` to `_`, appends stable role suffixes such as `_predicate`, and escapes C++ keywords. Mangling must be deterministic so golden output and diagnostics are reproducible.

Integer literals in Draft 0.2 are decimal, unsigned, and must lie in `0..4294967295`. Boolean literals are `#t` and `#f`.

## 4. Program structure

See `grammar.md` for the complete formal grammar.

A CCA program consists of two files:

- **Kernel file** (`*-kernel.cca`) — declares the algorithm: constants, vertex type, and actions
- **Driver file** (`*-driver.cca`) — the entry point: requires the kernel, defines the host program using `define-program` with host-level forms

```text
program ::= #lang cca
            (require "kernel-file.cca")
            (define-program binary-name
              host-form ...)
```

Host-level forms map directly to simulator C++ API calls:
- `(create-simulator ...)` → `CCASimulator` constructor
- `(load-graph ...)` → `Graph` constructor + `CyclicMemoryAllocator`
- `(register-actions ...)` → `register_function_event` calls
- `(germinate ...)` → `germinate_action` with `Action` object
- `(run)` → `run_simulation`
- `(when verify? (verify ...))` → verification block
- `(write-results ...)` → statistics and animation output

CLI arguments are declared via `(let ([name (cli-arg ...)] ...) host-form)` bindings
which generate `parser.set_required`/`set_optional` and `parser.get<>` calls.

### 4.1 Paper-compatible action sugar

The paper pseudocode nests work and diffusion under the outer predicate:

```racket
(predicate condition
  (begin
    work-statement ...
    (diffuse
      (predicate diffuse-condition
        diffuse-statement ...))))
```

The reader/desugaring pass should accept this form and convert it to the canonical phased form above. The canonical form is used by all later compiler passes because it makes effects and simulator event boundaries explicit.

The paper's `(define name (lambda (...) body))` spelling may be added as compatibility sugar, but `define-action` is the normative Draft 0.2 declaration.

## 5. Types

### 5.1 Type forms

```text
type ::= Unit
       | Boolean
       | UInt32
       | Integer
       | Address
       | Edge
       | (Pointer vertex-name)
       | (Vector element-type)
```

`Integer` is a paper-compatibility alias for `UInt32` in Draft 0.2. It is not an arbitrary-precision Racket integer.

| CCA type | Meaning | C++ representation | Payload-serializable? |
|---|---|---|---|
| `Unit` | no useful value | `void`/statement context | no |
| `Boolean` | `#t` or `#f` | `bool` | yes |
| `UInt32` / `Integer` | unsigned 32-bit integer | `u_int32_t` | yes |
| `Address` | simulator object address | `Address` | yes, subject to C++ trivial-copy check |
| `Edge` | current adjacency entry | simulator edge element | no |
| `(Pointer V)` | typed reference to target vertex `V` | contextual `Address`, cast in handler | no in Draft 0.2 |
| `(Vector Edge)` | runtime-owned logical adjacency | RPVO traversal | no |

A **scalar type** is `Boolean`, `UInt32`, `Integer`, or `Address`. A **payload type** is a scalar type whose generated C++ representation is trivially copyable. A **storable type** in Draft 0.2 is `Boolean`, `UInt32`, or `Integer`.

The generated payload struct must satisfy `std::is_trivially_copyable_v<T>` because `cca_create_action_argument<T>` and `cca_get_action_argument<T>` serialize it with `memcpy`.

### 5.2 Built-in vertex and edge members

Every declared vertex implicitly inherits runtime-managed members. They are not repeated in `define-vertex`:

| Source operation | Type | Backend source |
|---|---|---|
| `(vertex-id v)` | `UInt32` | inherited `id` |
| `(vertex-edges v)` | `(Vector Edge)` | inherited local edges plus generated RPVO traversal |
| `(edge-address e)` | `Address` | `e.edge` |
| `(edge-weight e)` | `UInt32` | `e.weight` |

`edge-weight` is typed now so the type vocabulary remains stable, but BFS Draft 0.2 does not require weighted graph semantics.

Each field declared by `(define-vertex V [level ...])` receives a generated accessor `(vertex-level v)`. A mutable field also receives `(set-vertex-level! v value)`. Accessor names are derived from the field name, not the vertex type name.

### 5.3 Arithmetic and comparison

Draft 0.2 provides:

```text
(+ UInt32 UInt32 ...+) -> UInt32
(- UInt32 UInt32)      -> UInt32
(= T T ...+)           -> Boolean, for scalar T
(< UInt32 UInt32)      -> Boolean
(<= UInt32 UInt32)     -> Boolean
(> UInt32 UInt32)      -> Boolean
(>= UInt32 UInt32)     -> Boolean
(and Boolean ... )     -> Boolean
(or Boolean ... )      -> Boolean
(not Boolean)          -> Boolean
```

`eq?` on numeric values should be accepted as paper-compatibility sugar for `=` and normalized during desugaring.

`UInt32` arithmetic has C++ unsigned 32-bit semantics. The compiler should warn when a constant expression overflows. BFS programs should choose a sentinel larger than any possible graph distance and ensure `level + 1` cannot wrap for supported inputs.

## 6. Expressions and statements

### 6.1 Pure expressions

```text
expression ::= literal
             | name
             | (vertex-id expression)
             | (vertex-field expression)
             | (edge-address expression)
             | (edge-weight expression)
             | (operator expression ...)
             | (let ([name expression] ...) expression)
             | (if expression expression expression)
```

Expressions in predicates must be pure: they may read the target vertex, action parameters, constants, and local immutable bindings, but may not mutate state or propagate actions.

### 6.2 Work statements

```text
statement ::= (set-vertex-field! target expression)
            | (let ([name expression] ...) statement ...+)
            | (if expression (begin statement ...) (begin statement ...))
```

A work phase may mutate only mutable fields of its own target parameter. It must not mutate an edge, another vertex, runtime-owned RPVO state, or an action parameter. `propagate` is forbidden in work in Draft 0.2 so communication remains deferred and visible to the runtime.

### 6.3 Diffuse statements

```text
diffuse-statement ::= (for-each ([name : Edge] (vertex-edges target))
                        diffuse-statement ...+)
                    | (propagate action-name address-expression argument-expression ...)
                    | (let ([name expression] ...) diffuse-statement ...+)
                    | (if expression
                        (begin diffuse-statement ...)
                        (begin diffuse-statement ...))
```

`propagate` has type/effect:

```text
(propagate A destination arg ...) -> Unit ! {propagate}
```

The destination must have type `Address`. The argument count and types must match all parameters of `A` after its first target parameter.

For a logical edge traversal lowered across RPVO ghosts, propagated argument expressions must be **ghost-safe**: they may depend on action payload parameters, constants, local `let` bindings derived from them, and the current `Edge`. They must not read application fields from the target vertex because ghost objects intentionally do not contain those fields. Draft 0.2 reports a compile-time error for a non-ghost-safe expression. A later compiler may capture required field values into a generated diffusion closure payload.

## 7. Static semantics and effects

The checker maintains separate environments for constants, vertex fields, actions, local values, and phase effects. Its judgment can be read as:

```text
Γ ; phase ⊢ expression : type ! effects
```

Draft 0.2 effects are `read-target`, `write-target`, and `propagate`.

Required rules include:

1. The first action parameter must be `(Pointer V)` and is the target parameter.
2. Remaining action parameters must be payload types.
3. Outer and diffuse predicates must have type `Boolean` and no effects beyond `read-target`.
4. Work may have `write-target` but not `propagate`.
5. Diffuse may have `propagate` but not `write-target`.
6. A field write must match the declared field type and the field must be mutable.
7. Every action must have exactly one outer predicate, one work phase, one diffuse predicate, and one diffuse phase after desugaring.
8. Every source `propagate` must resolve to a declared action with matching arity and types.
9. Root arguments must match the root action payload parameters.
10. The result field must exist and have type `UInt32`/`Integer` for verification.
11. Draft 0.2 forbids recursion, first-class functions, mutation through arbitrary addresses, user allocation, futures, `call/cc`, and rhizome forms.

## 8. Dynamic semantics of an action

For an action `A(target, payload...)` arriving at address `a`:

1. The runtime resolves `a` to an RPVO object.
2. If the object is a non-ghost application vertex, evaluate the outer predicate.
3. If false, discard the action. If true, execute work.
4. The generated work handler must return `Closure(cc.null_true_event, nullptr)`. In the current simulator this return value tells `ComputeCell` to retain the action for its deferred diffuse predicate; a null/false event would suppress diffusion.
5. When diffusion is selected, evaluate its predicate against current state. If false, discard that diffusion.
6. If true, execute source-level logical propagation. The generated backend first forwards the unchanged payload through existing ghost children and then processes each local real edge. On a non-ghost BFS vertex, `incoming-level` and the stored `vertex-level` are equal whenever the diffuse predicate succeeds, so a conforming backend may use either value for `+ 1`; the payload value is required on ghosts because they have no application level field.
7. Each `propagate` creates an `Action` with `actionType::application_action`, the current address as `origin_addr`, `is_ready = true`, a serialized payload, and the destination action's four registered event IDs.

On a ghost RPVO object, steps 2–3 are replaced with “predicate true, work no-op,” the diffuse predicate is true, and diffusion operates from payload/edge values only.

A predicate may be evaluated later than the source program's textual order. Therefore a diffuse predicate must remain valid when re-evaluated against newer vertex state.

## 9. Complete BFS program

### Kernel (`bfs-kernel.cca`):

```racket
#lang cca

(define-constant max-level : UInt32 999999)

(define-vertex BFSVertex
  [level : UInt32 #:initial max-level #:mutable])

(define-action bfs-action
  ([v : (Pointer BFSVertex)]
   [incoming-level : UInt32])
  (predicate (> (vertex-level v) incoming-level)
    (work
      (set-vertex-level! v incoming-level))
    (diffuse
      (predicate (= (vertex-level v) incoming-level))
      (for-each ([e : Edge] (vertex-edges v))
        (propagate bfs-action
                   (edge-address e)
                   (+ incoming-level 1))))))
```

### Driver (`bfs-driver.cca`):

```racket
#lang cca

(require "bfs-kernel.cca")

(define-program "BFS_Generated_CCASimulator"

  (let ([shape (cli-arg "s" #:long-name "shape" #:type String #:required #t
                        #:description "Shape of the compute cell")]
        [hx (cli-arg "hx" #:long-name "htree_x" #:type UInt32 #:default 3
                     #:description "Rows of Cells served by a single end Htree node")]
        ...)
    (create-simulator
      #:shape shape #:dim-x hx #:dim-y hy ...))

  (let ([graph-file (cli-arg "f" #:long-name "graphfile" #:type String #:required #t
                             #:description "Path to the input data graph file")]
        [graph-name (cli-arg "g" #:long-name "graphname" #:type String #:required #t
                             #:description "Name of the input graph")])
    (load-graph #:file graph-file #:name graph-name
                #:vertex-type BFSVertex #:weighted #f))

  (register-actions bfs-action)

  (let ([root (cli-arg "root" #:long-name "bfsroot" #:type UInt32 #:required #t
                       #:description "Root vertex for BFS")]
        [shuffle? (cli-flag "shuffle" #:long-name "shuffle_vertices"
                            #:description "Randomize vertex placement")])
    (germinate bfs-action #:root root #:arguments (0) #:shuffle shuffle?))

  (run)

  (let ([verify? (cli-flag "verify" #:long-name "verification"
                           #:description "Enable verification")])
    (when verify?
      (verify #:field level #:extension ".bfs")))

  (let ([output-dir (cli-arg "od" #:long-name "outputdirectory" #:type String #:default "./"
                             #:description "Output directory")]
        [trail (cli-arg "trail" #:long-name "trail_number" #:type UInt32 #:default 0
                        #:description "Trail number")])
    (write-results #:output-dir output-dir #:trail trail)))
```

### 9.1 BFS meaning

All vertices begin at `max-level`. The host germinates `bfs-action(root, 0)`. An action performs work only if its incoming level is lower than the current level. Work stores that level. The deferred diffusion remains valid only while the vertex level equals the action's incoming level; an action made stale by a better level is pruned. Each outgoing neighbor receives `incoming-level + 1`.

This is the source-level equivalent of the hand-written BFS behavior. The generated backend omits non-semantic debugging payload members; simulator termination uses `Action::origin_addr`.

## 10. Required C++ backend contract

A conforming Draft 0.2 compiler emits C++ that builds as a CCA-Simulator application. For BFS it must generate:

1. a `BFSVertex<Vertex_T>` template deriving from the supplied runtime vertex type, with initialized application fields and `configure_derived_class_LCOs()`;
2. a trivially-copyable action payload struct containing `incoming_level`;
3. four `CCAFunctionEvent` variables;
4. four templated handlers with internal signature
   `Closure(ComputeCell&, Address, ActionArgumentType)`;
5. four ABI wrappers matching `handler_func`:
   `Closure(ComputeCell&, Address, actionType, ActionArgumentType)`;
6. wrapper dispatch through `INVOKE_HANDLER_3` for `edges_min` and `edges_max` RPVO types;
7. payload conversion through `cca_create_action_argument<T>` and `cca_get_action_argument<T>`;
8. generated ghost bypass and fan-out behavior;
9. four calls to `register_function_event`;
10. a nine-argument `Action` construction for germination and each propagation;
11. graph loading and `transfer_graph_host_to_cca<BFSVertex<ghost_type_level_1>>`;
12. terminator creation, `germinate_action`, and `run_simulation`;
13. BFS result verification compatible with the simulator's `.bfs` files; and
14. a `CMakeLists.txt` using `cca_add_application(NAME ... SOURCE ...)`.

The generated application must compile under the simulator's C++ standard and warnings, pass the same BFS verification inputs as the hand-written implementation, and require no separate runtime library beyond CCA-Simulator.

## 11. Diagnostics

Compiler errors should include source file, line, column, phase, and a specific remedy. Examples:

```text
bfs.cca:12:18 [type] expected UInt32 for incoming-level, found Boolean
bfs.cca:17:9 [effect] propagate is only allowed in a diffuse phase
bfs.cca:19:20 [rpvo] vertex-level is unavailable on ghost vertices;
                    derive this argument from the action payload or edge instead
```

The parser should preserve Racket syntax locations through desugaring and IR nodes so errors remain attached to source forms.

## 12. Reserved extensions

The following are intentionally outside Draft 0.2:

- `Float64`, weighted-distance rules, and SSSP;
- fixed-iteration actions and reductions for PageRank;
- helper functions with inferred effects;
- streaming edge insertion;
- `(Future T)`, future LCO operations, remote allocation, and `call/cc` lowering;
- `#:rhizome-shared`, rhizome links, `bcast`, and `rhizome-collapse`; and
- first-class functions, general vectors, garbage collection, pthreads, or x86 lowering.

They should be introduced in later language versions only after BFS source semantics, C++ generation, simulator build integration, and end-to-end verification are stable.

## 13. Conformance criterion for Draft 0.2

Draft 0.2 is implemented when the compiler can compile the BFS program in Section 9 into an isolated generated application, the generated files build through the repository's top-level CMake configuration, and the resulting executable passes BFS verification on the same graph/root inputs as `BFS_CCASimulator`. Textual identity with the hand-written C++ is not required; observable BFS levels and simulator API compatibility are required.

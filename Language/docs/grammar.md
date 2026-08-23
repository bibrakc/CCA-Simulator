# CCA Language Grammar

Formal grammar for Draft 0.2 of the CCA language. The source syntax is
S-expression based (Racket datum syntax). `...` means zero or more repetitions,
`...+` means one or more.

## Program structure

A CCA program consists of two files:

- **Kernel** — algorithm definitions (vertex, constants, actions)
- **Driver** — host program (requires the kernel, orchestrates simulation)

The compiler entry point is the driver file.

### Kernel file

```
kernel ::= #lang cca
           kernel-declaration ...

kernel-declaration ::= constant-definition
                     | vertex-definition
                     | action-definition
```

A kernel must contain exactly one `define-vertex` and at least one `define-action`.

### Driver file

```
driver ::= #lang cca
           (require relative-path)
           (define-program binary-name
             host-form ...)
```

The `require` imports the kernel's declarations. The `define-program` defines
the host program that compiles to a C++ `main()`.

## Module system

```
require-form ::= (require string)    ; relative path to a .cca file
```

The required file's forms are inlined at the point of the `require`. Nested
`require` is supported.

## Constants

```
constant-definition ::= (define-constant name : type literal)
```

## Vertex definition

```
vertex-definition ::= (define-vertex name field-spec ...+)

field-spec ::= [name : type #:initial expr #:mutable]
             | [name : type #:initial expr]
             | [name : type #:mutable]
             | [name : type]
```

## Action definition

```
action-definition ::= (define-action name
                         (param-spec ...+)
                         action-body)

param-spec ::= [name : type]
```

The first parameter must be `(Pointer VertexType)` — the action target.
Remaining parameters form the serialized payload.

## Action body (canonical form)

```
action-body ::= (predicate expression
                  (work statement ...)
                  (diffuse
                    (predicate expression)
                    diffuse-statement ...))
```

### Paper-compatible sugar

```
action-body ::= (predicate expression
                  (begin
                    statement ...
                    (diffuse
                      (predicate expression
                        diffuse-statement ...))))

             |  (predicate expression
                  (begin
                    statement ...
                    (diffuse
                      diffuse-statement ...)))
```

Both forms desugar to the canonical four-phase shape during parsing.

## Host program (define-program)

```
program-definition ::= (define-program binary-name-string
                          host-form ...)

host-form ::= (let (host-binding ...+) host-form ...+)
            | (create-simulator host-option ...)
            | (load-graph host-option ...)
            | (register-actions action-name ...+)
            | (germinate action-name host-option ...)
            | (run)
            | (when condition (verify host-option ...))
            | (write-results host-option ...)

host-binding ::= [name cli-arg-form]
               | [name cli-flag-form]

host-option ::= #:keyword value

value ::= literal | name | cli-arg-form | cli-flag-form | (literal ...)

condition ::= name    ; a let-bound boolean variable
```

### CLI argument forms

```
cli-arg-form ::= (cli-arg short-name
                    #:long-name string
                    #:type cli-type
                    #:default literal         ; for optional args
                    #:required boolean         ; for required args
                    #:description string)

cli-flag-form ::= (cli-flag short-name
                    #:long-name string
                    #:description string)

cli-type ::= UInt32 | String | Boolean
```

### create-simulator options

```
#:shape      name-or-variable    ; e.g. shape (from CLI) or 'square (literal)
#:dim-x      uint32-value
#:dim-y      uint32-value
#:memory-per-cc  uint32-value
#:mesh-type  uint32-value
#:routing    uint32-value
#:htree-depth    uint32-value
#:htree-bandwidth uint32-value
```

### load-graph options

```
#:file         string-value      ; path to graph file
#:name         string-value      ; graph name for output naming
#:vertex-type  vertex-name       ; which vertex struct to instantiate
#:weighted     boolean           ; whether graph has edge weights
```

### germinate options

```
#:root       uint32-value        ; root vertex ID
#:arguments  (literal ...)       ; initial payload values
#:shuffle    boolean-value       ; whether to shuffle vertex placement
```

### verify options

```
#:field      field-name          ; which vertex field to check
#:extension  string              ; verification file extension (e.g. ".bfs")
```

### write-results options

```
#:output-dir string-value        ; directory for output files
#:trail      uint32-value        ; experiment trail number
```

## Types

```
type ::= Unit
       | Boolean
       | UInt32
       | Integer            ; alias for UInt32
       | Address
       | Edge
       | (Pointer name)
       | (Vector type)
```

## Expressions

```
expression ::= literal
             | name
             | (operator expression ...+)
             | (vertex-field expression)       ; e.g. (vertex-level v)
             | (vertex-id expression)
             | (edge-address expression)
             | (edge-weight expression)
             | (let ([name expression] ...) expression)
             | (if expression expression expression)

literal ::= integer          ; 0..4294967295
           | #t | #f

operator ::= + | - | *
           | > | >= | < | <= | =
           | and | or | not
           | eq?              ; alias for =, desugared during parsing
```

## Statements (work phase)

```
statement ::= (set-vertex-field! target expression)
            | (set! (vertex-field target) expression)
            | (let ([name expression] ...) statement ...+)
            | (if expression (begin statement ...) (begin statement ...))
            | (begin statement ...)
```

`set-vertex-field!` is shorthand: `(set-vertex-level! v x)` means set the
`level` field. The `(set! (vertex-field target) value)` form is equivalent.

## Statements (diffuse phase)

```
diffuse-statement ::= (for-each ([name : Edge] (vertex-edges target))
                        diffuse-statement ...+)
                    | (propagate action-name address-expr argument-expr ...)
                    | (let ([name expression] ...) diffuse-statement ...+)
                    | (if expression
                        (begin diffuse-statement ...)
                        (begin diffuse-statement ...))
```

## Vertex field accessors

For a vertex with field `level`:
- Read: `(vertex-level v)` → `UInt32`
- Write: `(set-vertex-level! v expr)` or `(set! (vertex-level v) expr)`

Built-in accessors available on all vertices:
- `(vertex-id v)` → `UInt32`
- `(vertex-edges v)` → `(Vector Edge)` (used only in `for-each`)

## Edge accessors

- `(edge-address e)` → `Address`
- `(edge-weight e)` → `UInt32`

## Phase restrictions

| Phase | May read vertex | May write vertex | May propagate |
|---|---|---|---|
| Predicate | ✓ | ✗ | ✗ |
| Work | ✓ | ✓ | ✗ |
| Diffuse predicate | ✓ | ✗ | ✗ |
| Diffuse | payload/edge only | ✗ | ✓ |

Diffuse propagation arguments must be **ghost-safe**: they may depend on action
payload parameters, constants, literals, and edge data, but must NOT read vertex
fields (ghost RPVO objects do not contain application fields).

## Name mangling

Source identifiers are mangled to valid C++ identifiers:
- `-` → `_` (e.g. `bfs-action` → `bfs_action`)
- `!` → removed (e.g. `set-vertex-level!` → `set_vertex_level`)
- `?` → `_p_0x3f` (e.g. `verify?` → `verify_p_0x3f`)

C++ keywords are prefixed with `cca_` to avoid conflicts.

## Comments

Line comments begin with `;`:
```racket
;; This is a comment
(define-constant max-level : UInt32 999999) ; inline comment
```

## Reserved for future versions

- `Float64` type
- Helper functions (`define`)
- `(Future T)` and `call/cc`
- `#:rhizome-shared` annotation
- `rhizome-collapse` and `bcast`
- Multiple vertex types per program
- Actions without all four phases (optional work, optional diffuse)

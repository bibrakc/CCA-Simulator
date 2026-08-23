# CCA Language Grammar

Formal grammar for Draft 0.1 of the CCA language. The source syntax is
S-expression based (Racket datum syntax). `...` means zero or more repetitions,
`...+` means one or more.

## Program structure

```
program ::= #lang cca
            declaration ...

declaration ::= constant-definition
              | vertex-definition
              | action-definition
              | application-definition
```

A Draft 0.1 program must contain exactly one `define-vertex`, at least one
`define-action`, and exactly one `define-application`.

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

## Application definition

```
application-definition ::= (define-application name
                              #:binary-name string
                              #:vertex-type vertex-name
                              #:root-action action-name
                              #:root-arguments (literal ...)
                              #:result-field field-name
                              #:verification verification-kind)

verification-kind ::= bfs-level-file
```

## Types

```
type ::= Unit
       | Boolean
       | UInt32
       | Integer            ; alias for UInt32 in Draft 0.1
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

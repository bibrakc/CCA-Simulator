# Cost Models

Configurable cost model files for the CCA compiler.

A cost model defines how many simulated cycles each primitive operation takes.
The compiler uses it to generate accurate `cc.apply_CPI(N)` calls in action
handlers.

## Usage

```bash
racket Language/main.rkt --compile --output /tmp/out \
    --cost-model Language/cost-models/memory-heavy-cost-model.rkt \
    Language/examples/bfs/bfs.cca
```

If no `--cost-model` is specified, the built-in default model is used
(defined in `compiler/cost-model.rkt`).

## File format

S-expression list of `(operation-name cycles)` pairs:

```racket
((field-read 2)
 (field-write 2)
 (comparison 1)
 (arithmetic 1)
 (boolean-op 1)
 (literal-access 0)
 (variable-access 0)
 (edge-read 2)
 (propagate 0))
```

Any omitted operation defaults to the built-in value.

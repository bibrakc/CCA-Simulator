# cca/lang/

This directory implements the Racket `#lang cca` reader module.

When a file begins with `#lang cca`, Racket looks for a reader at this path
(`cca/lang/reader.rkt`). The reader:

1. Reads all top-level S-expressions as **data** (no evaluation)
2. Wraps them in a module that runs the CCA compiler frontend (parse, resolve, typecheck)
3. Reports whether the program is valid

## Usage

```bash
# From the CCA-Simulator root, with -S to add Language to the search path:
racket -S Language Language/examples/bfs/bfs.cca
```

## Note

For full compilation to C++, use the CLI instead:
```bash
racket Language/main.rkt --compile --output <dir> <file.cca>
```

The `#lang cca` reader is primarily useful for quick syntax/type checking
directly from an editor or REPL without invoking the full compilation pipeline.

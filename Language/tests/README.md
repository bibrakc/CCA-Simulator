# Tests

Compiler test suite using Racket's `rackunit` framework.

## Running

```bash
racket Language/tests/run-tests.rkt
```

## Test categories

- **Reader tests** — S-expression reading, `#lang` stripping, missing files
- **Parser tests** — AST construction, field/action/application parsing, error cases
- **Type/effect tests** — negative cases that must be rejected:
  - `propagate` in work phase
  - mutation in diffuse phase
  - non-Boolean predicate
  - type mismatch in field assignment

## Adding tests

Add test cases to `run-tests.rkt` or create focused test files in the
subdirectories (`unit/`, `passes/`, `negative/`, `golden/`, `integration/`).

# Frontend

Analysis passes that validate CCA source programs without generating code.

| File | Pass | Input → Output |
|---|---|---|
| `read-source.rkt` | Reader | file path → list of S-expressions |
| `parse.rkt` | Parser | S-expressions → `cca-program` AST |
| `resolve.rkt` | Resolution | AST → `resolved-program` with symbol table |
| `typecheck.rkt` | Type/Effect check | resolved program → validated program (or error) |

All passes raise `exn:fail` with source location and phase on error.

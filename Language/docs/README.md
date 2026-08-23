# Language Documentation

Authoritative specifications and design documents for the CCA language and compiler.

| Document | Contents |
|---|---|
| `grammar.md` | **Formal grammar (Draft 0.2)** — complete BNF-style syntax for kernel files, driver files, host-level forms, CLI arguments, and all declarations |
| `language-specification.md` | **Language semantics** — execution model, type system, phase effects, RPVO lowering, and C++ backend contract |
| `compiler-design.md` | **Implementation plan** — pass pipeline, IRs, testing strategy, milestones, and architectural decisions |

## Relationship between documents

- **`grammar.md`** is the canonical syntax reference. If the grammar says something is valid, the compiler must accept it.
- **`language-specification.md`** explains *what programs mean* — semantics, type rules, effect restrictions, ghost-safety, and what C++ must be generated. It references the grammar rather than duplicating it.
- **`compiler-design.md`** explains *how the compiler is built* — passes, data structures, testing, and development milestones. It's an implementation guide, not a user-facing document.

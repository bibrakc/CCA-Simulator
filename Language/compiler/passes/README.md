# Compiler Passes

This directory will contain explicit lowering passes between the typed AST
and the C++ backend:

- `lower-actions.rkt` — split actions into four explicit phase records
- `synthesize-payloads.rkt` — generate payload struct layouts
- `lower-rpvo.rkt` — make ghost/RPVO behavior explicit in IR

Currently, these transformations happen directly inside the backend emitter.
They will be extracted here as the language grows beyond BFS.

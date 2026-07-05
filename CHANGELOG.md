# Changelog

## [0.1.0]

First release. Rapier 3D physics as a viewport-lib runtime plugin:

- `RapierPreparePlugin` / `RapierSimulatePlugin` run rapier on the runtime's
  fixed-timestep loop.
- `NodeId <-> Handle` map so bodies are addressed by scene `NodeId`.
- Transform writeback for dynamic bodies and NodeId-keyed contact events.
- `with_body_mut` / `with_collider_mut` / `with_state` / `with_state_mut` for
  direct access to the full rapier API (joints, queries, character controller).
- Eight examples and integration tests covering the lifecycle, sync, writeback,
  events, and a broad-phase raycast.

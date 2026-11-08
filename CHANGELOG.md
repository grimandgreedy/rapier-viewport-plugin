# Changelog

## [0.2.0]

- Update to viewport-lib 0.23. Node world transforms are now `f64`; the plugin
  converts them to `f32` when it syncs kinematic and fixed bodies from the
  scene.
- The examples build against the 0.23 input API.

## [0.1.1]

- Update to viewport-lib 0.20. The plugin is source-compatible with 0.20 and
  needs no code changes.

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

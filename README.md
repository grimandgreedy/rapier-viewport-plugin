# rapier-viewport-plugin

[Rapier 3D][rapier] physics as a [viewport-lib][vp] runtime plugin.

This is a small adapter crate. It does two things:

1. Implements viewport-lib's `RuntimePlugin` trait so rapier runs inside a `ViewportRuntime` alongside any other plugin, on its fixed-timestep loop.
2. Keeps a `NodeId <-> Handle` map so you address bodies by the same `viewport_lib::NodeId` you use for scene nodes everywhere else.

Everything else is rapier, used directly through its own builders. The crate re-exports `rapier3d` and `parry3d` at the crate root so you don't manage those dependencies separately. It does not wrap the rapier API: you build bodies and colliders with rapier's `RigidBodyBuilder` / `ColliderBuilder` and reach the rest of rapier through the `with_*` methods below.
[rapier]: https://github.com/dimforge/rapier
[vp]: https://github.com/grimandgreedy/viewport-lib

## Using it

```rust
use rapier_viewport_plugin::{
    parry3d::shape::SharedShape,
    rapier3d::prelude::{ActiveEvents, ColliderBuilder, RigidBodyBuilder},
    RapierContactEvent, RapierPlugin,
};
use viewport_lib::{FixedTimestep, ViewportRuntime};

// 1. Create the handle and register bodies.
let mut rapier = RapierPlugin::new();

let floor_body = RigidBodyBuilder::fixed().build();
let floor_collider = ColliderBuilder::new(SharedShape::halfspace(glam::Vec3::Z)).build();
rapier.add_body(floor_node_id, floor_body, floor_collider);

let ball_body = RigidBodyBuilder::dynamic()
    .translation(glam::Vec3::new(0.0, 0.0, 5.0))
    .build();
let ball_collider = ColliderBuilder::ball(0.5)
    .restitution(0.6)
    .active_events(ActiveEvents::COLLISION_EVENTS)
    .build();
rapier.add_body(ball_node_id, ball_body, ball_collider);

// 2. Split into the two RuntimePlugins the runtime needs. Keep `rapier`
//    around to add/remove bodies later.
let (prepare, simulate) = rapier.clone().into_plugins();
let mut runtime = ViewportRuntime::new()
    .with_fixed_timestep(FixedTimestep::new(120.0))
    .with_plugin(prepare)
    .with_plugin(simulate);

// 3. Per frame: step the runtime, drain physics events.
let output = runtime.step(&mut scene, &mut selection, &frame_ctx);
for ev in output.events.drain::<RapierContactEvent>() {
    // ev.node_a, ev.node_b, ev.started
}
```

The two plugins do their work automatically:

- `RapierPreparePlugin` runs in the `Prepare` phase and copies the scene transform of every `Fixed` or `KinematicPosition` body into rapier. Move a kinematic node with `scene.set_local_transform(...)` and it pushes dynamic bodies in the next step.
- `RapierSimulatePlugin` runs in the `Simulate` phase, calls `PhysicsPipeline::step`, writes every dynamic body's new transform back to the scene, and translates rapier's `CollisionEvent`s into NodeId-keyed `RapierContactEvent`s on the runtime output bus.

## Reaching the rest of rapier

The handle has a few lifecycle methods plus three ways to get at rapier's own types directly. Together they reach the entire rapier API.


| Access | Methods | Use when |
|---|---|---|
| Lifecycle | `add_body`, `remove_body`, `clear_bodies`, `body_handle`, `collider_handle`, `into_plugins` | Registering bodies and looking up handles; the common path |
| Per body | `with_body_mut(node, |b| ...)`, `with_collider_mut(node, |c| ...)` | Apply impulses, set velocities, lock axes, flip the sensor flag, change collision groups |
| Full state | `with_state_mut(|s| ...)`, `with_state(|s| ...)` | Joints (`s.impulse_joints`), spatial queries (`s.broad_phase.as_query_pipeline(...)`), character controller, vehicle controller, diagnostics |

Apply an impulse:

```rust
rapier.with_body_mut(ball_id, |body| {
    body.apply_impulse(glam::Vec3::Z * 5.0, true);
});
```

Add a revolute joint, using `body_handle` to translate `NodeId`s to rapier
handles:

```rust
use rapier_viewport_plugin::rapier3d::prelude::*;

let h_parent = rapier.body_handle(parent_node).unwrap();
let h_child = rapier.body_handle(child_node).unwrap();
rapier.with_state_mut(|s| {
    let joint = RevoluteJointBuilder::new(Vector::y_axis())
        .local_anchor1([0.0, 0.0, 0.0].into())
        .local_anchor2([0.0, 1.0, 0.0].into())
        .build();
    s.impulse_joints.insert(h_parent, h_child, joint, true);
});
```

Raycast through the broad phase:

```rust
use rapier_viewport_plugin::{
    parry3d::query::Ray,
    rapier3d::pipeline::QueryFilter,
};

let hit_node = rapier.with_state(|s| {
    let qp = s.broad_phase.as_query_pipeline(
        s.narrow_phase.query_dispatcher(),
        &s.bodies,
        &s.colliders,
        QueryFilter::default(),
    );
    let ray = Ray::new(origin, dir);
    qp.cast_ray_and_get_normal(&ray, 100.0, true)
        .and_then(|(handle, _)| s.node_for_collider(handle))
});
```

## Examples

Eight examples cover the things you'd typically wire up:

| Example | What it shows |
|---|---|
| `falling_stack` | Dynamic bodies, gravity, a halfspace floor, transform writeback to the scene |
| `bouncing_spheres` | Restitution, multiple simultaneous contacts, draining `RapierContactEvent`s from the runtime output |
| `kinematic_pusher` | Driving a `KinematicPosition` body's transform from input each frame; rapier's implicit velocity pushes dynamic balls |
| `ray_probe` | Camera-forward raycast through `with_state` and `BroadPhaseBvh::as_query_pipeline`. Click or space to mark the hit body |
| `joint_chain` | A 5-link revolute-joint chain: `body_handle` to translate NodeIds, `with_state_mut` to insert into `ImpulseJointSet`, `with_body_mut` to apply impulses |
| `sensor_trigger` | Trigger-volume detection: `ColliderBuilder::sensor(true)` for a ghost collider that emits events without contact forces |
| `collision_groups` | Layer-based collision filtering with `InteractionGroups` (32-bit membership / filter bitmasks) |
| `character_controller` | Keyboard-driven character using rapier's `KinematicCharacterController::move_shape(...)` through `with_state`. Walks around fixed box obstacles |

Run any of them:

```
cargo run --example falling_stack
cargo run --example bouncing_spheres
cargo run --example kinematic_pusher
cargo run --example ray_probe
cargo run --example joint_chain
cargo run --example sensor_trigger
cargo run --example collision_groups
cargo run --example character_controller
```

### Not shown by an example yet

These rapier features work through the plugin (everything is reachable via `with_state`) but don't have a dedicated example yet:

- Other joint types (Fixed, Prismatic, Spherical, Rope/Distance, Generic 6-DOF,
  motors and limits)
- `DynamicRayCastVehicleController`
- Compound colliders, `Heightfield`, `TriMesh`, `ConvexHull`
- Continuous collision detection for tunnelling prevention
- Locked translation / rotation axes
- Continuous forces / torques (rather than the impulse path)
- Sleeping / islands inspection
- `PhysicsPipeline::counters` for step diagnostics

The patterns above generalise: joint variants follow `joint_chain`, vehicle and character follow `character_controller`, continuous forces follow the `with_body_mut` pattern in `joint_chain`'s `kick()`.

## License

`viewport-lib-wind` is a part of the viewport-lib ecosystem.

GPL-3.0

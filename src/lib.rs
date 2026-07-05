//! Rapier 3D physics plugin for viewport-lib.
//!
//! Wraps [`rapier3d`] behind the [`RuntimePlugin`](viewport_lib::RuntimePlugin)
//! trait so it runs inside a `ViewportRuntime` alongside any other viewport-lib
//! plugin. The crate is intentionally a thin adapter: it owns the lifecycle
//! integration and the `NodeId` translation layer, and re-exports [`rapier3d`]
//! and [`parry3d`] so consumers use rapier's own types and builders directly.
//!
//! # Quick start
//!
//! ```rust,ignore
//! use rapier_viewport_plugin::{RapierPlugin, RapierContactEvent, rapier3d::prelude::*};
//! use viewport_lib::{FixedTimestep, ViewportRuntime};
//!
//! let mut rapier = RapierPlugin::new();
//!
//! // Build bodies and colliders with rapier's own builders.
//! let floor_body = RigidBodyBuilder::fixed().build();
//! let floor_collider = ColliderBuilder::halfspace(Vector::z_axis()).build();
//! rapier.add_body(floor_id, floor_body, floor_collider);
//!
//! let box_body = RigidBodyBuilder::dynamic()
//!     .translation(glam::Vec3::new(0.0, 0.0, 5.0))
//!     .build();
//! let box_collider = ColliderBuilder::cuboid(0.5, 0.5, 0.5)
//!     .restitution(0.4)
//!     .active_events(ActiveEvents::COLLISION_EVENTS)
//!     .build();
//! rapier.add_body(box_id, box_body, box_collider);
//!
//! let (prepare, simulate) = rapier.clone().into_plugins();
//! let runtime = ViewportRuntime::new()
//!     .with_fixed_timestep(FixedTimestep::new(120.0))
//!     .with_plugin(prepare)
//!     .with_plugin(simulate);
//!
//! // Per-frame:
//! //   let output = runtime.step(&mut scene, &mut selection, &frame_ctx);
//! //   for ev in output.events.drain::<RapierContactEvent>() { ... }
//! ```
//!
//! For anything not in the curated API -- impulses, velocity get/set, joints,
//! collision groups, sensor flags, locked axes, the query pipeline, character
//! controllers, etc. -- reach in through [`RapierPlugin::with_body_mut`],
//! [`RapierPlugin::with_collider_mut`], or [`RapierPlugin::with_state_mut`].
//!
//! # Architecture
//!
//! Two `RuntimePlugin` implementors share a single `Arc<Mutex<RapierState>>`:
//!
//! | Phase    | Plugin                  | What it does                                                  |
//! |----------|-------------------------|---------------------------------------------------------------|
//! | Prepare  | `RapierPreparePlugin`   | Syncs fixed and kinematic body transforms from scene -> rapier |
//! | Simulate | `RapierSimulatePlugin`  | Calls `PhysicsPipeline::step`, writes dynamic bodies back     |
//!
//! The user-facing [`RapierPlugin`] keeps a third reference to the same state,
//! so bodies can be added or removed at runtime while the simulation runs.

pub mod events;
pub mod plugin;
pub mod state;

pub use events::RapierContactEvent;
pub use plugin::{RapierPlugin, RapierPreparePlugin, RapierSimulatePlugin};
pub use state::RapierState;

/// Re-export of the [`rapier3d`] crate so plugin consumers don't need a
/// separate dependency (and can't accidentally pull in a different version
/// than the plugin was built against).
pub use rapier3d;

/// Re-export of the [`parry3d`] crate (rapier's collision-detection backend).
/// Useful for constructing custom shapes or query filters to pass through the
/// escape hatches on [`RapierPlugin`].
pub use parry3d;

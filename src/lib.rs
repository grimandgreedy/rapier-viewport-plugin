//! Rapier 3D physics plugin for viewport-lib.
//!
//! Wraps [`rapier3d`] behind the [`RuntimePlugin`](viewport_lib::RuntimePlugin) trait so it
//! integrates into a `ViewportRuntime` alongside any other viewport-lib plugin.
//!
//! # Quick start
//!
//! ```rust,ignore
//! use rapier_viewport_plugin::{RapierPlugin, RapierBodyDesc, RapierColliderShape, RapierContactEvent};
//! use viewport_lib::{FixedTimestep, ViewportRuntime};
//!
//! let mut rapier = RapierPlugin::new();
//!
//! // Fixed floor.
//! rapier.add_body_at(
//!     RapierBodyDesc::fixed(floor_id, RapierColliderShape::HalfSpace {
//!         outward_normal: glam::Vec3::Z,
//!     }),
//!     glam::Vec3::ZERO,
//!     glam::Quat::IDENTITY,
//! )?;
//!
//! // Falling box.
//! rapier.add_body_at(
//!     RapierBodyDesc::dynamic(box_id, RapierColliderShape::Cuboid {
//!         half_extents: glam::Vec3::splat(0.5),
//!     }),
//!     glam::Vec3::new(0.0, 0.0, 5.0),
//!     glam::Quat::IDENTITY,
//! )?;
//!
//! let (prepare, simulate) = rapier.clone().into_plugins();
//! let runtime = ViewportRuntime::new()
//!     .with_fixed_timestep(FixedTimestep::new(120.0))
//!     .with_plugin(prepare)
//!     .with_plugin(simulate);
//!
//! // Per-frame event loop:
//! // let output = runtime.step(&frame_ctx, &mut scene, &mut selection);
//! // for ev in output.events.drain::<RapierContactEvent>() { ... }
//! ```
//!
//! # Architecture
//!
//! Two `RuntimePlugin` implementors share a single `Arc<Mutex<RapierState>>`:
//!
//! | Phase   | Plugin                  | What it does                                               |
//! |---------|-------------------------|------------------------------------------------------------|
//! | Prepare | `RapierPreparePlugin`   | Syncs fixed and kinematic body transforms from scene -> Rapier |
//! | Simulate| `RapierSimulatePlugin`  | Calls `PhysicsPipeline::step`, writes dynamic bodies back  |
//!
//! The user-facing [`RapierPlugin`] clone keeps a third reference to the same
//! state, allowing bodies to be added or removed at runtime while the simulation
//! is running.

pub mod body;
pub mod convert;
pub mod error;
pub mod events;
pub mod plugin;
pub mod queries;
pub(crate) mod state;

pub use body::{RapierBodyDesc, RapierBodyKind, RapierColliderShape};
pub use error::RapierError;
pub use events::RapierContactEvent;
pub use plugin::{RapierPlugin, RapierPreparePlugin, RapierSimulatePlugin};
pub use queries::{RapierRaycastHit, RapierShapeCastHit};

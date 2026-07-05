use std::sync::{Arc, Mutex, MutexGuard};

use rapier3d::dynamics::{RigidBody, RigidBodyHandle};
use rapier3d::geometry::{Collider, ColliderHandle, CollisionEvent};
use rapier3d::math::Vector;
use viewport_lib::runtime::plugin::phase;
use viewport_lib::{NodeId, RuntimePlugin, RuntimeStepContext};

use crate::events::{EventCollector, RapierContactEvent};
use crate::state::RapierState;

/// Acquire the state mutex, transparently recovering from poisoning.
///
/// A poisoned mutex means a previous holder panicked. `RapierState` is
/// plain Rust values (rapier's sets, two `IndexMap`s, integration params,
/// etc.) with no unsafe interior mutation, so taking the underlying state
/// and continuing is safer in practice than cascading the panic into
/// every future plugin call.
#[inline]
fn lock_state(state: &Mutex<RapierState>) -> MutexGuard<'_, RapierState> {
    state.lock().unwrap_or_else(|poisoned| poisoned.into_inner())
}

// ---------------------------------------------------------------------------
// RapierPlugin: user-facing handle
// ---------------------------------------------------------------------------

/// User-facing handle to a Rapier physics simulation running inside viewport-lib.
///
/// Holds an `Arc<Mutex<RapierState>>` shared with the two `RuntimePlugin`
/// implementors returned by [`into_plugins`](Self::into_plugins). Cloning the
/// handle is cheap.
///
/// All methods on this handle take the state mutex. They block while the
/// simulate plugin is running; keep handle calls off the hot path or batch
/// them between frames.
///
/// **Mutex poisoning:** if a previous holder of the lock panicked (most
/// commonly: a closure passed to `with_body_mut` / `with_state_mut`
/// panicked), every subsequent lock acquisition transparently recovers
/// the underlying state and continues. The plugin never panics on a
/// poisoned mutex.
///
/// # Usage
///
/// ```rust,ignore
/// use rapier_viewport_plugin::{RapierPlugin, rapier3d::prelude::*};
///
/// let mut rapier = RapierPlugin::new();
///
/// // Build bodies and colliders using rapier's own builders.
/// let floor_body = RigidBodyBuilder::fixed().build();
/// let floor_collider = ColliderBuilder::halfspace(Vector::z_axis()).build();
/// rapier.add_body(floor_id, floor_body, floor_collider);
///
/// let box_body = RigidBodyBuilder::dynamic()
///     .translation(glam::Vec3::new(0.0, 0.0, 5.0))
///     .build();
/// let box_collider = ColliderBuilder::cuboid(0.5, 0.5, 0.5)
///     .restitution(0.4)
///     .active_events(ActiveEvents::COLLISION_EVENTS)
///     .build();
/// rapier.add_body(box_id, box_body, box_collider);
///
/// let (prepare, simulate) = rapier.clone().into_plugins();
/// ```
///
/// For anything not in the curated API (joints, locked axes, sensor flags,
/// collision groups, the query pipeline, etc.) use the escape hatches:
/// [`with_body_mut`](Self::with_body_mut), [`with_collider_mut`](Self::with_collider_mut),
/// [`with_state_mut`](Self::with_state_mut), and [`with_state`](Self::with_state).
#[derive(Clone)]
pub struct RapierPlugin {
    state: Arc<Mutex<RapierState>>,
}

impl Default for RapierPlugin {
    fn default() -> Self {
        Self::new()
    }
}

impl RapierPlugin {
    /// Create with Earth-like gravity (-9.81 m/s^2 on the Z axis, Z-up world).
    pub fn new() -> Self {
        Self {
            state: Arc::new(Mutex::new(RapierState::new(Vector::new(0.0, 0.0, -9.81)))),
        }
    }

    pub fn with_gravity(self, g: glam::Vec3) -> Self {
        lock_state(&self.state).gravity = g;
        self
    }

    // ------------------------------------------------------------------
    // Body lifecycle
    // ------------------------------------------------------------------

    /// Register a body and its collider, binding both to a scene node.
    ///
    /// Construct `body` and `collider` with rapier's `RigidBodyBuilder` and
    /// `ColliderBuilder`. Initial transform, mass, restitution, friction,
    /// collision groups, sensor flag, CCD, locked axes, and every other
    /// rapier property are set on the builders.
    pub fn add_body(&mut self, node_id: NodeId, body: RigidBody, collider: Collider) {
        lock_state(&self.state).add_body(node_id, body, collider);
    }

    /// Remove the body bound to `node_id`. No-op if the node was never registered.
    pub fn remove_body(&mut self, node_id: NodeId) {
        lock_state(&self.state).remove_body(node_id);
    }

    /// Remove every registered body and collider.
    pub fn clear_bodies(&mut self) {
        lock_state(&self.state).clear_bodies();
    }

    // ------------------------------------------------------------------
    // Handle accessors + escape hatches
    // ------------------------------------------------------------------

    /// Look up the rapier [`RigidBodyHandle`] for a registered body.
    pub fn body_handle(&self, node_id: NodeId) -> Option<RigidBodyHandle> {
        lock_state(&self.state).node_to_body.get(&node_id).copied()
    }

    /// Look up the rapier [`ColliderHandle`] for a registered body.
    pub fn collider_handle(&self, node_id: NodeId) -> Option<ColliderHandle> {
        lock_state(&self.state).node_to_collider.get(&node_id).copied()
    }

    /// Run a closure with mutable access to a registered body's [`RigidBody`].
    /// Returns `None` if the node has no body.
    ///
    /// ```rust,ignore
    /// plugin.with_body_mut(id, |b| {
    ///     b.apply_impulse(glam::Vec3::Z * 5.0, true);
    ///     b.lock_translations(true, true);
    /// });
    /// ```
    pub fn with_body_mut<R>(
        &mut self,
        node_id: NodeId,
        f: impl FnOnce(&mut RigidBody) -> R,
    ) -> Option<R> {
        let mut state = lock_state(&self.state);
        let handle = state.node_to_body.get(&node_id).copied()?;
        let body = state.bodies.get_mut(handle)?;
        Some(f(body))
    }

    /// Run a closure with mutable access to a registered body's [`Collider`].
    /// Returns `None` if the node has no collider.
    pub fn with_collider_mut<R>(
        &mut self,
        node_id: NodeId,
        f: impl FnOnce(&mut Collider) -> R,
    ) -> Option<R> {
        let mut state = lock_state(&self.state);
        let handle = state.node_to_collider.get(&node_id).copied()?;
        let collider = state.colliders.get_mut(handle)?;
        Some(f(collider))
    }

    /// Run a closure with mutable access to the entire [`RapierState`]. Use
    /// this for anything the curated API does not cover: joints, spatial
    /// queries against the broad phase, character controllers, etc.
    ///
    /// Note: the internal mutex is held for the duration of `f`. Do not call
    /// back into other [`RapierPlugin`] methods from inside the closure -- that
    /// will deadlock.
    pub fn with_state_mut<R>(&mut self, f: impl FnOnce(&mut RapierState) -> R) -> R {
        f(&mut lock_state(&self.state))
    }

    /// Read-only counterpart to [`with_state_mut`](Self::with_state_mut).
    pub fn with_state<R>(&self, f: impl FnOnce(&RapierState) -> R) -> R {
        f(&lock_state(&self.state))
    }

    // ------------------------------------------------------------------
    // Plugin extraction
    // ------------------------------------------------------------------

    /// Split into the two `RuntimePlugin` objects the runtime needs. Bodies
    /// can still be added or removed at runtime through the cloned handle
    /// after this is called.
    pub fn into_plugins(self) -> (RapierPreparePlugin, RapierSimulatePlugin) {
        (
            RapierPreparePlugin {
                state: Arc::clone(&self.state),
            },
            RapierSimulatePlugin {
                state: Arc::clone(&self.state),
            },
        )
    }
}

// ---------------------------------------------------------------------------
// Prepare plugin: sync scene -> rapier for fixed and kinematic bodies
// ---------------------------------------------------------------------------

pub struct RapierPreparePlugin {
    state: Arc<Mutex<RapierState>>,
}

impl RuntimePlugin for RapierPreparePlugin {
    fn priority(&self) -> i32 {
        phase::PREPARE
    }

    fn step(&mut self, ctx: &mut RuntimeStepContext<'_>) {
        let mut state = lock_state(&self.state);

        // Snapshot (node, handle, is_kinematic) up front so we can mutate
        // bodies in the loop without aliasing the map.
        let synced: Vec<(NodeId, RigidBodyHandle, bool)> = state
            .node_to_body
            .iter()
            .filter_map(|(&node_id, &handle)| {
                let body = state.bodies.get(handle)?;
                if body.is_kinematic() {
                    Some((node_id, handle, true))
                } else if body.is_fixed() {
                    Some((node_id, handle, false))
                } else {
                    None
                }
            })
            .collect();

        for (node_id, handle, is_kinematic) in synced {
            let Some(node) = ctx.scene.node(node_id) else {
                continue;
            };
            let m = node.world_transform();
            let (_, rotation, translation) = m.to_scale_rotation_translation();
            let pose = rapier3d::math::Pose { rotation, translation };
            if let Some(body) = state.bodies.get_mut(handle) {
                if is_kinematic {
                    body.set_next_kinematic_position(pose);
                } else {
                    body.set_position(pose, true);
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Simulate plugin: step rapier, write back dynamic body transforms
// ---------------------------------------------------------------------------

pub struct RapierSimulatePlugin {
    state: Arc<Mutex<RapierState>>,
}

impl RuntimePlugin for RapierSimulatePlugin {
    fn priority(&self) -> i32 {
        phase::SIMULATE
    }

    fn step(&mut self, ctx: &mut RuntimeStepContext<'_>) {
        let dt = ctx.dt;
        let mut state = lock_state(&self.state);

        state.integration_params.dt = dt;

        let collector = EventCollector::default();

        // Destructure to satisfy the borrow checker: pipeline.step needs
        // &mut pipeline plus independent borrows of every other field.
        let RapierState {
            ref mut pipeline,
            ref gravity,
            ref integration_params,
            ref mut islands,
            ref mut broad_phase,
            ref mut narrow_phase,
            ref mut bodies,
            ref mut colliders,
            ref mut impulse_joints,
            ref mut multibody_joints,
            ref mut ccd_solver,
            ..
        } = *state;

        pipeline.step(
            *gravity,
            integration_params,
            islands,
            broad_phase,
            narrow_phase,
            bodies,
            colliders,
            impulse_joints,
            multibody_joints,
            ccd_solver,
            &(),
            &collector,
        );

        // Write dynamic body transforms back to the scene.
        let dynamic_bodies: Vec<(NodeId, glam::Affine3A)> = state
            .node_to_body
            .iter()
            .filter_map(|(&node_id, &handle)| {
                let body = state.bodies.get(handle)?;
                if body.is_dynamic() {
                    let pose = body.position();
                    let transform =
                        glam::Affine3A::from_rotation_translation(pose.rotation, pose.translation);
                    Some((node_id, transform))
                } else {
                    None
                }
            })
            .collect();

        for (node_id, transform) in dynamic_bodies {
            ctx.writeback.set_preserve_scale(node_id, transform);
        }

        // Translate raw collision events to NodeId-based events.
        let collisions = collector
            .collisions
            .into_inner()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        for event in collisions {
            let (h1, h2, started) = match event {
                CollisionEvent::Started(h1, h2, _) => (h1, h2, true),
                CollisionEvent::Stopped(h1, h2, _) => (h1, h2, false),
            };
            let node_a = state.node_for_collider(h1);
            let node_b = state.node_for_collider(h2);
            if let (Some(node_a), Some(node_b)) = (node_a, node_b) {
                ctx.output.events.emit(RapierContactEvent {
                    node_a,
                    node_b,
                    started,
                });
            }
        }
    }
}

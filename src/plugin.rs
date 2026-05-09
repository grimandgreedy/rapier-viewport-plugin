use std::sync::{Arc, Mutex};

use rapier3d::geometry::CollisionEvent;
use rapier3d::math::Vector;
use viewport_lib::runtime::plugin::phase;
use viewport_lib::{NodeId, RuntimePlugin, RuntimeStepContext, Scene};

use crate::body::RapierBodyDesc;
use crate::convert::{mat4_to_pose, pose_to_affine};
use crate::error::RapierError;
use crate::events::{EventCollector, RapierContactEvent};
use crate::queries::{RapierRaycastHit, RapierShapeCastHit};
use crate::state::RapierState;

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
/// simulate plugin is running, which matters if you ever drive the runtime from
/// a worker thread: keep handle calls off the hot path or batch them between
/// frames.
///
/// # Usage
///
/// ```rust,ignore
/// let mut rapier = RapierPlugin::new();
/// rapier.add_body_at(
///     RapierBodyDesc::fixed(floor_id, RapierColliderShape::HalfSpace {
///         outward_normal: glam::Vec3::Z,
///     }),
///     glam::Vec3::ZERO,
///     glam::Quat::IDENTITY,
/// )?;
///
/// let (prepare, simulate) = rapier.clone().into_plugins();
/// let runtime = ViewportRuntime::new()
///     .with_fixed_timestep(FixedTimestep::new(120.0))
///     .with_plugin(prepare)
///     .with_plugin(simulate);
/// ```
///
/// After each frame, read [`RapierContactEvent`]s from `output.events`:
/// ```rust,ignore
/// for ev in output.events.drain::<RapierContactEvent>() {
///     println!("contact: {:?} <-> {:?} started={}", ev.node_a, ev.node_b, ev.started);
/// }
/// ```
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
        self.state.lock().unwrap().gravity = g;
        self
    }

    // ------------------------------------------------------------------
    // Body management
    // ------------------------------------------------------------------

    /// Register a body at a specific world-space position and orientation.
    ///
    /// Returns an error if the body's collider shape is malformed (degenerate
    /// convex hull or trimesh). Other shapes are infallible.
    pub fn add_body_at(
        &mut self,
        desc: RapierBodyDesc,
        position: glam::Vec3,
        rotation: glam::Quat,
    ) -> Result<(), RapierError> {
        let transform = glam::Affine3A::from_rotation_translation(rotation, position);
        self.state.lock().unwrap().add_body(&desc, transform)
    }

    /// Register a body using the world transform of the scene node it is bound
    /// to. The node must already exist in the scene; if it does not, the body
    /// is registered at the origin.
    pub fn add_body_from_scene(
        &mut self,
        desc: RapierBodyDesc,
        scene: &Scene,
    ) -> Result<(), RapierError> {
        let transform = scene
            .node(desc.node_id)
            .map(|n| glam::Affine3A::from_mat4(n.world_transform()))
            .unwrap_or(glam::Affine3A::IDENTITY);
        self.state.lock().unwrap().add_body(&desc, transform)
    }

    /// Remove a body from the simulation. No-op if the node was never registered.
    pub fn remove_body(&mut self, node_id: NodeId) {
        self.state.lock().unwrap().remove_body(node_id);
    }

    /// Remove every body and collider from the simulation.
    pub fn clear_bodies(&mut self) {
        self.state.lock().unwrap().clear_bodies();
    }

    // ------------------------------------------------------------------
    // Runtime mutations
    // ------------------------------------------------------------------

    /// Apply a world-space linear impulse to a dynamic body.
    pub fn apply_impulse(&mut self, node_id: NodeId, impulse: glam::Vec3) {
        let mut state = self.state.lock().unwrap();
        if let Some(&handle) = state.node_to_body.get(&node_id) {
            if let Some(body) = state.bodies.get_mut(handle) {
                body.apply_impulse(impulse, true);
            }
        }
    }

    /// Apply a world-space angular impulse to a dynamic body.
    pub fn apply_angular_impulse(&mut self, node_id: NodeId, torque_impulse: glam::Vec3) {
        let mut state = self.state.lock().unwrap();
        if let Some(&handle) = state.node_to_body.get(&node_id) {
            if let Some(body) = state.bodies.get_mut(handle) {
                body.apply_torque_impulse(torque_impulse, true);
            }
        }
    }

    /// Set the linear velocity of a dynamic body directly.
    pub fn set_linear_velocity(&mut self, node_id: NodeId, vel: glam::Vec3) {
        let mut state = self.state.lock().unwrap();
        if let Some(&handle) = state.node_to_body.get(&node_id) {
            if let Some(body) = state.bodies.get_mut(handle) {
                body.set_linvel(vel, true);
            }
        }
    }

    /// Set the angular velocity of a dynamic body directly.
    pub fn set_angular_velocity(&mut self, node_id: NodeId, ang_vel: glam::Vec3) {
        let mut state = self.state.lock().unwrap();
        if let Some(&handle) = state.node_to_body.get(&node_id) {
            if let Some(body) = state.bodies.get_mut(handle) {
                body.set_angvel(ang_vel, true);
            }
        }
    }

    /// Wake a sleeping body.
    pub fn wake_up(&mut self, node_id: NodeId) {
        let mut state = self.state.lock().unwrap();
        if let Some(&handle) = state.node_to_body.get(&node_id) {
            if let Some(body) = state.bodies.get_mut(handle) {
                body.wake_up(true);
            }
        }
    }

    // ------------------------------------------------------------------
    // Runtime queries
    // ------------------------------------------------------------------

    /// World-space position of a body.
    pub fn position(&self, node_id: NodeId) -> Option<glam::Vec3> {
        let state = self.state.lock().unwrap();
        let handle = state.node_to_body.get(&node_id).copied()?;
        Some(state.bodies.get(handle)?.position().translation)
    }

    /// World-space orientation of a body.
    pub fn rotation(&self, node_id: NodeId) -> Option<glam::Quat> {
        let state = self.state.lock().unwrap();
        let handle = state.node_to_body.get(&node_id).copied()?;
        Some(state.bodies.get(handle)?.position().rotation)
    }

    /// World-space position and orientation of a body.
    pub fn body_transform(&self, node_id: NodeId) -> Option<(glam::Vec3, glam::Quat)> {
        let state = self.state.lock().unwrap();
        let handle = state.node_to_body.get(&node_id).copied()?;
        let body = state.bodies.get(handle)?;
        let pose = body.position();
        Some((pose.translation, pose.rotation))
    }

    /// Current linear velocity of a body.
    pub fn linear_velocity(&self, node_id: NodeId) -> Option<glam::Vec3> {
        let state = self.state.lock().unwrap();
        let handle = state.node_to_body.get(&node_id).copied()?;
        Some(state.bodies.get(handle)?.linvel())
    }

    /// Current angular velocity of a body (rad/s, axis-angle in world space).
    pub fn angular_velocity(&self, node_id: NodeId) -> Option<glam::Vec3> {
        let state = self.state.lock().unwrap();
        let handle = state.node_to_body.get(&node_id).copied()?;
        Some(state.bodies.get(handle)?.angvel())
    }

    // ------------------------------------------------------------------
    // Spatial queries
    // ------------------------------------------------------------------

    /// Cast a ray and return the nearest hit within `max_distance`, if any.
    ///
    /// `direction` does not need to be unit length: the returned `distance` is
    /// always in world units along `direction.normalize()`.
    pub fn raycast(
        &self,
        origin: glam::Vec3,
        direction: glam::Vec3,
        max_distance: f32,
    ) -> Option<RapierRaycastHit> {
        self.raycast_filtered(origin, direction, max_distance, &[])
    }

    /// Like [`raycast`](Self::raycast) but ignores any collider attached to a
    /// node in `exclude`.
    pub fn raycast_excluding(
        &self,
        origin: glam::Vec3,
        direction: glam::Vec3,
        max_distance: f32,
        exclude: &[viewport_lib::NodeId],
    ) -> Option<RapierRaycastHit> {
        self.raycast_filtered(origin, direction, max_distance, exclude)
    }

    /// Cast a ray and return every collider along its path, sorted by distance.
    pub fn raycast_all(
        &self,
        origin: glam::Vec3,
        direction: glam::Vec3,
        max_distance: f32,
    ) -> Vec<RapierRaycastHit> {
        use parry3d::query::Ray;

        let state = self.state.lock().unwrap();
        let dir = direction.normalize_or_zero();
        if dir == glam::Vec3::ZERO {
            return Vec::new();
        }
        let ray = Ray::new(origin, dir);
        let query_pipeline = state.broad_phase.as_query_pipeline(
            state.narrow_phase.query_dispatcher(),
            &state.bodies,
            &state.colliders,
            rapier3d::pipeline::QueryFilter::default(),
        );

        let mut hits: Vec<RapierRaycastHit> = query_pipeline
            .intersect_ray(ray, max_distance, true)
            .filter_map(|(handle, _collider, intersection)| {
                let node = state.node_for_collider(handle)?;
                Some(RapierRaycastHit {
                    node,
                    distance: intersection.time_of_impact,
                    point: origin + dir * intersection.time_of_impact,
                    normal: intersection.normal,
                })
            })
            .collect();
        hits.sort_by(|a, b| a.distance.partial_cmp(&b.distance).unwrap_or(std::cmp::Ordering::Equal));
        hits
    }

    /// Sweep a sphere from `start` along `direction` for up to `max_distance`
    /// and return the nearest hit, if any.
    pub fn cast_sphere(
        &self,
        start: glam::Vec3,
        radius: f32,
        direction: glam::Vec3,
        max_distance: f32,
    ) -> Option<RapierShapeCastHit> {
        use parry3d::query::ShapeCastOptions;
        use parry3d::shape::Ball;
        use rapier3d::math::Pose;

        let state = self.state.lock().unwrap();
        let dir = direction.normalize_or_zero();
        if dir == glam::Vec3::ZERO {
            return None;
        }
        let shape = Ball::new(radius);
        let pose = Pose { rotation: glam::Quat::IDENTITY, translation: start };
        let velocity = dir;
        let options = ShapeCastOptions {
            max_time_of_impact: max_distance,
            ..Default::default()
        };
        let query_pipeline = state.broad_phase.as_query_pipeline(
            state.narrow_phase.query_dispatcher(),
            &state.bodies,
            &state.colliders,
            rapier3d::pipeline::QueryFilter::default(),
        );
        let (handle, hit) = query_pipeline.cast_shape(&pose, velocity, &shape, options)?;
        let node = state.node_for_collider(handle)?;
        Some(RapierShapeCastHit {
            node,
            time_of_impact: hit.time_of_impact,
            point: hit.witness1,
            normal: hit.normal1,
        })
    }

    /// Return every node whose collider overlaps a sphere at `center` with
    /// the given `radius`.
    pub fn overlap_sphere(
        &self,
        center: glam::Vec3,
        radius: f32,
    ) -> Vec<viewport_lib::NodeId> {
        use parry3d::shape::Ball;
        use rapier3d::math::Pose;

        let state = self.state.lock().unwrap();
        let shape = Ball::new(radius);
        let pose = Pose { rotation: glam::Quat::IDENTITY, translation: center };
        let query_pipeline = state.broad_phase.as_query_pipeline(
            state.narrow_phase.query_dispatcher(),
            &state.bodies,
            &state.colliders,
            rapier3d::pipeline::QueryFilter::default(),
        );
        query_pipeline
            .intersect_shape(pose, &shape)
            .filter_map(|(handle, _collider)| state.node_for_collider(handle))
            .collect()
    }

    fn raycast_filtered(
        &self,
        origin: glam::Vec3,
        direction: glam::Vec3,
        max_distance: f32,
        exclude: &[viewport_lib::NodeId],
    ) -> Option<RapierRaycastHit> {
        use parry3d::query::Ray;

        let state = self.state.lock().unwrap();
        let dir = direction.normalize_or_zero();
        if dir == glam::Vec3::ZERO {
            return None;
        }

        // Materialise the exclusion list as collider handles so the predicate
        // closure does not need to take a lock or re-walk the maps per call.
        let excluded: Vec<rapier3d::geometry::ColliderHandle> = exclude
            .iter()
            .filter_map(|id| state.node_to_collider.get(id).copied())
            .collect();
        let predicate = move |handle: rapier3d::geometry::ColliderHandle,
                              _collider: &rapier3d::geometry::Collider|
              -> bool { !excluded.contains(&handle) };

        let filter = rapier3d::pipeline::QueryFilter {
            predicate: Some(&predicate),
            ..Default::default()
        };
        let query_pipeline = state.broad_phase.as_query_pipeline(
            state.narrow_phase.query_dispatcher(),
            &state.bodies,
            &state.colliders,
            filter,
        );
        let ray = Ray::new(origin, dir);
        let (handle, intersection) =
            query_pipeline.cast_ray_and_get_normal(&ray, max_distance, true)?;
        let node = state.node_for_collider(handle)?;
        Some(RapierRaycastHit {
            node,
            distance: intersection.time_of_impact,
            point: origin + dir * intersection.time_of_impact,
            normal: intersection.normal,
        })
    }

    // ------------------------------------------------------------------
    // Plugin extraction
    // ------------------------------------------------------------------

    /// Split into the two `RuntimePlugin` objects the runtime needs.
    ///
    /// Call after all bodies have been registered (or register bodies at runtime
    /// through the clone of `self` you keep).
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
// Prepare plugin: sync scene -> Rapier for fixed and kinematic bodies
// ---------------------------------------------------------------------------

pub struct RapierPreparePlugin {
    state: Arc<Mutex<RapierState>>,
}

impl RuntimePlugin for RapierPreparePlugin {
    fn priority(&self) -> i32 {
        phase::PREPARE
    }

    fn step(&mut self, ctx: &mut RuntimeStepContext<'_>) {
        let mut state = self.state.lock().unwrap();

        // Snapshot the (node, handle, is_kinematic) triples up front so we can
        // mutate bodies in the loop without aliasing the map.
        let synced: Vec<(NodeId, rapier3d::dynamics::RigidBodyHandle, bool)> = state
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
            let pose = mat4_to_pose(node.world_transform());
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
// Simulate plugin: step Rapier, write back dynamic body transforms
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
        let mut state = self.state.lock().unwrap();

        state.integration_params.dt = dt;

        let collector = EventCollector::default();

        // Destructure to satisfy the borrow checker: pipeline.step needs &mut pipeline
        // plus independent borrows of all other fields simultaneously.
        let crate::state::RapierState {
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

        // Write dynamic body transforms back to viewport-lib scene.
        let dynamic_bodies: Vec<(NodeId, glam::Affine3A)> = state
            .node_to_body
            .iter()
            .filter_map(|(&node_id, &handle)| {
                let body = state.bodies.get(handle)?;
                if body.is_dynamic() {
                    Some((node_id, pose_to_affine(body.position())))
                } else {
                    None
                }
            })
            .collect();

        for (node_id, transform) in dynamic_bodies {
            ctx.writeback.set_preserve_scale(node_id, transform);
        }

        // Translate raw collision events to NodeId-based events and emit them.
        let collisions = collector.collisions.into_inner().unwrap();
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

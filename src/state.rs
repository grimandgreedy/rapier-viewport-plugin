use indexmap::IndexMap;

use rapier3d::dynamics::{
    CCDSolver, ImpulseJointSet, IntegrationParameters, IslandManager, MultibodyJointSet, RigidBody,
    RigidBodyHandle, RigidBodySet,
};
use rapier3d::geometry::{
    BroadPhaseBvh, Collider, ColliderHandle, ColliderSet, NarrowPhase,
};
use rapier3d::math::Vector;
use rapier3d::pipeline::PhysicsPipeline;
use viewport_lib::NodeId;

/// Shared world state: every rapier set the simulation needs, plus the
/// `NodeId`-to-handle maps that keep scene identity in sync with rapier's
/// handle namespace.
///
/// The four maps are [`IndexMap`]s (insertion-ordered) rather than
/// [`HashMap`](std::collections::HashMap) so iteration order is the same
/// on every run. The simulate plugin walks `node_to_body` to drive its
/// writeback and event-translation passes, so this gives deterministic
/// writeback and `RapierContactEvent` order within a single process.
///
/// For cross-platform floating-point determinism (different CPUs
/// producing the same bits), enable rapier's `enhanced-determinism` Cargo
/// feature on the `rapier3d` dependency. That is a compile-time feature,
/// not a runtime switch.
///
/// Fields are public; reach in via [`crate::RapierPlugin::with_state_mut`]
/// for anything not covered by the curated handle API.
pub struct RapierState {
    pub integration_params: IntegrationParameters,
    pub pipeline: PhysicsPipeline,
    pub islands: IslandManager,
    pub broad_phase: BroadPhaseBvh,
    pub narrow_phase: NarrowPhase,
    pub bodies: RigidBodySet,
    pub colliders: ColliderSet,
    pub impulse_joints: ImpulseJointSet,
    pub multibody_joints: MultibodyJointSet,
    pub ccd_solver: CCDSolver,
    pub gravity: Vector,

    pub node_to_body: IndexMap<NodeId, RigidBodyHandle>,
    pub body_to_node: IndexMap<RigidBodyHandle, NodeId>,
    pub node_to_collider: IndexMap<NodeId, ColliderHandle>,
    pub collider_to_node: IndexMap<ColliderHandle, NodeId>,
}

impl RapierState {
    pub fn new(gravity: Vector) -> Self {
        Self {
            integration_params: IntegrationParameters::default(),
            pipeline: PhysicsPipeline::new(),
            islands: IslandManager::new(),
            broad_phase: BroadPhaseBvh::new(),
            narrow_phase: NarrowPhase::new(),
            bodies: RigidBodySet::new(),
            colliders: ColliderSet::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            gravity,
            node_to_body: IndexMap::new(),
            body_to_node: IndexMap::new(),
            node_to_collider: IndexMap::new(),
            collider_to_node: IndexMap::new(),
        }
    }

    /// Insert a rigid body + collider pair and bind them to a scene node. The
    /// body and collider should already be configured (use rapier's
    /// `RigidBodyBuilder` / `ColliderBuilder`).
    pub fn add_body(&mut self, node_id: NodeId, body: RigidBody, collider: Collider) {
        let body_handle = self.bodies.insert(body);
        let collider_handle =
            self.colliders
                .insert_with_parent(collider, body_handle, &mut self.bodies);
        self.node_to_body.insert(node_id, body_handle);
        self.body_to_node.insert(body_handle, node_id);
        self.node_to_collider.insert(node_id, collider_handle);
        self.collider_to_node.insert(collider_handle, node_id);
    }

    /// Remove the body bound to `node_id` (and its collider). No-op if the
    /// node was never registered.
    ///
    /// Uses `shift_remove` on every map so the insertion order of the
    /// remaining entries is preserved; this is what gives the simulate
    /// plugin's writeback a deterministic order across runs.
    pub fn remove_body(&mut self, node_id: NodeId) {
        let Some(body_handle) = self.node_to_body.shift_remove(&node_id) else {
            return;
        };
        self.body_to_node.shift_remove(&body_handle);

        if let Some(collider_handle) = self.node_to_collider.shift_remove(&node_id) {
            self.collider_to_node.shift_remove(&collider_handle);
        }

        self.bodies.remove(
            body_handle,
            &mut self.islands,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            true,
        );
    }

    /// Remove every registered body.
    pub fn clear_bodies(&mut self) {
        let ids: Vec<NodeId> = self.node_to_body.keys().copied().collect();
        for id in ids {
            self.remove_body(id);
        }
    }

    /// `NodeId` bound to a collider handle, if any.
    pub fn node_for_collider(&self, handle: ColliderHandle) -> Option<NodeId> {
        self.collider_to_node.get(&handle).copied()
    }

    /// `NodeId` bound to a rigid body handle, if any.
    pub fn node_for_body(&self, handle: RigidBodyHandle) -> Option<NodeId> {
        self.body_to_node.get(&handle).copied()
    }
}

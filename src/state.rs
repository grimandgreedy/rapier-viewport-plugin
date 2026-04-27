use std::collections::HashMap;

use rapier3d::dynamics::IntegrationParameters;
use rapier3d::dynamics::{
    CCDSolver, ImpulseJointSet, IslandManager, MultibodyJointSet, RigidBodyHandle, RigidBodySet,
};
use rapier3d::geometry::{BroadPhaseBvh, ColliderHandle, ColliderSet, NarrowPhase};
use rapier3d::math::Vector;
use rapier3d::pipeline::PhysicsPipeline;
use viewport_lib::NodeId;

use crate::body::{RapierBodyDesc, RapierBodyKind, RapierColliderShape};
use crate::convert::affine_to_pose;
use crate::error::RapierError;

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

    pub node_to_body: HashMap<NodeId, RigidBodyHandle>,
    pub body_to_node: HashMap<RigidBodyHandle, NodeId>,
    pub node_to_collider: HashMap<NodeId, ColliderHandle>,
    pub collider_to_node: HashMap<ColliderHandle, NodeId>,
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
            node_to_body: HashMap::new(),
            body_to_node: HashMap::new(),
            node_to_collider: HashMap::new(),
            collider_to_node: HashMap::new(),
        }
    }

    pub fn add_body(
        &mut self,
        desc: &RapierBodyDesc,
        initial_transform: glam::Affine3A,
    ) -> Result<(), RapierError> {
        use rapier3d::dynamics::RigidBodyBuilder;
        use rapier3d::pipeline::ActiveEvents;

        // Build the collider first so we can fail without leaking a rigid body
        // into the world.
        let collider_builder = build_collider(&desc.shape)?
            .restitution(desc.restitution)
            .friction(desc.friction)
            .density(desc.density);

        let collider_builder = if desc.active_events {
            collider_builder.active_events(ActiveEvents::COLLISION_EVENTS)
        } else {
            collider_builder
        };

        let pose = affine_to_pose(initial_transform);

        let rb = match desc.kind {
            RapierBodyKind::Dynamic => RigidBodyBuilder::dynamic()
                .pose(pose)
                .gravity_scale(desc.gravity_scale)
                .linvel(desc.initial_linear_velocity)
                .angvel(desc.initial_angular_velocity)
                .ccd_enabled(desc.ccd_enabled)
                .build(),
            RapierBodyKind::Fixed => RigidBodyBuilder::fixed().pose(pose).build(),
            RapierBodyKind::KinematicPosition => RigidBodyBuilder::kinematic_position_based()
                .pose(pose)
                .build(),
        };

        let body_handle = self.bodies.insert(rb);
        let collider_handle = self.colliders.insert_with_parent(
            collider_builder.build(),
            body_handle,
            &mut self.bodies,
        );

        self.node_to_body.insert(desc.node_id, body_handle);
        self.body_to_node.insert(body_handle, desc.node_id);
        self.node_to_collider.insert(desc.node_id, collider_handle);
        self.collider_to_node.insert(collider_handle, desc.node_id);
        Ok(())
    }

    pub fn remove_body(&mut self, node_id: NodeId) {
        let Some(body_handle) = self.node_to_body.remove(&node_id) else {
            return;
        };
        self.body_to_node.remove(&body_handle);

        if let Some(collider_handle) = self.node_to_collider.remove(&node_id) {
            self.collider_to_node.remove(&collider_handle);
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

    pub fn clear_bodies(&mut self) {
        let ids: Vec<NodeId> = self.node_to_body.keys().copied().collect();
        for id in ids {
            self.remove_body(id);
        }
    }

    /// Look up the `NodeId` for a given collider handle.
    pub fn node_for_collider(&self, handle: ColliderHandle) -> Option<NodeId> {
        self.collider_to_node.get(&handle).copied()
    }
}

fn build_collider(
    shape: &RapierColliderShape,
) -> Result<rapier3d::geometry::ColliderBuilder, RapierError> {
    use rapier3d::geometry::ColliderBuilder;

    Ok(match shape {
        RapierColliderShape::Ball { radius } => ColliderBuilder::ball(*radius),
        RapierColliderShape::Cuboid { half_extents } => {
            ColliderBuilder::cuboid(half_extents.x, half_extents.y, half_extents.z)
        }
        RapierColliderShape::CapsuleZ {
            half_height,
            radius,
        } => ColliderBuilder::capsule_z(*half_height, *radius),
        RapierColliderShape::HalfSpace { outward_normal } => {
            use parry3d::shape::HalfSpace;
            use rapier3d::geometry::SharedShape;
            ColliderBuilder::new(SharedShape::new(HalfSpace::new(outward_normal.normalize())))
        }
        RapierColliderShape::ConvexHull { points } => {
            ColliderBuilder::convex_hull(points).ok_or(RapierError::InvalidConvexHull)?
        }
        RapierColliderShape::TriMesh { vertices, indices } => {
            ColliderBuilder::trimesh(vertices.clone(), indices.clone())
                .map_err(|_| RapierError::InvalidTriMesh)?
        }
    })
}

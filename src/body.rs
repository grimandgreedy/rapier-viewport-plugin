use viewport_lib::NodeId;

/// How a body participates in the simulation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RapierBodyKind {
    /// Fully simulated; affected by gravity, forces, and impulses.
    #[default]
    Dynamic,
    /// Immovable; acts as an obstacle for dynamic bodies. Position is sourced
    /// from the bound scene node each `Prepare` step, so moving the node with
    /// a gizmo or scripted animation teleports the collider too.
    Fixed,
    /// Transform is driven externally each frame (read from the scene node).
    /// Rapier computes velocity implicitly from position changes.
    ///
    /// Only the translation and rotation of the scene node's world transform
    /// are forwarded to the collider. Scaling a kinematic node does not resize
    /// the collider; rebuild the body with a new shape if you need that.
    KinematicPosition,
}

/// Collision shape used by a body's collider.
#[derive(Debug, Clone)]
pub enum RapierColliderShape {
    Ball { radius: f32 },
    Cuboid { half_extents: glam::Vec3 },
    /// Capsule aligned with the Z axis.
    CapsuleZ { half_height: f32, radius: f32 },
    /// Infinite half-space whose surface passes through the origin.
    HalfSpace { outward_normal: glam::Vec3 },
    /// Convex hull computed from the given point cloud.
    ConvexHull { points: Vec<glam::Vec3> },
    /// Arbitrary triangle mesh. Only suitable for Fixed / KinematicPosition bodies.
    TriMesh {
        vertices: Vec<glam::Vec3>,
        indices: Vec<[u32; 3]>,
    },
}

/// Descriptor used to register a body with [`crate::RapierPlugin`].
#[derive(Debug, Clone)]
pub struct RapierBodyDesc {
    /// The viewport-lib scene node this body drives / is driven by.
    pub node_id: NodeId,
    pub kind: RapierBodyKind,
    pub shape: RapierColliderShape,
    /// Restitution (bounciness). 0 = no bounce, 1 = perfectly elastic.
    pub restitution: f32,
    /// Coulomb friction coefficient.
    pub friction: f32,
    /// Collider density used to compute mass. Ignored for Fixed bodies.
    pub density: f32,
    /// Multiplier on the world gravity vector. Default 1.0.
    pub gravity_scale: f32,
    /// Enable continuous collision detection (tunnelling prevention).
    pub ccd_enabled: bool,
    /// Initial linear velocity (world space). Only applied to Dynamic bodies.
    pub initial_linear_velocity: glam::Vec3,
    /// Initial angular velocity (world space, rad/s axis-angle). Dynamic only.
    pub initial_angular_velocity: glam::Vec3,
    /// Emit collision events for this body. Required to receive events via
    /// [`crate::RapierContactEvent`].
    pub active_events: bool,
}

impl RapierBodyDesc {
    pub fn dynamic(node_id: NodeId, shape: RapierColliderShape) -> Self {
        Self {
            node_id,
            kind: RapierBodyKind::Dynamic,
            shape,
            restitution: 0.0,
            friction: 0.5,
            density: 1.0,
            gravity_scale: 1.0,
            ccd_enabled: false,
            initial_linear_velocity: glam::Vec3::ZERO,
            initial_angular_velocity: glam::Vec3::ZERO,
            active_events: true,
        }
    }

    pub fn fixed(node_id: NodeId, shape: RapierColliderShape) -> Self {
        Self {
            node_id,
            kind: RapierBodyKind::Fixed,
            shape,
            restitution: 0.0,
            friction: 0.5,
            density: 1.0,
            gravity_scale: 1.0,
            ccd_enabled: false,
            initial_linear_velocity: glam::Vec3::ZERO,
            initial_angular_velocity: glam::Vec3::ZERO,
            active_events: false,
        }
    }

    pub fn kinematic(node_id: NodeId, shape: RapierColliderShape) -> Self {
        Self {
            node_id,
            kind: RapierBodyKind::KinematicPosition,
            shape,
            restitution: 0.0,
            friction: 0.5,
            density: 1.0,
            gravity_scale: 1.0,
            ccd_enabled: false,
            initial_linear_velocity: glam::Vec3::ZERO,
            initial_angular_velocity: glam::Vec3::ZERO,
            active_events: false,
        }
    }

    pub fn with_restitution(mut self, r: f32) -> Self {
        self.restitution = r;
        self
    }

    pub fn with_friction(mut self, f: f32) -> Self {
        self.friction = f;
        self
    }

    pub fn with_density(mut self, d: f32) -> Self {
        self.density = d;
        self
    }

    pub fn with_gravity_scale(mut self, s: f32) -> Self {
        self.gravity_scale = s;
        self
    }

    pub fn with_ccd(mut self) -> Self {
        self.ccd_enabled = true;
        self
    }

    pub fn with_initial_linear_velocity(mut self, v: glam::Vec3) -> Self {
        self.initial_linear_velocity = v;
        self
    }

    pub fn with_initial_angular_velocity(mut self, v: glam::Vec3) -> Self {
        self.initial_angular_velocity = v;
        self
    }
}

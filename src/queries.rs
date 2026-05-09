//! Spatial query results returned from [`crate::RapierPlugin`] raycast and
//! shape cast methods.

use viewport_lib::NodeId;

/// A single raycast hit against the physics world.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RapierRaycastHit {
    /// Scene node bound to the collider that was hit.
    pub node: NodeId,
    /// Distance along the ray at which the hit occurred (`0.0` means the ray
    /// origin is already inside the shape).
    pub distance: f32,
    /// Hit point in world space.
    pub point: glam::Vec3,
    /// Surface normal at the hit point.
    pub normal: glam::Vec3,
}

/// A single shape-cast hit. `time_of_impact` is the fraction of `velocity *
/// max_toi` travelled before contact: multiply by the cast direction and add
/// to the start pose to get the hit pose.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RapierShapeCastHit {
    /// Scene node bound to the collider that was hit.
    pub node: NodeId,
    /// Time of impact along the cast trajectory.
    pub time_of_impact: f32,
    /// Contact point on the world-space collider that was hit.
    pub point: glam::Vec3,
    /// Surface normal at the hit point (on the world collider, world space).
    pub normal: glam::Vec3,
}

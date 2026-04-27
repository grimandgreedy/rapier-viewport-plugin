use std::sync::Mutex;

use rapier3d::geometry::{ColliderSet, CollisionEvent, ContactPair};
use rapier3d::dynamics::RigidBodySet;
use rapier3d::math::Real;
use rapier3d::pipeline::EventHandler;
use viewport_lib::NodeId;

/// A collision start/stop event translated to `NodeId` pairs and emittable
/// onto the viewport-lib event bus.
#[derive(Debug, Clone)]
pub struct RapierContactEvent {
    pub node_a: NodeId,
    pub node_b: NodeId,
    pub started: bool,
}

/// Collects raw `CollisionEvent`s during `PhysicsPipeline::step` and stores
/// them for translation to `RapierContactEvent` after the step completes.
///
/// Uses interior mutability so it satisfies `EventHandler: Sync`.
#[derive(Default)]
pub struct EventCollector {
    pub collisions: Mutex<Vec<CollisionEvent>>,
}

impl EventHandler for EventCollector {
    fn handle_collision_event(
        &self,
        _bodies: &RigidBodySet,
        _colliders: &ColliderSet,
        event: CollisionEvent,
        _contact_pair: Option<&ContactPair>,
    ) {
        self.collisions.lock().unwrap().push(event);
    }

    fn handle_contact_force_event(
        &self,
        _dt: Real,
        _bodies: &RigidBodySet,
        _colliders: &ColliderSet,
        _contact_pair: &ContactPair,
        _total_force_magnitude: Real,
    ) {
    }
}

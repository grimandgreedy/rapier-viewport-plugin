//! `add_body` / `remove_body` / `clear_bodies` maintain the NodeId maps
//! atomically with the underlying rapier sets.

mod common;

use common::add_node_at;
use rapier_viewport_plugin::{
    rapier3d::prelude::{ColliderBuilder, RigidBodyBuilder},
    RapierPlugin,
};
use viewport_lib::Scene;

fn dyn_body() -> rapier_viewport_plugin::rapier3d::dynamics::RigidBody {
    RigidBodyBuilder::dynamic().build()
}

fn ball_collider() -> rapier_viewport_plugin::rapier3d::geometry::Collider {
    ColliderBuilder::ball(0.5).build()
}

#[test]
fn add_body_populates_both_maps() {
    let mut scene = Scene::new();
    let mut plugin = RapierPlugin::new();
    let id = add_node_at(&mut scene, glam::Vec3::ZERO);

    plugin.add_body(id, dyn_body(), ball_collider());

    let body_handle = plugin.body_handle(id);
    let collider_handle = plugin.collider_handle(id);
    assert!(body_handle.is_some(), "body handle should be registered");
    assert!(collider_handle.is_some(), "collider handle should be registered");

    plugin.with_state(|s| {
        let h = body_handle.unwrap();
        assert_eq!(s.node_for_body(h), Some(id), "reverse body map missing entry");
        let ch = collider_handle.unwrap();
        assert_eq!(s.node_for_collider(ch), Some(id), "reverse collider map missing entry");
        assert_eq!(s.bodies.len(), 1);
        assert_eq!(s.colliders.len(), 1);
    });
}

#[test]
fn remove_body_clears_both_maps_and_rapier_sets() {
    let mut scene = Scene::new();
    let mut plugin = RapierPlugin::new();
    let id = add_node_at(&mut scene, glam::Vec3::ZERO);

    plugin.add_body(id, dyn_body(), ball_collider());
    let body_handle = plugin.body_handle(id).unwrap();
    let collider_handle = plugin.collider_handle(id).unwrap();

    plugin.remove_body(id);

    assert!(plugin.body_handle(id).is_none());
    assert!(plugin.collider_handle(id).is_none());

    plugin.with_state(|s| {
        assert_eq!(s.node_for_body(body_handle), None, "reverse body map should be empty");
        assert_eq!(
            s.node_for_collider(collider_handle),
            None,
            "reverse collider map should be empty",
        );
        assert_eq!(s.bodies.len(), 0, "rapier body set should be empty");
        assert_eq!(s.colliders.len(), 0, "rapier collider set should be empty");
    });
}

#[test]
fn remove_unregistered_node_is_noop() {
    let mut scene = Scene::new();
    let mut plugin = RapierPlugin::new();

    let registered = add_node_at(&mut scene, glam::Vec3::ZERO);
    let unregistered = add_node_at(&mut scene, glam::Vec3::ZERO);

    plugin.add_body(registered, dyn_body(), ball_collider());

    // Removing a node that was never registered must not disturb a sibling.
    plugin.remove_body(unregistered);

    assert!(plugin.body_handle(registered).is_some());
    plugin.with_state(|s| assert_eq!(s.bodies.len(), 1));
}

#[test]
fn clear_bodies_removes_every_registration() {
    let mut scene = Scene::new();
    let mut plugin = RapierPlugin::new();

    let n1 = add_node_at(&mut scene, glam::Vec3::ZERO);
    let n2 = add_node_at(&mut scene, glam::Vec3::ZERO);
    let n3 = add_node_at(&mut scene, glam::Vec3::ZERO);

    plugin.add_body(n1, dyn_body(), ball_collider());
    plugin.add_body(n2, dyn_body(), ball_collider());
    plugin.add_body(n3, dyn_body(), ball_collider());

    plugin.clear_bodies();

    for &id in &[n1, n2, n3] {
        assert!(plugin.body_handle(id).is_none());
        assert!(plugin.collider_handle(id).is_none());
    }
    plugin.with_state(|s| {
        assert_eq!(s.bodies.len(), 0);
        assert_eq!(s.colliders.len(), 0);
        assert!(s.node_to_body.is_empty());
        assert!(s.body_to_node.is_empty());
        assert!(s.node_to_collider.is_empty());
        assert!(s.collider_to_node.is_empty());
    });
}

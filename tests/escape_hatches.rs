//! The `with_*_mut` and `with_state` escape hatches return `None` for unknown
//! nodes, return the closure's value for known ones, and mutations applied
//! inside the closure persist in the rapier state.

mod common;

use common::add_node_at;
use rapier_viewport_plugin::{
    rapier3d::prelude::{ColliderBuilder, RigidBodyBuilder},
    RapierPlugin,
};
use viewport_lib::Scene;

#[test]
fn with_body_mut_returns_none_for_unknown_node() {
    let mut scene = Scene::new();
    let mut plugin = RapierPlugin::new();

    // Make a node but never register it with the plugin.
    let id = add_node_at(&mut scene, glam::Vec3::ZERO);

    let result = plugin.with_body_mut(id, |_b| 7_i32);
    assert!(result.is_none());
}

#[test]
fn with_body_mut_runs_closure_and_returns_value_for_known_node() {
    let mut scene = Scene::new();
    let mut plugin = RapierPlugin::new();
    let id = add_node_at(&mut scene, glam::Vec3::ZERO);

    plugin.add_body(
        id,
        RigidBodyBuilder::dynamic().build(),
        ColliderBuilder::ball(0.5).build(),
    );

    let translation = plugin
        .with_body_mut(id, |b| b.position().translation)
        .expect("body should be present");
    assert_eq!(translation, glam::Vec3::ZERO);
}

#[test]
fn with_body_mut_mutations_persist() {
    let mut scene = Scene::new();
    let mut plugin = RapierPlugin::new();
    let id = add_node_at(&mut scene, glam::Vec3::ZERO);

    plugin.add_body(
        id,
        RigidBodyBuilder::dynamic().build(),
        ColliderBuilder::ball(0.5).build(),
    );

    plugin.with_body_mut(id, |b| {
        b.apply_impulse(glam::Vec3::new(10.0, 0.0, 0.0), true);
    });

    let linvel = plugin
        .with_body_mut(id, |b| b.linvel())
        .expect("body should still be present");
    assert!(linvel.x > 0.0, "impulse should produce positive x velocity");
}

#[test]
fn with_collider_mut_returns_none_for_unknown_node() {
    let mut scene = Scene::new();
    let mut plugin = RapierPlugin::new();
    let id = add_node_at(&mut scene, glam::Vec3::ZERO);
    let result: Option<()> = plugin.with_collider_mut(id, |_| ());
    assert!(result.is_none());
}

#[test]
fn with_collider_mut_mutations_persist() {
    let mut scene = Scene::new();
    let mut plugin = RapierPlugin::new();
    let id = add_node_at(&mut scene, glam::Vec3::ZERO);

    plugin.add_body(
        id,
        RigidBodyBuilder::dynamic().build(),
        ColliderBuilder::ball(0.5).build(),
    );

    plugin.with_collider_mut(id, |c| c.set_restitution(0.9));

    let restitution = plugin
        .with_collider_mut(id, |c| c.restitution())
        .expect("collider should be present");
    assert!((restitution - 0.9).abs() < 1e-5);
}

#[test]
fn with_state_mut_runs_and_returns_value() {
    let mut scene = Scene::new();
    let mut plugin = RapierPlugin::new();
    let id = add_node_at(&mut scene, glam::Vec3::ZERO);
    plugin.add_body(
        id,
        RigidBodyBuilder::dynamic().build(),
        ColliderBuilder::ball(0.5).build(),
    );

    let body_count = plugin.with_state_mut(|s| s.bodies.len());
    assert_eq!(body_count, 1);
}

#[test]
fn with_state_runs_without_lock_held_after_return() {
    let mut scene = Scene::new();
    let mut plugin = RapierPlugin::new();
    let id = add_node_at(&mut scene, glam::Vec3::ZERO);
    plugin.add_body(
        id,
        RigidBodyBuilder::dynamic().build(),
        ColliderBuilder::ball(0.5).build(),
    );

    // Read body count, release lock, then call another plugin method that
    // re-acquires the lock. If `with_state` held the lock across return this
    // second call would deadlock.
    let count = plugin.with_state(|s| s.bodies.len());
    assert_eq!(count, 1);
    let _ = plugin.body_handle(id);
}

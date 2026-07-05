//! If a closure passed to one of the escape hatches panics while holding
//! the lock, the plugin's mutex becomes poisoned. The next plugin call
//! should still succeed -- the `lock_state` helper recovers transparently
//! via `into_inner`.

mod common;

use std::thread;

use common::add_node_at;
use rapier_viewport_plugin::{
    rapier3d::prelude::{ColliderBuilder, RigidBodyBuilder},
    RapierPlugin,
};
use viewport_lib::Scene;

#[test]
fn plugin_keeps_working_after_panic_inside_with_state() {
    let mut scene = Scene::new();
    let mut plugin = RapierPlugin::new();
    let id = add_node_at(&mut scene, glam::Vec3::ZERO);
    plugin.add_body(
        id,
        RigidBodyBuilder::dynamic().build(),
        ColliderBuilder::ball(0.5).build(),
    );

    // Run a panicking closure on a worker thread so the panic doesn't
    // unwind the test. The worker holds the mutex during `with_state`,
    // and the panic poisons it on drop.
    let worker_plugin = plugin.clone();
    let join = thread::spawn(move || {
        worker_plugin.with_state(|_s| {
            panic!("intentional test panic");
        });
    });
    let _ = join.join();

    // The mutex is now poisoned. The plugin must transparently recover.
    let handle = plugin.body_handle(id);
    assert!(handle.is_some(), "lookup must work after a poisoned mutex");

    let count = plugin.with_state(|s| s.bodies.len());
    assert_eq!(count, 1, "state must be intact after recovery");
}

#[test]
fn plugin_keeps_working_after_panic_inside_with_body_mut() {
    let mut scene = Scene::new();
    let mut plugin = RapierPlugin::new();
    let id = add_node_at(&mut scene, glam::Vec3::ZERO);
    plugin.add_body(
        id,
        RigidBodyBuilder::dynamic().build(),
        ColliderBuilder::ball(0.5).build(),
    );

    let worker_plugin = plugin.clone();
    let join = thread::spawn(move || {
        let mut p = worker_plugin;
        p.with_body_mut(id, |_b| {
            panic!("intentional test panic");
        });
    });
    let _ = join.join();

    // Subsequent reads and writes must both still work.
    let pos = plugin
        .with_body_mut(id, |b| b.position().translation)
        .expect("body should still be reachable");
    assert_eq!(pos, glam::Vec3::ZERO);

    plugin.with_body_mut(id, |b| {
        b.apply_impulse(glam::Vec3::new(1.0, 0.0, 0.0), true);
    });
    let linvel = plugin.with_body_mut(id, |b| b.linvel()).unwrap();
    assert!(linvel.x > 0.0);
}

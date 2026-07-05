//! The simulate plugin's per-step writeback order is the order bodies were
//! registered, not a hash-table order that varies between runs.
//!
//! This is what `IndexMap` (vs `HashMap`) buys us. Combined with rapier's
//! own determinism mode, it enables bit-reproducible runs across processes.

mod common;

use common::{add_node_at, make_runtime};
use rapier_viewport_plugin::{
    rapier3d::prelude::{ColliderBuilder, RigidBodyBuilder},
    RapierPlugin,
};
use viewport_lib::{NodeId, RuntimeFrameContext, Scene, Selection};

const DT: f32 = 1.0 / 60.0;

fn frame_ctx() -> RuntimeFrameContext {
    let mut ctx = RuntimeFrameContext::default();
    ctx.dt = DT;
    ctx
}

#[test]
fn node_to_body_iterates_in_insertion_order() {
    let mut scene = Scene::new();
    let mut plugin = RapierPlugin::new();

    // Register bodies in a fixed sequence. The map's iter order should be
    // exactly that sequence.
    let mut inserted = Vec::new();
    for i in 0..16 {
        let id = add_node_at(&mut scene, glam::Vec3::new(i as f32, 0.0, 0.0));
        plugin.add_body(
            id,
            RigidBodyBuilder::dynamic().build(),
            ColliderBuilder::ball(0.5).build(),
        );
        inserted.push(id);
    }

    let iterated: Vec<NodeId> = plugin.with_state(|s| s.node_to_body.keys().copied().collect());
    assert_eq!(
        iterated, inserted,
        "node_to_body iteration order should match insertion order"
    );
}

#[test]
fn remove_preserves_remaining_insertion_order() {
    let mut scene = Scene::new();
    let mut plugin = RapierPlugin::new();

    let ids: Vec<NodeId> = (0..8)
        .map(|i| {
            let id = add_node_at(&mut scene, glam::Vec3::new(i as f32, 0.0, 0.0));
            plugin.add_body(
                id,
                RigidBodyBuilder::dynamic().build(),
                ColliderBuilder::ball(0.5).build(),
            );
            id
        })
        .collect();

    // Remove a middle entry. The remaining entries should keep their
    // relative insertion order (this is `shift_remove`, not `swap_remove`).
    plugin.remove_body(ids[3]);

    let mut expected: Vec<NodeId> = ids.clone();
    expected.remove(3);

    let after: Vec<NodeId> = plugin.with_state(|s| s.node_to_body.keys().copied().collect());
    assert_eq!(after, expected, "shift_remove should preserve order");
}

#[test]
fn writeback_order_matches_insertion_order() {
    // Run a step and verify that the writeback iteration -- which the
    // simulate plugin drives off `node_to_body.iter()` -- produces the
    // bodies in the order they were registered.
    let mut scene = Scene::new();
    let mut selection = Selection::new();
    let mut plugin = RapierPlugin::new();

    let mut inserted = Vec::new();
    for i in 0..8 {
        let id = add_node_at(&mut scene, glam::Vec3::new(0.0, 0.0, 10.0 + i as f32));
        plugin.add_body(
            id,
            RigidBodyBuilder::dynamic()
                .translation(glam::Vec3::new(0.0, 0.0, 10.0 + i as f32))
                .build(),
            ColliderBuilder::ball(0.5).build(),
        );
        inserted.push(id);
    }

    let mut runtime = make_runtime(&plugin, 60.0);
    runtime.step(&mut scene, &mut selection, &frame_ctx());

    // The simulate plugin's dynamic-body collection walks node_to_body in
    // insertion order. We re-walk the same map here and assert it.
    let walked: Vec<NodeId> = plugin.with_state(|s| s.node_to_body.keys().copied().collect());
    assert_eq!(walked, inserted);
}

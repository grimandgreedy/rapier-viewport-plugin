//! `RapierSimulatePlugin` writes dynamic body transforms back to the scene
//! every step and leaves non-dynamic bodies alone.

mod common;

use common::{add_node_at, make_runtime};
use rapier_viewport_plugin::{
    rapier3d::prelude::{ColliderBuilder, RigidBodyBuilder},
    RapierPlugin,
};
use viewport_lib::{RuntimeFrameContext, Scene, Selection};

const DT: f32 = 1.0 / 60.0;

fn frame_ctx() -> RuntimeFrameContext {
    let mut ctx = RuntimeFrameContext::default();
    ctx.dt = DT;
    ctx
}

fn scene_position(scene: &Scene, id: viewport_lib::NodeId) -> glam::Vec3 {
    scene.node(id).unwrap().world_transform().col(3).truncate()
}

#[test]
fn dynamic_body_writeback_moves_scene_node_under_gravity() {
    let mut scene = Scene::new();
    let mut selection = Selection::new();
    let mut plugin = RapierPlugin::new();

    let id = add_node_at(&mut scene, glam::Vec3::new(0.0, 0.0, 10.0));
    plugin.add_body(
        id,
        RigidBodyBuilder::dynamic()
            .translation(glam::Vec3::new(0.0, 0.0, 10.0))
            .build(),
        ColliderBuilder::ball(0.5).build(),
    );

    let mut runtime = make_runtime(&plugin, 60.0);
    for _ in 0..60 {
        runtime.step(&mut scene, &mut selection, &frame_ctx());
    }

    let pos = scene_position(&scene, id);
    assert!(
        pos.z < 9.0,
        "dynamic body should have fallen due to gravity, got z={}",
        pos.z
    );
}

#[test]
fn fixed_body_scene_node_does_not_drift() {
    let mut scene = Scene::new();
    let mut selection = Selection::new();
    let mut plugin = RapierPlugin::new();

    let pos = glam::Vec3::new(2.0, 3.0, 4.0);
    let id = add_node_at(&mut scene, pos);
    plugin.add_body(
        id,
        RigidBodyBuilder::fixed().translation(pos).build(),
        ColliderBuilder::ball(0.5).build(),
    );

    let mut runtime = make_runtime(&plugin, 60.0);
    for _ in 0..30 {
        runtime.step(&mut scene, &mut selection, &frame_ctx());
    }

    let actual = scene_position(&scene, id);
    assert!(
        (actual - pos).length() < 1e-4,
        "fixed scene node should stay put, got {actual:?}",
    );
}

#[test]
fn kinematic_body_scene_position_is_user_controlled_not_writeback() {
    // The simulate plugin must not write back kinematic bodies. The user
    // drives the scene transform; the writeback should leave it alone.
    let mut scene = Scene::new();
    let mut selection = Selection::new();
    let mut plugin = RapierPlugin::new();

    let id = add_node_at(&mut scene, glam::Vec3::ZERO);
    plugin.add_body(
        id,
        RigidBodyBuilder::kinematic_position_based().build(),
        ColliderBuilder::ball(0.5).build(),
    );

    let mut runtime = make_runtime(&plugin, 60.0);
    runtime.step(&mut scene, &mut selection, &frame_ctx());

    // User updates the scene; if writeback ran on kinematic bodies, the
    // rapier-side position (still origin after one substep) would clobber
    // this back to (0, 0, 0).
    let target = glam::Vec3::new(3.0, 0.0, 0.0);
    scene.set_local_transform(id, glam::Mat4::from_translation(target));
    scene.update_transforms();
    runtime.step(&mut scene, &mut selection, &frame_ctx());

    let actual = scene_position(&scene, id);
    assert!(
        (actual - target).length() < 1e-3,
        "kinematic scene node should be user-controlled, got {actual:?}"
    );
}

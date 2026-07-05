//! `RapierPreparePlugin` mirrors the scene transform of `Fixed` and
//! `KinematicPosition` bodies into rapier each step, and leaves `Dynamic`
//! bodies alone.

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

#[test]
fn fixed_body_position_is_pulled_from_scene_at_first_step() {
    let mut scene = Scene::new();
    let mut selection = Selection::new();
    let mut plugin = RapierPlugin::new();

    // Scene has node at (1, 2, 3); rapier body is built at the origin.
    // Prepare should overwrite the body to match the scene.
    let id = add_node_at(&mut scene, glam::Vec3::new(1.0, 2.0, 3.0));
    plugin.add_body(
        id,
        RigidBodyBuilder::fixed().build(),
        ColliderBuilder::ball(0.5).build(),
    );

    let mut runtime = make_runtime(&plugin, 60.0);
    runtime.step(&mut scene, &mut selection, &frame_ctx());

    let translation = plugin
        .with_body_mut(id, |b| b.position().translation)
        .unwrap();
    assert!(
        (translation - glam::Vec3::new(1.0, 2.0, 3.0)).length() < 1e-4,
        "fixed body should be teleported to scene transform, got {translation:?}"
    );
}

#[test]
fn kinematic_body_follows_scene_transform_changes() {
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

    // Now move the scene node and step again. The prepare phase reads
    // world_transform, which is the cached value; update_transforms makes
    // it current.
    scene.set_local_transform(
        id,
        glam::Mat4::from_translation(glam::Vec3::new(5.0, 0.0, 0.0)),
    );
    scene.update_transforms();
    runtime.step(&mut scene, &mut selection, &frame_ctx());

    let translation = plugin
        .with_body_mut(id, |b| b.position().translation)
        .unwrap();
    assert!(
        (translation - glam::Vec3::new(5.0, 0.0, 0.0)).length() < 0.1,
        "kinematic body should follow scene to ~(5,0,0), got {translation:?}"
    );
}

#[test]
fn dynamic_body_is_not_overwritten_by_scene_position() {
    // Build a dynamic body at (10, 0, 0); meanwhile pretend the user (or some
    // other system) wrote (99, 99, 99) into the scene node. Without gravity
    // the body should stay near its rapier position, not jump to the scene
    // position -- prepare must skip dynamic bodies.
    let mut scene = Scene::new();
    let mut selection = Selection::new();
    let mut plugin = RapierPlugin::new().with_gravity(glam::Vec3::ZERO);

    let id = add_node_at(&mut scene, glam::Vec3::new(99.0, 99.0, 99.0));
    plugin.add_body(
        id,
        RigidBodyBuilder::dynamic()
            .translation(glam::Vec3::new(10.0, 0.0, 0.0))
            .build(),
        ColliderBuilder::ball(0.5).build(),
    );

    let mut runtime = make_runtime(&plugin, 60.0);
    runtime.step(&mut scene, &mut selection, &frame_ctx());

    let translation = plugin
        .with_body_mut(id, |b| b.position().translation)
        .unwrap();
    assert!(
        (translation - glam::Vec3::new(10.0, 0.0, 0.0)).length() < 1e-3,
        "dynamic body should keep its rapier-side position, got {translation:?}"
    );
}

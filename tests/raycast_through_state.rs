//! Raycast through the escape hatch: construct a `QueryPipeline` inside
//! `with_state` and translate the hit collider handle back to a `NodeId`.
//! This mirrors what `examples/ray_probe.rs` does in the real app.

mod common;

use common::{add_node_at, make_runtime};
use rapier_viewport_plugin::{
    parry3d::{query::Ray, shape::SharedShape},
    rapier3d::{
        pipeline::QueryFilter,
        prelude::{ColliderBuilder, RigidBodyBuilder},
    },
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
fn ray_hits_registered_body_and_resolves_to_node_id() {
    let mut scene = Scene::new();
    let mut selection = Selection::new();
    let mut plugin = RapierPlugin::new().with_gravity(glam::Vec3::ZERO);

    let floor = add_node_at(&mut scene, glam::Vec3::ZERO);
    plugin.add_body(
        floor,
        RigidBodyBuilder::fixed().build(),
        ColliderBuilder::new(SharedShape::halfspace(glam::Vec3::Z)).build(),
    );

    let target_pos = glam::Vec3::new(0.0, 0.0, 5.0);
    let target = add_node_at(&mut scene, target_pos);
    plugin.add_body(
        target,
        RigidBodyBuilder::dynamic().translation(target_pos).build(),
        ColliderBuilder::ball(0.5).build(),
    );

    // The broad phase is only populated during simulate; step once before
    // the query so the BVH contains both colliders.
    let mut runtime = make_runtime(&plugin, 60.0);
    runtime.step(&mut scene, &mut selection, &frame_ctx());

    let origin = glam::Vec3::new(0.0, 0.0, 10.0);
    let dir = glam::Vec3::NEG_Z;
    let floor_collider = plugin.collider_handle(floor);

    let hit_node = plugin.with_state(|s| {
        let qp = s.broad_phase.as_query_pipeline(
            s.narrow_phase.query_dispatcher(),
            &s.bodies,
            &s.colliders,
            QueryFilter {
                exclude_collider: floor_collider,
                ..Default::default()
            },
        );
        let ray = Ray::new(origin, dir);
        qp.cast_ray_and_get_normal(&ray, 100.0, true)
            .and_then(|(handle, _)| s.node_for_collider(handle))
    });

    assert_eq!(hit_node, Some(target), "ray should hit the dynamic ball");
}

#[test]
fn ray_misses_when_nothing_in_path() {
    let mut scene = Scene::new();
    let mut selection = Selection::new();
    let mut plugin = RapierPlugin::new().with_gravity(glam::Vec3::ZERO);

    let id = add_node_at(&mut scene, glam::Vec3::ZERO);
    plugin.add_body(
        id,
        RigidBodyBuilder::dynamic().build(),
        ColliderBuilder::ball(0.5).build(),
    );

    let mut runtime = make_runtime(&plugin, 60.0);
    runtime.step(&mut scene, &mut selection, &frame_ctx());

    // Ray pointing away from the only body in the world.
    let origin = glam::Vec3::new(100.0, 100.0, 100.0);
    let dir = glam::Vec3::X;

    let hit = plugin.with_state(|s| {
        let qp = s.broad_phase.as_query_pipeline(
            s.narrow_phase.query_dispatcher(),
            &s.bodies,
            &s.colliders,
            QueryFilter::default(),
        );
        let ray = Ray::new(origin, dir);
        qp.cast_ray_and_get_normal(&ray, 100.0, true)
    });

    assert!(hit.is_none(), "ray should not hit anything");
}

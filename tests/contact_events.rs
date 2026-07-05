//! `RapierSimulatePlugin` translates rapier's raw `CollisionEvent`s into
//! NodeId-keyed `RapierContactEvent`s on the runtime output bus.

mod common;

use common::{add_node_at, make_runtime};
use rapier_viewport_plugin::{
    parry3d::shape::SharedShape,
    rapier3d::prelude::{ActiveEvents, ColliderBuilder, RigidBodyBuilder},
    RapierContactEvent, RapierPlugin,
};
use viewport_lib::{RuntimeFrameContext, Scene, Selection};

const DT: f32 = 1.0 / 60.0;

fn frame_ctx() -> RuntimeFrameContext {
    let mut ctx = RuntimeFrameContext::default();
    ctx.dt = DT;
    ctx
}

#[test]
fn dynamic_ball_dropped_on_halfspace_emits_started_event() {
    let mut scene = Scene::new();
    let mut selection = Selection::new();
    let mut plugin = RapierPlugin::new();

    let floor = add_node_at(&mut scene, glam::Vec3::ZERO);
    plugin.add_body(
        floor,
        RigidBodyBuilder::fixed().build(),
        ColliderBuilder::new(SharedShape::halfspace(glam::Vec3::Z))
            .active_events(ActiveEvents::COLLISION_EVENTS)
            .build(),
    );

    let ball = add_node_at(&mut scene, glam::Vec3::new(0.0, 0.0, 2.0));
    plugin.add_body(
        ball,
        RigidBodyBuilder::dynamic()
            .translation(glam::Vec3::new(0.0, 0.0, 2.0))
            .build(),
        ColliderBuilder::ball(0.5)
            .active_events(ActiveEvents::COLLISION_EVENTS)
            .build(),
    );

    let mut runtime = make_runtime(&plugin, 60.0);
    let mut got_started = false;
    // 2 simulated seconds is plenty for a fall of ~1.5 m.
    for _ in 0..120 {
        let mut output = runtime.step(&mut scene, &mut selection, &frame_ctx());
        for ev in output.events.drain::<RapierContactEvent>() {
            let pair_matches = (ev.node_a == floor && ev.node_b == ball)
                || (ev.node_a == ball && ev.node_b == floor);
            if pair_matches && ev.started {
                got_started = true;
            }
        }
        if got_started {
            break;
        }
    }
    assert!(
        got_started,
        "expected a Started RapierContactEvent between the ball and the floor"
    );
}

#[test]
fn started_then_stopped_event_pair_for_bouncing_ball() {
    // A high-restitution ball lands on a halfspace, then leaves it again.
    // We should see both a Started and a Stopped event (in that order) for
    // the same pair.
    let mut scene = Scene::new();
    let mut selection = Selection::new();
    let mut plugin = RapierPlugin::new();

    let floor = add_node_at(&mut scene, glam::Vec3::ZERO);
    plugin.add_body(
        floor,
        RigidBodyBuilder::fixed().build(),
        ColliderBuilder::new(SharedShape::halfspace(glam::Vec3::Z))
            .restitution(1.0)
            .active_events(ActiveEvents::COLLISION_EVENTS)
            .build(),
    );

    let ball = add_node_at(&mut scene, glam::Vec3::new(0.0, 0.0, 3.0));
    plugin.add_body(
        ball,
        RigidBodyBuilder::dynamic()
            .translation(glam::Vec3::new(0.0, 0.0, 3.0))
            .build(),
        ColliderBuilder::ball(0.5)
            .restitution(1.0)
            .friction(0.0)
            .active_events(ActiveEvents::COLLISION_EVENTS)
            .build(),
    );

    let mut runtime = make_runtime(&plugin, 240.0);
    let mut started_seen = false;
    let mut stopped_seen = false;

    for _ in 0..480 {
        let mut output = runtime.step(&mut scene, &mut selection, &frame_ctx());
        for ev in output.events.drain::<RapierContactEvent>() {
            let pair_matches = (ev.node_a == floor && ev.node_b == ball)
                || (ev.node_a == ball && ev.node_b == floor);
            if !pair_matches {
                continue;
            }
            if ev.started {
                started_seen = true;
            } else if started_seen {
                stopped_seen = true;
            }
        }
        if started_seen && stopped_seen {
            break;
        }
    }
    assert!(started_seen, "missed Started event");
    assert!(stopped_seen, "missed Stopped event after Started");
}

#[test]
fn no_events_emitted_when_active_events_unset() {
    // Bodies whose colliders do not opt in to collision events should not
    // produce any RapierContactEvent at all.
    let mut scene = Scene::new();
    let mut selection = Selection::new();
    let mut plugin = RapierPlugin::new();

    let floor = add_node_at(&mut scene, glam::Vec3::ZERO);
    plugin.add_body(
        floor,
        RigidBodyBuilder::fixed().build(),
        ColliderBuilder::new(SharedShape::halfspace(glam::Vec3::Z)).build(),
    );

    let ball = add_node_at(&mut scene, glam::Vec3::new(0.0, 0.0, 2.0));
    plugin.add_body(
        ball,
        RigidBodyBuilder::dynamic()
            .translation(glam::Vec3::new(0.0, 0.0, 2.0))
            .build(),
        ColliderBuilder::ball(0.5).build(),
    );

    let mut runtime = make_runtime(&plugin, 60.0);
    let mut event_count = 0;
    for _ in 0..120 {
        let mut output = runtime.step(&mut scene, &mut selection, &frame_ctx());
        event_count += output.events.drain::<RapierContactEvent>().len();
    }
    assert_eq!(event_count, 0, "events should be silent without ActiveEvents");
}

//! Helpers shared by the integration test files.
//!
//! Each `tests/*.rs` is its own binary; including this via `mod common;`
//! pulls the helpers in without making them part of the public surface.

#![allow(dead_code)] // not every test uses every helper

use rapier_viewport_plugin::RapierPlugin;
use viewport_lib::{
    FixedTimestep, MaterialId, NodeId, Scene, ViewportRuntime,
};

/// Add a node with no mesh at the given world-space translation. Used to
/// stand in for the visible scene objects examples register.
pub fn add_node_at(scene: &mut Scene, pos: glam::Vec3) -> NodeId {
    scene.add(
        None,
        glam::Mat4::from_translation(pos),
        MaterialId::DEFAULT,
    )
}

/// Construct a runtime wired up with the plugin's prepare + simulate phases
/// and a fixed timestep that matches the test's wall dt (so one substep per
/// `step` call).
pub fn make_runtime(plugin: &RapierPlugin, sim_hz: f32) -> ViewportRuntime {
    let (prepare, simulate) = plugin.clone().into_plugins();
    ViewportRuntime::new()
        .with_fixed_timestep(FixedTimestep::new(sim_hz))
        .with_plugin(prepare)
        .with_plugin(simulate)
}

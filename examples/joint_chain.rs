//! A 5-link chain hanging from a fixed anchor, connected by revolute
//! (hinge) joints rotating around the Y axis. Press space to kick the
//! bottom link sideways.
//!
//! Demonstrates the **escape hatches** end-to-end on a realistic feature
//! (joints) that the plugin does not curate:
//!
//! - `body_handle(node_id)` translates the scene's `NodeId`s to rapier
//!   `RigidBodyHandle`s. Joints require handles, not NodeIds.
//! - `with_state_mut(|s| { ... s.impulse_joints.insert(...) ... })`
//!   inserts the joints into `RapierState::impulse_joints` directly --
//!   no `RapierJointDesc`, no `add_joint`, just rapier's own builders.
//! - `with_body_mut(id, |body| body.apply_impulse(...))` kicks the
//!   bottom link without us writing an `apply_impulse` wrapper.
//!
//! Controls: space to kick the chain, R to reset. Left-drag to orbit,
//! right-drag to pan, scroll to zoom.
//!
//! Run with: `cargo run --example joint_chain`

use eframe::egui;
use rapier_viewport_plugin::{
    rapier3d::prelude::{ColliderBuilder, RevoluteJointBuilder, RigidBodyBuilder},
    RapierPlugin,
};
use viewport_lib::{
    ButtonState, Camera, CameraFrame, FixedTimestep, FrameData, LightingSettings, Colour, Material,
    MaterialId, MeshId, Modifiers, MouseButton, NodeId, OrbitCameraController, RuntimeFrameContext,
    Scene, SceneFrame, SceneRenderItem, ScrollUnits, Selection, ViewportContext, ViewportEvent,
    ViewportRenderer, ViewportRuntime, primitives,
};

fn main() -> eframe::Result {
    eframe::run_native(
        "rapier: Joint Chain",
        eframe::NativeOptions {
            viewport: egui::ViewportBuilder::default().with_inner_size([1100.0, 680.0]),
            depth_buffer: 24,
            stencil_buffer: 8,
            ..Default::default()
        },
        Box::new(|cc| {
            let rs = cc.wgpu_render_state.as_ref().expect("wgpu backend required");
            let device = &rs.device;
            let format = rs.target_format;

            let mut renderer = ViewportRenderer::new(device, format);
            let res = renderer.resources_mut();

            let link_mesh = res
                .upload_mesh_data(
                    device,
                    &primitives::cuboid(
                        2.0 * LINK_HALF.x,
                        2.0 * LINK_HALF.y,
                        2.0 * LINK_HALF.z,
                    ),
                )
                .expect("link mesh upload");

            let link_materials = LINK_COLOURS
                .iter()
                .map(|&c| res.upload_material(Material::from_colour(Colour::from_linear_rgb_array(c))).expect("link material"))
                .collect();

            rs.renderer.write().callback_resources.insert(renderer);
            Ok(Box::new(App::new(link_mesh, link_materials)))
        }),
    )
}

// ---------------------------------------------------------------------------

const SIM_HZ: f32 = 240.0;

const LINK_COLOURS: [[f32; 3]; 7] = [
    [0.9, 0.3, 0.3],
    [0.9, 0.6, 0.2],
    [0.8, 0.8, 0.2],
    [0.3, 0.8, 0.3],
    [0.2, 0.7, 0.8],
    [0.3, 0.4, 0.9],
    [0.7, 0.3, 0.9],
];
const CHAIN_LEN: usize = 7;
const LINK_HALF: glam::Vec3 = glam::Vec3::new(0.15, 0.15, 0.35);
const LINK_DENSITY: f32 = 8.0;
const LINK_LINEAR_DAMPING: f32 = 0.4;
const LINK_ANGULAR_DAMPING: f32 = 1.2;
// Tuned for the link mass (volume * LINK_DENSITY): a gentle nudge that
// makes the bottom swing out and propagate up the chain rather than
// spinning the whole thing around the top hinge.
const KICK_IMPULSE: f32 = 1.5;

/// World-space center Z of chain link `i` (link 0 is topmost). Links touch
/// end-to-end along the Z axis; the joint between links N and N+1 lives
/// where the bottom of N meets the top of N+1.
fn link_center_z(i: usize) -> f32 {
    -LINK_HALF.z - 2.0 * LINK_HALF.z * i as f32
}

// ---------------------------------------------------------------------------

struct App {
    camera: Camera,
    controller: OrbitCameraController,
    link_mesh: MeshId,
    link_materials: Vec<MaterialId>,

    scene: Scene,
    selection: Selection,
    runtime: ViewportRuntime,
    rapier: RapierPlugin,

    /// Invisible fixed body the chain hangs from.
    anchor_id: NodeId,
    /// Top-to-bottom list of chain link nodes.
    chain_ids: Vec<NodeId>,

    cursor_vp: Option<glam::Vec2>,
    left_held: bool,
    drag_started: bool,
    press_origin: Option<glam::Vec2>,
}

impl App {
    fn new(link_mesh: MeshId, link_materials: Vec<MaterialId>) -> Self {
        let mut app = Self {
            camera: Camera::orbit(
                glam::DVec3::new(0.0, 0.0, -1.8),
                8.0,
                Camera::default().orientation(),
            ),
            controller: OrbitCameraController::viewport_primitives(),
            link_mesh,
            link_materials,
            scene: Scene::new(),
            selection: Selection::new(),
            runtime: ViewportRuntime::new(),
            rapier: RapierPlugin::new(),
            anchor_id: NodeId::default(),
            chain_ids: Vec::new(),
            cursor_vp: None,
            left_held: false,
            drag_started: false,
            press_origin: None,
        };
        app.reset();
        app
    }

    fn reset(&mut self) {
        self.scene = Scene::new();
        self.selection = Selection::new();
        self.rapier = RapierPlugin::new();

        // ---- Anchor: a fixed body at the origin with no visible mesh. ------
        // Its collider is a tiny sensor so it can't physically interact with
        // the chain -- it exists only as the joint's parent body.
        self.anchor_id = self
            .scene
            .add(None, glam::Mat4::IDENTITY, MaterialId::DEFAULT);
        self.rapier.add_body(
            self.anchor_id,
            RigidBodyBuilder::fixed().build(),
            ColliderBuilder::ball(0.01).sensor(true).build(),
        );

        // ---- Chain links: dynamic boxes, top to bottom. -------------------
        // Damping lets the chain settle naturally instead of swinging
        // forever, and gives each link's relative oscillation time to
        // develop into visible flex when the bottom is kicked.
        self.chain_ids.clear();
        for i in 0..CHAIN_LEN {
            let pos = glam::Vec3::new(0.0, 0.0, link_center_z(i));
            let id = self.scene.add(
                Some(self.link_mesh),
                glam::Mat4::from_translation(pos),
                self.link_materials[i % self.link_materials.len()],
            );
            let body = RigidBodyBuilder::dynamic()
                .translation(pos)
                .linear_damping(LINK_LINEAR_DAMPING)
                .angular_damping(LINK_ANGULAR_DAMPING)
                .build();
            let collider = ColliderBuilder::cuboid(LINK_HALF.x, LINK_HALF.y, LINK_HALF.z)
                .density(LINK_DENSITY)
                .build();
            self.rapier.add_body(id, body, collider);
            self.chain_ids.push(id);
        }

        // ---- Joints: this is the escape-hatch path. -----------------------
        // Joints aren't part of the curated API. We translate the scene
        // NodeIds to rapier RigidBodyHandles via `body_handle` *before*
        // entering `with_state_mut` -- calling a plugin method from inside
        // the closure would deadlock the state mutex.
        let anchor_handle = self.rapier.body_handle(self.anchor_id).expect("anchor");
        let link_handles: Vec<_> = self
            .chain_ids
            .iter()
            .map(|&id| self.rapier.body_handle(id).expect("link"))
            .collect();

        self.rapier.with_state_mut(|s| {
            // Anchor -> top link, hinged at the world origin.
            let top_joint = RevoluteJointBuilder::new(glam::Vec3::Y)
                .local_anchor1(glam::Vec3::ZERO)
                .local_anchor2(glam::Vec3::new(0.0, 0.0, LINK_HALF.z))
                .build();
            s.impulse_joints
                .insert(anchor_handle, link_handles[0], top_joint, true);

            // Link N -> link N+1: bottom of N meets top of N+1.
            for i in 0..CHAIN_LEN - 1 {
                let joint = RevoluteJointBuilder::new(glam::Vec3::Y)
                    .local_anchor1(glam::Vec3::new(0.0, 0.0, -LINK_HALF.z))
                    .local_anchor2(glam::Vec3::new(0.0, 0.0, LINK_HALF.z))
                    .build();
                s.impulse_joints
                    .insert(link_handles[i], link_handles[i + 1], joint, true);
            }
        });

        // ---- Wire the plugin into the runtime. ----------------------------
        let (prepare, simulate) = self.rapier.clone().into_plugins();
        self.runtime = ViewportRuntime::new()
            .with_fixed_timestep(FixedTimestep::new(SIM_HZ))
            .with_plugin(prepare)
            .with_plugin(simulate);
    }

    fn kick(&mut self) {
        // Apply an X-axis impulse to the bottom link. The plugin doesn't
        // expose `apply_impulse`; we reach the body through `with_body_mut`
        // and call rapier directly.
        let Some(&bottom) = self.chain_ids.last() else {
            return;
        };
        self.rapier.with_body_mut(bottom, |body| {
            body.apply_impulse(glam::Vec3::new(KICK_IMPULSE, 0.0, 0.0), true);
        });
    }
}

// ---------------------------------------------------------------------------

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Capture global key intent up front. Both Space and R fire on the
        // press edge (not while held), so we read them with `key_pressed`.
        let (space_pressed, r_pressed) = ctx.input(|i| {
            (
                i.key_pressed(egui::Key::Space),
                i.key_pressed(egui::Key::R),
            )
        });

        egui::SidePanel::left("controls")
            .exact_width(220.0)
            .show(ctx, |ui| {
                ui.add_space(8.0);
                ui.heading("Joint Chain");
                ui.add_space(4.0);
                ui.separator();
                ui.add_space(6.0);

                ui.label("Five boxes hinged together by rapier");
                ui.label("revolute joints (Y-axis).");
                ui.add_space(6.0);
                ui.label("Space: kick the bottom link.");
                ui.label("R: reset.");

                ui.add_space(12.0);
                if ui.button("Reset").clicked() || r_pressed {
                    self.reset();
                }
                if ui.button("Kick").clicked() {
                    self.kick();
                }
            });

        if space_pressed {
            self.kick();
        }

        egui::CentralPanel::default().show(ctx, |ui| {
            let dt = ctx.input(|i| i.stable_dt).min(0.25);

            let (rect, response) =
                ui.allocate_exact_size(ui.available_size(), egui::Sense::click_and_drag());

            self.drag_started = false;
            self.controller.begin_frame(ViewportContext {
                hovered: response.hovered(),
                focused: response.has_focus(),
                viewport_size: [rect.width(), rect.height()],
            });

            ui.input(|i| {
                self.controller.push_event(ViewportEvent::ModifiersChanged(
                    Modifiers::NONE
                        .with_alt(i.modifiers.alt)
                        .with_shift(i.modifiers.shift)
                        .with_ctrl(i.modifiers.command),
                ));
                let local = i
                    .pointer
                    .interact_pos()
                    .map(|p| glam::Vec2::new(p.x - rect.left(), p.y - rect.top()));
                self.cursor_vp = local;
                if let Some(pos) = local {
                    self.controller
                        .push_event(ViewportEvent::PointerMoved { position: pos });
                }
                for event in &i.events {
                    match event {
                        egui::Event::PointerButton { button, pressed, .. } => {
                            let vp_btn = match button {
                                egui::PointerButton::Primary => MouseButton::Left,
                                egui::PointerButton::Secondary => MouseButton::Right,
                                egui::PointerButton::Middle => MouseButton::Middle,
                                _ => continue,
                            };
                            self.controller.push_event(ViewportEvent::MouseButton {
                                button: vp_btn,
                                state: if *pressed {
                                    ButtonState::Pressed
                                } else {
                                    ButtonState::Released
                                },
                            });
                            if *button == egui::PointerButton::Primary {
                                if *pressed {
                                    self.left_held = true;
                                    self.press_origin = self.cursor_vp;
                                    self.drag_started = true;
                                } else {
                                    self.left_held = false;
                                    self.press_origin = None;
                                }
                            }
                        }
                        egui::Event::MouseWheel { unit, delta, .. } => {
                            let units = match unit {
                                egui::MouseWheelUnit::Line => ScrollUnits::Lines,
                                egui::MouseWheelUnit::Point => ScrollUnits::Pixels,
                                egui::MouseWheelUnit::Page => ScrollUnits::Pages,
                            };
                            self.controller.push_event(ViewportEvent::Wheel {
                                delta: glam::Vec2::new(delta.x, delta.y),
                                units,
                            });
                        }
                        _ => {}
                    }
                }
            });

            self.controller.apply_to_camera(&mut self.camera);
            self.camera.set_aspect_ratio(rect.width(), rect.height());

            // Step the runtime. The simulate plugin advances rapier and
            // writes every dynamic link's pose back to the scene.
            let mut frame_ctx = RuntimeFrameContext::default();
            frame_ctx.dt = dt;
            frame_ctx.camera = self.camera.clone();
            frame_ctx.viewport_size = glam::Vec2::new(rect.width(), rect.height());
            self.runtime
                .step(&mut self.scene, &mut self.selection, &frame_ctx);

            let alpha = self.runtime.alpha();
            let snapshots = self.runtime.snapshots();

            let items: Vec<SceneRenderItem> = self
                .chain_ids
                .iter()
                .filter_map(|&id| {
                    let node = self.scene.node(id)?;
                    let world = node.world_transform();
                    let model = snapshots
                        .interpolated(id, alpha)
                        .map(viewport_lib::Transform::from)
                        .unwrap_or(world);
                    let mut item = SceneRenderItem::new(self.link_mesh, node.material());
                    item.model = model;
                    Some(item)
                })
                .collect();

            let mut frame_data = FrameData::new(
                CameraFrame::from_camera(&self.camera, [rect.width(), rect.height()])
                    .with_pixels_per_point(ui.ctx().pixels_per_point()),
                SceneFrame::from_surface_items(items).with_generation(self.scene.version()),
            );
            frame_data.effects.lighting = LightingSettings::default();

            ui.painter().add(eframe::egui_wgpu::Callback::new_paint_callback(
                rect,
                ViewportCallback { frame: frame_data },
            ));
        });

        ctx.request_repaint();
    }
}

// ---------------------------------------------------------------------------

struct ViewportCallback {
    frame: FrameData,
}

impl eframe::egui_wgpu::CallbackTrait for ViewportCallback {
    fn prepare(
        &self,
        device: &eframe::wgpu::Device,
        queue: &eframe::wgpu::Queue,
        _screen_descriptor: &eframe::egui_wgpu::ScreenDescriptor,
        _egui_encoder: &mut eframe::wgpu::CommandEncoder,
        callback_resources: &mut eframe::egui_wgpu::CallbackResources,
    ) -> Vec<eframe::wgpu::CommandBuffer> {
        if let Some(renderer) = callback_resources.get_mut::<ViewportRenderer>() {
            return renderer.pass().prepare(device, queue, &self.frame);
        }
        Vec::new()
    }

    fn paint(
        &self,
        _info: eframe::egui::PaintCallbackInfo,
        render_pass: &mut eframe::wgpu::RenderPass<'static>,
        callback_resources: &eframe::egui_wgpu::CallbackResources,
    ) {
        if let Some(renderer) = callback_resources.get::<ViewportRenderer>() {
            renderer.pass_view().paint(render_pass, &self.frame);
        }
    }
}

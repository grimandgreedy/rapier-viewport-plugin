//! A keyboard-driven character (capsule) moving around a floor strewn with
//! box obstacles. The movement runs through rapier's
//! `KinematicCharacterController`, which sweeps the shape against the
//! world and produces an "effective" translation that slides along walls,
//! stops at obstacles, and snaps to the ground.
//!
//! Demonstrates:
//!
//! - `KinematicCharacterController::move_shape(...)`, the canonical rapier
//!   way to move a player-style character.
//! - Driving the controller through `with_state` so it borrows the
//!   `BroadPhaseBvh`, `NarrowPhase`, and collider/body sets transiently
//!   without us writing a wrapper.
//! - Wiring the controller's result back through the scene: we advance
//!   the character's `KinematicPosition` body by writing the scene
//!   transform, then let `RapierPreparePlugin` mirror it into rapier.
//!
//! Controls: arrow keys / WASD to move (X/Y plane). Left-drag to orbit,
//! scroll to zoom.
//!
//! Run with: `cargo run --example character_controller`

use eframe::egui;
use rapier_viewport_plugin::{
    parry3d::shape::SharedShape,
    rapier3d::{
        control::KinematicCharacterController,
        pipeline::QueryFilter,
        prelude::{ColliderBuilder, RigidBodyBuilder},
    },
    RapierPlugin,
};
use viewport_lib::{
    BackfacePolicy, ButtonState, Camera, CameraFrame, FixedTimestep, FrameData, LightingSettings,
    Material, MeshId, Modifiers, MouseButton, NodeId, OrbitCameraController, RuntimeFrameContext,
    Scene, SceneFrame, SceneRenderItem, ScrollUnits, Selection, ViewportContext, ViewportEvent,
    ViewportRenderer, ViewportRuntime, primitives,
};

fn main() -> eframe::Result {
    eframe::run_native(
        "rapier: Character Controller",
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

            // Approximate the capsule visually with a sphere stretched along
            // Z. (The plugin's collider is a real capsule_z; this is just
            // the visual.)
            let char_mesh = res
                .upload_mesh_data(device, &primitives::sphere(CHAR_RADIUS, 24, 12))
                .expect("char mesh upload");
            let box_mesh = res
                .upload_mesh_data(
                    device,
                    &primitives::cuboid(2.0 * BOX_HALF, 2.0 * BOX_HALF, 2.0 * BOX_HALF),
                )
                .expect("box mesh upload");
            let floor_mesh = res
                .upload_mesh_data(device, &primitives::plane(20.0, 20.0))
                .expect("floor mesh upload");

            rs.renderer.write().callback_resources.insert(renderer);
            Ok(Box::new(App::new(char_mesh, box_mesh, floor_mesh)))
        }),
    )
}

// ---------------------------------------------------------------------------

const SIM_HZ: f32 = 120.0;
const CHAR_RADIUS: f32 = 0.35;
const CHAR_HALF_HEIGHT: f32 = 0.4;
const CHAR_SPEED: f32 = 4.0;
const BOX_HALF: f32 = 0.6;
const GRAVITY: f32 = 9.81;

const OBSTACLES: &[(f32, f32)] = &[
    (2.0, 0.0),
    (-2.0, 1.0),
    (0.0, 2.5),
    (1.5, -2.5),
    (-2.5, -2.0),
];

// ---------------------------------------------------------------------------

struct App {
    camera: Camera,
    controller_view: OrbitCameraController,
    char_mesh: MeshId,
    box_mesh: MeshId,
    floor_mesh: MeshId,

    scene: Scene,
    selection: Selection,
    runtime: ViewportRuntime,
    rapier: RapierPlugin,

    floor_id: NodeId,
    obstacle_ids: Vec<NodeId>,
    char_id: NodeId,
    char_pos: glam::Vec3,
    char_grounded: bool,

    /// The rapier kinematic character controller. Cheap to construct and
    /// configure; we keep it around so its config persists across frames.
    char_controller: KinematicCharacterController,

    cursor_vp: Option<glam::Vec2>,
    left_held: bool,
    drag_started: bool,
    press_origin: Option<glam::Vec2>,
}

impl App {
    fn new(char_mesh: MeshId, box_mesh: MeshId, floor_mesh: MeshId) -> Self {
        // Our world is Z-up, but the default controller's `up` is Y.
        // Override it so floor / slope detection works correctly.
        let mut char_controller = KinematicCharacterController::default();
        char_controller.up = glam::Vec3::Z;

        let mut app = Self {
            camera: Camera {
                center: glam::Vec3::new(0.0, 0.0, 0.5),
                distance: 12.0,
                ..Camera::default()
            },
            controller_view: OrbitCameraController::viewport_primitives(),
            char_mesh,
            box_mesh,
            floor_mesh,
            scene: Scene::new(),
            selection: Selection::new(),
            runtime: ViewportRuntime::new(),
            rapier: RapierPlugin::new(),
            floor_id: NodeId::default(),
            obstacle_ids: Vec::new(),
            char_id: NodeId::default(),
            char_pos: glam::Vec3::ZERO,
            char_grounded: false,
            char_controller,
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
        self.char_grounded = false;

        // ---- Floor. -------------------------------------------------------
        self.floor_id = self.scene.add(
            Some(self.floor_mesh),
            glam::Mat4::IDENTITY,
            {
                let mut m = Material::from_colour([0.35, 0.35, 0.35]);
                m.backface_policy = BackfacePolicy::Identical;
                m
            },
        );
        self.rapier.add_body(
            self.floor_id,
            RigidBodyBuilder::fixed().build(),
            ColliderBuilder::new(SharedShape::halfspace(glam::Vec3::Z)).build(),
        );

        // ---- Fixed box obstacles. -----------------------------------------
        self.obstacle_ids.clear();
        for &(x, y) in OBSTACLES {
            let pos = glam::Vec3::new(x, y, BOX_HALF);
            let id = self.scene.add(
                Some(self.box_mesh),
                glam::Mat4::from_translation(pos),
                Material::from_colour([0.5, 0.55, 0.6]),
            );
            self.rapier.add_body(
                id,
                RigidBodyBuilder::fixed().translation(pos).build(),
                ColliderBuilder::cuboid(BOX_HALF, BOX_HALF, BOX_HALF).build(),
            );
            self.obstacle_ids.push(id);
        }

        // ---- Character: a kinematic-position capsule. ---------------------
        // The character controller will move it; we relay the new transform
        // through the scene each frame so `RapierPreparePlugin` keeps the
        // rapier body in sync.
        self.char_pos = glam::Vec3::new(0.0, -3.0, CHAR_HALF_HEIGHT + CHAR_RADIUS);
        self.char_id = self.scene.add(
            Some(self.char_mesh),
            glam::Mat4::from_translation(self.char_pos),
            Material::from_colour([0.9, 0.7, 0.2]),
        );
        self.rapier.add_body(
            self.char_id,
            RigidBodyBuilder::kinematic_position_based()
                .translation(self.char_pos)
                .build(),
            ColliderBuilder::capsule_z(CHAR_HALF_HEIGHT, CHAR_RADIUS).build(),
        );

        let (prepare, simulate) = self.rapier.clone().into_plugins();
        self.runtime = ViewportRuntime::new()
            .with_fixed_timestep(FixedTimestep::new(SIM_HZ))
            .with_plugin(prepare)
            .with_plugin(simulate);
    }

    /// Advance the character one frame using `KinematicCharacterController`.
    fn move_character(&mut self, dt: f32, move_dir: glam::Vec2) {
        // Build the desired translation: horizontal from input, plus a
        // constant downward "gravity" so the character settles onto floors
        // and ramps. Kinematic bodies don't get gravity from rapier.
        let mut desired = glam::Vec3::new(
            move_dir.x * CHAR_SPEED * dt,
            move_dir.y * CHAR_SPEED * dt,
            -GRAVITY * dt,
        );

        // If the controller said we were grounded last frame, we can clamp
        // the downward intent so the snap-to-ground feature handles it.
        if self.char_grounded {
            desired.z = desired.z.min(0.0);
        }

        // Exclude the character's own collider so the sweep doesn't hit it.
        let char_collider = self.rapier.collider_handle(self.char_id);

        // The escape hatch: borrow the live state, construct a
        // QueryPipeline, ask the controller for the safe movement, and
        // return the resulting EffectiveCharacterMovement.
        let effective = self.rapier.with_state(|s| {
            let qp = s.broad_phase.as_query_pipeline(
                s.narrow_phase.query_dispatcher(),
                &s.bodies,
                &s.colliders,
                QueryFilter {
                    exclude_collider: char_collider,
                    ..Default::default()
                },
            );
            let collider = s.colliders.get(char_collider.unwrap()).unwrap();
            let pose = *collider.position();
            self.char_controller
                .move_shape(dt, &qp, collider.shape(), &pose, desired, |_event| {})
        });

        // Apply the result to the character's scene transform. The Prepare
        // plugin reads this on the next step and updates rapier.
        self.char_pos += effective.translation;
        self.char_grounded = effective.grounded;
        self.scene.set_local_transform(
            self.char_id,
            glam::Mat4::from_translation(self.char_pos),
        );

        // Force the dirty `world_transform` to be recomputed now, before
        // `runtime.step` reads it in the Prepare phase. The runtime only
        // calls `update_transforms` after writebacks, and this example has
        // no dynamic bodies generating writebacks -- without this call the
        // Prepare phase keeps reading the stale (initial) world_transform
        // and the rapier kinematic body never moves.
        self.scene.update_transforms();
    }
}

// ---------------------------------------------------------------------------

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        let (move_dir, dt_wall) = ctx.input(|i| {
            let mut dir = glam::Vec2::ZERO;
            if i.key_down(egui::Key::ArrowUp) || i.key_down(egui::Key::W) {
                dir.y += 1.0;
            }
            if i.key_down(egui::Key::ArrowDown) || i.key_down(egui::Key::S) {
                dir.y -= 1.0;
            }
            if i.key_down(egui::Key::ArrowLeft) || i.key_down(egui::Key::A) {
                dir.x -= 1.0;
            }
            if i.key_down(egui::Key::ArrowRight) || i.key_down(egui::Key::D) {
                dir.x += 1.0;
            }
            if dir.length_squared() > 0.0 {
                dir = dir.normalize();
            }
            (dir, i.stable_dt.min(0.05))
        });

        egui::SidePanel::left("controls")
            .exact_width(220.0)
            .show(ctx, |ui| {
                ui.add_space(8.0);
                ui.heading("Character Controller");
                ui.add_space(4.0);
                ui.separator();
                ui.add_space(6.0);

                ui.label("Arrow keys / WASD to move.");
                ui.label("Capsule slides off the boxes and");
                ui.label("snaps to the floor via rapier's");
                ui.label("KinematicCharacterController.");

                ui.add_space(10.0);
                ui.label(format!(
                    "Pos: ({:+.2}, {:+.2}, {:+.2})",
                    self.char_pos.x, self.char_pos.y, self.char_pos.z
                ));
                ui.label(format!("Grounded: {}", self.char_grounded));

                ui.add_space(12.0);
                if ui.button("Reset").clicked() {
                    self.reset();
                }
            });

        egui::CentralPanel::default().show(ctx, |ui| {
            let (rect, response) =
                ui.allocate_exact_size(ui.available_size(), egui::Sense::click_and_drag());

            self.drag_started = false;
            self.controller_view.begin_frame(ViewportContext {
                hovered: response.hovered(),
                focused: response.has_focus(),
                viewport_size: [rect.width(), rect.height()],
            });

            ui.input(|i| {
                self.controller_view
                    .push_event(ViewportEvent::ModifiersChanged(Modifiers {
                        alt: i.modifiers.alt,
                        shift: i.modifiers.shift,
                        ctrl: i.modifiers.command,
                    }));
                let local = i
                    .pointer
                    .interact_pos()
                    .map(|p| glam::Vec2::new(p.x - rect.left(), p.y - rect.top()));
                self.cursor_vp = local;
                if let Some(pos) = local {
                    self.controller_view
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
                            self.controller_view.push_event(ViewportEvent::MouseButton {
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
                            self.controller_view.push_event(ViewportEvent::Wheel {
                                delta: glam::Vec2::new(delta.x, delta.y),
                                units,
                            });
                        }
                        _ => {}
                    }
                }
            });

            self.controller_view.apply_to_camera(&mut self.camera);
            self.camera.set_aspect_ratio(rect.width(), rect.height());

            // Move the character before stepping the runtime, so the new
            // scene transform is what Prepare sees this frame.
            self.move_character(dt_wall, move_dir);

            let mut frame_ctx = RuntimeFrameContext::default();
            frame_ctx.dt = dt_wall;
            frame_ctx.camera = self.camera.clone();
            frame_ctx.viewport_size = glam::Vec2::new(rect.width(), rect.height());
            self.runtime
                .step(&mut self.scene, &mut self.selection, &frame_ctx);

            let mut items: Vec<SceneRenderItem> = Vec::new();

            // Character (kinematic: render straight from the scene
            // transform, no interpolation since position is user-driven).
            if let Some(node) = self.scene.node(self.char_id) {
                let mut item = SceneRenderItem::default();
                item.mesh_id = self.char_mesh;
                item.model = node.world_transform().to_cols_array_2d();
                item.material = *node.material();
                items.push(item);
            }

            // Obstacles + floor.
            for &id in &self.obstacle_ids {
                if let Some(node) = self.scene.node(id) {
                    let mut item = SceneRenderItem::default();
                    item.mesh_id = self.box_mesh;
                    item.model = node.world_transform().to_cols_array_2d();
                    item.material = *node.material();
                    items.push(item);
                }
            }
            if let Some(node) = self.scene.node(self.floor_id) {
                let mut item = SceneRenderItem::default();
                item.mesh_id = self.floor_mesh;
                item.model = node.world_transform().to_cols_array_2d();
                item.material = *node.material();
                items.push(item);
            }

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

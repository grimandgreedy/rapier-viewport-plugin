//! A kinematic box driven by the arrow keys, pushing a pile of dynamic
//! balls. Demonstrates the prepare-phase sync path: the pusher's scene
//! transform is updated each frame from keyboard input, and
//! `RapierPreparePlugin` mirrors it into the rapier kinematic body every
//! step. Rapier converts the position change into an implicit velocity
//! that pushes the dynamic balls.
//!
//! Controls:
//!   Arrow keys / WASD: move pusher (X/Y plane)
//!   Left-drag: orbit, Right-drag: pan, Scroll: zoom
//!
//! Run with: `cargo run --example kinematic_pusher`

use eframe::egui;
use rapier_viewport_plugin::{
    parry3d::shape::SharedShape,
    rapier3d::prelude::{ColliderBuilder, RigidBodyBuilder},
    RapierPlugin,
};
use viewport_lib::{
    BackfacePolicy, ButtonState, Camera, CameraFrame, FixedTimestep, FrameData, LightingSettings,
    Colour, Material, MaterialId, MeshId, Modifiers, MouseButton, NodeId, OrbitCameraController,
    RuntimeFrameContext, Scene, SceneFrame, SceneRenderItem, ScrollUnits, Selection,
    ViewportContext, ViewportEvent, ViewportRenderer, ViewportRuntime, primitives,
};
use viewport_lib::{InputOwnership, PointerOwnership};

fn main() -> eframe::Result {
    eframe::run_native(
        "rapier: Kinematic Pusher",
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

            let sphere_mesh = res
                .upload_mesh_data(device, &primitives::sphere(BALL_RADIUS, 24, 12))
                .expect("sphere mesh upload");
            let floor_mesh = res
                .upload_mesh_data(device, &primitives::plane(20.0, 20.0))
                .expect("floor mesh upload");
            let box_mesh = res
                .upload_mesh_data(
                    device,
                    &primitives::cuboid(
                        2.0 * PUSHER_HALF.x,
                        2.0 * PUSHER_HALF.y,
                        2.0 * PUSHER_HALF.z,
                    ),
                )
                .expect("box mesh upload");

            let materials = Materials::upload(res);

            rs.renderer.write().callback_resources.insert(renderer);
            Ok(Box::new(App::new(sphere_mesh, floor_mesh, box_mesh, materials)))
        }),
    )
}

// ---------------------------------------------------------------------------

const SIM_HZ: f32 = 120.0;
const BALL_RADIUS: f32 = 0.5;
const BALL_COUNT: usize = 8;
const PUSHER_HALF: glam::Vec3 = glam::Vec3::new(0.6, 0.6, 0.6);
const PUSHER_SPEED: f32 = 5.0;

// ---------------------------------------------------------------------------

const BALL_COLOURS: [[f32; 3]; 8] = [
    [0.9, 0.3, 0.3],
    [0.9, 0.6, 0.2],
    [0.8, 0.8, 0.2],
    [0.3, 0.8, 0.3],
    [0.2, 0.7, 0.8],
    [0.3, 0.4, 0.9],
    [0.7, 0.3, 0.9],
    [0.9, 0.3, 0.7],
];

/// Materials uploaded once at setup and shared by every node of a kind.
struct Materials {
    floor: MaterialId,
    pusher: MaterialId,
    balls: Vec<MaterialId>,
}

impl Materials {
    fn upload(res: &mut viewport_lib::DeviceResources) -> Self {
        let mut floor = Material::from_colour(Colour::from_linear_rgb_array([0.35, 0.35, 0.35]));
        floor.backface_policy = BackfacePolicy::Identical;
        let mut upload = |m: Material| res.upload_material(m).expect("material upload");
        Self {
            floor: upload(floor),
            pusher: upload(Material::from_colour(Colour::from_linear_rgb_array([0.3, 0.5, 0.9]))),
            balls: BALL_COLOURS.iter().map(|&c| upload(Material::from_colour(Colour::from_linear_rgb_array(c)))).collect(),
        }
    }
}

struct App {
    camera: Camera,
    controller: OrbitCameraController,
    sphere_mesh: MeshId,
    floor_mesh: MeshId,
    box_mesh: MeshId,
    materials: Materials,

    scene: Scene,
    selection: Selection,
    runtime: ViewportRuntime,
    rapier: RapierPlugin,

    floor_id: NodeId,
    pusher_id: NodeId,
    ball_ids: Vec<NodeId>,
    pusher_pos: glam::Vec3,

    cursor_vp: Option<glam::Vec2>,
    left_held: bool,
    drag_started: bool,
    press_origin: Option<glam::Vec2>,
}

impl App {
    fn new(
        sphere_mesh: MeshId,
        floor_mesh: MeshId,
        box_mesh: MeshId,
        materials: Materials,
    ) -> Self {
        let mut app = Self {
            camera: Camera::orbit(
                glam::DVec3::new(0.0, 0.0, 1.0),
                16.0,
                Camera::default().orientation(),
            ),
            controller: OrbitCameraController::viewport_primitives(),
            sphere_mesh,
            floor_mesh,
            box_mesh,
            materials,
            scene: Scene::new(),
            selection: Selection::new(),
            runtime: ViewportRuntime::new(),
            rapier: RapierPlugin::new(),
            floor_id: NodeId::default(),
            pusher_id: NodeId::default(),
            ball_ids: Vec::new(),
            pusher_pos: glam::Vec3::ZERO,
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

        self.floor_id = self.scene.add(
            Some(self.floor_mesh),
            glam::Mat4::IDENTITY,
            self.materials.floor,
        );
        self.rapier.add_body(
            self.floor_id,
            RigidBodyBuilder::fixed().build(),
            ColliderBuilder::new(SharedShape::halfspace(glam::Vec3::Z)).build(),
        );

        // Kinematic pusher.
        self.pusher_pos = glam::Vec3::new(-3.5, 0.0, PUSHER_HALF.z);
        self.pusher_id = self.scene.add(
            Some(self.box_mesh),
            glam::Mat4::from_translation(self.pusher_pos),
            self.materials.pusher,
        );
        // Pusher: a kinematic_position_based body. Its transform is driven by
        // the scene each step (see the keyboard-driven `scene.set_local_transform`
        // in `update`). Rapier turns the position delta into the implicit
        // velocity that pushes other bodies.
        self.rapier.add_body(
            self.pusher_id,
            RigidBodyBuilder::kinematic_position_based()
                .translation(self.pusher_pos)
                .build(),
            ColliderBuilder::cuboid(PUSHER_HALF.x, PUSHER_HALF.y, PUSHER_HALF.z).build(),
        );

        // A pile of balls in front of the pusher.
        self.ball_ids.clear();
        for i in 0..BALL_COUNT {
            let fi = i as f32;
            let row = (i / 4) as f32;
            let col = (i % 4) as f32;
            let pos = glam::Vec3::new(
                1.5 + col * (BALL_RADIUS * 2.2),
                -1.0 + row * (BALL_RADIUS * 2.2) + (fi * 0.6).sin() * 0.05,
                BALL_RADIUS,
            );
            let id = self.scene.add(
                Some(self.sphere_mesh),
                glam::Mat4::from_translation(pos),
                self.materials.balls[i % self.materials.balls.len()],
            );
            let body = RigidBodyBuilder::dynamic().translation(pos).build();
            let collider = ColliderBuilder::ball(BALL_RADIUS)
                .restitution(0.2)
                .friction(0.4)
                .build();
            self.rapier.add_body(id, body, collider);
            self.ball_ids.push(id);
        }

        let (prepare, simulate) = self.rapier.clone().into_plugins();
        self.runtime = ViewportRuntime::new()
            .with_fixed_timestep(FixedTimestep::new(SIM_HZ))
            .with_plugin(prepare)
            .with_plugin(simulate);
    }
}

// ---------------------------------------------------------------------------

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Read keyboard intent for the pusher up front, before the viewport
        // input closure consumes events. egui's key_down state is global so
        // this is independent of viewport focus.
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
            (dir, i.stable_dt.min(0.25))
        });

        egui::SidePanel::left("controls")
            .exact_width(220.0)
            .show(ctx, |ui| {
                ui.add_space(8.0);
                ui.heading("Kinematic Pusher");
                ui.add_space(4.0);
                ui.separator();
                ui.add_space(6.0);

                ui.label("Arrow keys / WASD to move");
                ui.add_space(6.0);
                ui.label(format!(
                    "Pusher: ({:+.2}, {:+.2}, {:+.2})",
                    self.pusher_pos.x, self.pusher_pos.y, self.pusher_pos.z
                ));
                ui.add_space(10.0);

                if ui.button("Reset").clicked() {
                    self.reset();
                }
            });

        egui::CentralPanel::default().show(ctx, |ui| {
            let (rect, response) =
                ui.allocate_exact_size(ui.available_size(), egui::Sense::click_and_drag());

            self.drag_started = false;
            let pointer = if response.hovered() || response.dragged() {
                PointerOwnership::Owned
            } else {
                PointerOwnership::Elsewhere
            };
            self.controller.begin_frame(ViewportContext::new(
                InputOwnership::pointer_only(pointer),
                [rect.width(), rect.height()],
            ));

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

            // Advance the pusher's scene transform from keyboard input. The
            // Rapier prepare plugin reads this every fixed step and converts
            // the position change into the kinematic body's implicit velocity.
            let delta = glam::Vec3::new(move_dir.x, move_dir.y, 0.0) * PUSHER_SPEED * dt_wall;
            self.pusher_pos += delta;
            self.scene.set_local_transform(
                self.pusher_id,
                glam::Mat4::from_translation(self.pusher_pos),
            );

            let mut frame_ctx = RuntimeFrameContext::default();
            frame_ctx.dt = dt_wall;
            frame_ctx.camera = self.camera.clone();
            frame_ctx.viewport_size = glam::Vec2::new(rect.width(), rect.height());
            self.runtime
                .step(&mut self.scene, &mut self.selection, &frame_ctx);

            let alpha = self.runtime.alpha();
            let snapshots = self.runtime.snapshots();

            let mut items: Vec<SceneRenderItem> = self
                .ball_ids
                .iter()
                .filter_map(|&id| {
                    let world = self.scene.node(id)?.world_transform();
                    let model = snapshots
                        .interpolated(id, alpha)
                        .map(viewport_lib::Transform::from)
                        .unwrap_or(world);
                    let mat = self.scene.node(id)?.material();
                    let mut item = SceneRenderItem::new(self.sphere_mesh, mat);
                    item.model = model;
                    Some(item)
                })
                .collect();

            // Pusher (kinematic: render from its scene transform directly, no
            // physics interpolation since position is user-driven).
            if let Some(node) = self.scene.node(self.pusher_id) {
                let mut item = SceneRenderItem::new(self.box_mesh, node.material());
                item.model = node.world_transform();
                items.push(item);
            }

            if let Some(node) = self.scene.node(self.floor_id) {
                let mut floor_item = SceneRenderItem::new(self.floor_mesh, node.material());
                floor_item.model = node.world_transform();
                items.push(floor_item);
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

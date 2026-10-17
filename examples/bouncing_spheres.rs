//! Bouncing spheres inside a closed box. Demonstrates contact events:
//! colliders with `ActiveEvents::COLLISION_EVENTS` set produce
//! `RapierContactEvent`s on the runtime output bus, which the app drains
//! each frame and counts in the side panel.
//!
//! Controls: left-drag to orbit, right-drag to pan, scroll to zoom.
//!
//! Run with: `cargo run --example bouncing_spheres`

use eframe::egui;
use rapier_viewport_plugin::{
    parry3d::shape::SharedShape,
    rapier3d::prelude::{ActiveEvents, ColliderBuilder, RigidBodyBuilder},
    RapierContactEvent, RapierPlugin,
};
use viewport_lib::{
    BackfacePolicy, ButtonState, Camera, CameraFrame, FixedTimestep, FrameData, LightingSettings,
    Colour, Material, MaterialId, MeshId, Modifiers, MouseButton, NodeId, OrbitCameraController,
    RuntimeFrameContext, Scene, SceneFrame, SceneRenderItem, ScrollUnits, Selection,
    ViewportContext, ViewportEvent, ViewportRenderer, ViewportRuntime, primitives,
};

fn main() -> eframe::Result {
    eframe::run_native(
        "rapier: Bouncing Spheres",
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
                .upload_mesh_data(device, &primitives::plane(2.0 * BOX_HALF, 2.0 * BOX_HALF))
                .expect("floor mesh upload");
            let wall_mesh = res
                .upload_mesh_data(
                    device,
                    &primitives::cuboid(2.0 * BOX_HALF, 2.0 * WALL_THICKNESS, 2.0 * WALL_HEIGHT),
                )
                .expect("wall mesh upload");

            let materials = Materials::upload(res);

            rs.renderer.write().callback_resources.insert(renderer);
            Ok(Box::new(App::new(sphere_mesh, floor_mesh, wall_mesh, materials)))
        }),
    )
}

// ---------------------------------------------------------------------------

const SIM_HZ: f32 = 120.0;
const BALL_RADIUS: f32 = 0.45;
const BALL_COUNT: usize = 8;
const BOX_HALF: f32 = 5.0;
const WALL_THICKNESS: f32 = 0.25;
// Half-extent: full wall height = 2 * WALL_HEIGHT, just enough to keep the
// balls in.
const WALL_HEIGHT: f32 = 0.6;
const LAUNCH_SPEED: f32 = 13.0;

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

// ---------------------------------------------------------------------------

/// Materials uploaded once at setup and shared by every node of a kind.
struct Materials {
    floor: MaterialId,
    wall: MaterialId,
    balls: Vec<MaterialId>,
}

impl Materials {
    fn upload(res: &mut viewport_lib::DeviceResources) -> Self {
        let mut floor = Material::from_colour(Colour::from_linear_rgb_array([0.35, 0.35, 0.35]));
        floor.backface_policy = BackfacePolicy::Identical;
        let mut upload = |m: Material| res.upload_material(m).expect("material upload");
        Self {
            floor: upload(floor),
            wall: upload(Material::from_colour(Colour::from_linear_rgb_array([0.5, 0.5, 0.55]))),
            balls: BALL_COLOURS.iter().map(|&c| upload(Material::from_colour(Colour::from_linear_rgb_array(c)))).collect(),
        }
    }
}

struct App {
    camera: Camera,
    controller: OrbitCameraController,
    sphere_mesh: MeshId,
    floor_mesh: MeshId,
    wall_mesh: MeshId,
    materials: Materials,

    scene: Scene,
    selection: Selection,
    runtime: ViewportRuntime,
    rapier: RapierPlugin,

    floor_id: NodeId,
    wall_ids: Vec<NodeId>,
    ball_ids: Vec<NodeId>,

    contact_started: u64,
    contact_stopped: u64,

    cursor_vp: Option<glam::Vec2>,
    left_held: bool,
    drag_started: bool,
    press_origin: Option<glam::Vec2>,
}

impl App {
    fn new(
        sphere_mesh: MeshId,
        floor_mesh: MeshId,
        wall_mesh: MeshId,
        materials: Materials,
    ) -> Self {
        let mut app = Self {
            camera: Camera {
                center: glam::Vec3::new(0.0, 0.0, 1.5),
                distance: 18.0,
                ..Camera::default()
            },
            controller: OrbitCameraController::viewport_primitives(),
            sphere_mesh,
            floor_mesh,
            wall_mesh,
            materials,
            scene: Scene::new(),
            selection: Selection::new(),
            runtime: ViewportRuntime::new(),
            rapier: RapierPlugin::new(),
            floor_id: NodeId::default(),
            wall_ids: Vec::new(),
            ball_ids: Vec::new(),
            contact_started: 0,
            contact_stopped: 0,
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
        self.contact_started = 0;
        self.contact_stopped = 0;

        // Floor.
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

        // Four walls forming a square arena around the origin. Each wall is a
        // thin cuboid placed at one edge of the box.
        self.wall_ids.clear();
        let wall_offsets = [
            (glam::Vec3::new(0.0, BOX_HALF, WALL_HEIGHT), glam::Quat::IDENTITY),
            (glam::Vec3::new(0.0, -BOX_HALF, WALL_HEIGHT), glam::Quat::IDENTITY),
            (
                glam::Vec3::new(BOX_HALF, 0.0, WALL_HEIGHT),
                glam::Quat::from_rotation_z(std::f32::consts::FRAC_PI_2),
            ),
            (
                glam::Vec3::new(-BOX_HALF, 0.0, WALL_HEIGHT),
                glam::Quat::from_rotation_z(std::f32::consts::FRAC_PI_2),
            ),
        ];
        for (pos, rot) in wall_offsets {
            let id = self.scene.add(
                Some(self.wall_mesh),
                glam::Mat4::from_rotation_translation(rot, pos),
                self.materials.wall,
            );
            let body = RigidBodyBuilder::fixed()
                .translation(pos)
                .rotation(glam::Quat::to_scaled_axis(rot))
                .build();
            let collider = ColliderBuilder::cuboid(BOX_HALF, WALL_THICKNESS, WALL_HEIGHT)
                .restitution(0.9)
                .build();
            self.rapier.add_body(id, body, collider);
            self.wall_ids.push(id);
        }

        // Balls launched inward from positions around the box edges.
        self.ball_ids.clear();
        for i in 0..BALL_COUNT {
            let fi = i as f32;
            // Place balls on a ring inside the arena, with velocities aimed at
            // the centre
            let angle = fi * std::f32::consts::TAU / BALL_COUNT as f32;
            let radius = BOX_HALF - 1.0;
            // Spawn just above the floor, inside the wall height, with a
            // bit of jitter so the balls don't all stack at the same Z.
            let pos = glam::Vec3::new(
                angle.cos() * radius,
                angle.sin() * radius,
                BALL_RADIUS + 0.05 + (fi * 0.3).sin().abs() * 0.2,
            );
            // Inward + a bit of swirl.
            let inward = -glam::Vec3::new(angle.cos(), angle.sin(), 0.0);
            let tangent = glam::Vec3::new(-angle.sin(), angle.cos(), 0.0);
            let vel = inward * LAUNCH_SPEED + tangent * (LAUNCH_SPEED * 0.3);

            let id = self.scene.add(
                Some(self.sphere_mesh),
                glam::Mat4::from_translation(pos),
                self.materials.balls[i % self.materials.balls.len()],
            );
            // ActiveEvents::COLLISION_EVENTS on the collider is what makes
            // this body emit `RapierContactEvent`s on the runtime output bus.
            let body = RigidBodyBuilder::dynamic()
                .translation(pos)
                .linvel(vel)
                .build();
            let collider = ColliderBuilder::ball(BALL_RADIUS)
                .restitution(0.92)
                .friction(0.1)
                .active_events(ActiveEvents::COLLISION_EVENTS)
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
        egui::SidePanel::left("controls")
            .exact_width(220.0)
            .show(ctx, |ui| {
                ui.add_space(8.0);
                ui.heading("Bouncing Spheres");
                ui.add_space(4.0);
                ui.separator();
                ui.add_space(6.0);

                ui.label(format!("Balls:    {}", self.ball_ids.len()));
                ui.label(format!("Walls:    {}", self.wall_ids.len() + 1));
                ui.add_space(8.0);
                ui.label(egui::RichText::new("Contact events").strong());
                egui::Grid::new("ev").num_columns(2).striped(true).show(ui, |ui| {
                    ui.label("Started");
                    ui.label(self.contact_started.to_string());
                    ui.end_row();
                    ui.label("Stopped");
                    ui.label(self.contact_stopped.to_string());
                    ui.end_row();
                });
                ui.add_space(10.0);

                if ui.button("Reset").clicked() {
                    self.reset();
                }
            });

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
                self.controller
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

            let mut frame_ctx = RuntimeFrameContext::default();
            frame_ctx.dt = dt;
            frame_ctx.camera = self.camera.clone();
            frame_ctx.viewport_size = glam::Vec2::new(rect.width(), rect.height());
            let mut output =
                self.runtime
                    .step(&mut self.scene, &mut self.selection, &frame_ctx);

            // Contact events for this step. Both colliders need
            // `ActiveEvents::COLLISION_EVENTS` for an event to fire;
            // see how the ball/wall colliders are built in `reset()`.
            for ev in output.events.drain::<RapierContactEvent>() {
                if ev.started {
                    self.contact_started += 1;
                } else {
                    self.contact_stopped += 1;
                }
            }

            let alpha = self.runtime.alpha();
            let snapshots = self.runtime.snapshots();

            let mut items: Vec<SceneRenderItem> = self
                .ball_ids
                .iter()
                .filter_map(|&id| {
                    let world = self.scene.node(id)?.world_transform();
                    let model = snapshots
                        .interpolated(id, alpha)
                        .map(glam::Mat4::from)
                        .unwrap_or(world);
                    let mat = self.scene.node(id)?.material();
                    let mut item = SceneRenderItem::new(self.sphere_mesh, mat);
                    item.model = model.to_cols_array_2d();
                    Some(item)
                })
                .collect();

            for &wall_id in &self.wall_ids {
                if let Some(node) = self.scene.node(wall_id) {
                    let mut item = SceneRenderItem::new(self.wall_mesh, node.material());
                    item.model = node.world_transform().to_cols_array_2d();
                    items.push(item);
                }
            }

            if let Some(node) = self.scene.node(self.floor_id) {
                let mut floor_item = SceneRenderItem::new(self.floor_mesh, node.material());
                floor_item.model = node.world_transform().to_cols_array_2d();
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

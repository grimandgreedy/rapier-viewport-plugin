//! A trigger volume that detects bodies entering and leaving it without
//! exerting any contact forces. Balls fall through the sensor on their way
//! to the floor; the panel counts entry / exit events.
//!
//! Demonstrates:
//!
//! - `ColliderBuilder::sensor(true)` -- a "ghost" collider that produces
//!   events but no contact forces.
//! - `ActiveEvents::COLLISION_EVENTS` on both the sensor and the bodies
//!   you want to track.
//! - Reading `RapierContactEvent`s from the runtime output bus and
//!   filtering them down to ones involving the sensor body.
//!
//! Run with: `cargo run --example sensor_trigger`

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
        "rapier: Sensor Trigger",
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

            let ball_mesh = res
                .upload_mesh_data(device, &primitives::sphere(BALL_RADIUS, 24, 12))
                .expect("ball mesh upload");
            let floor_mesh = res
                .upload_mesh_data(device, &primitives::plane(20.0, 20.0))
                .expect("floor mesh upload");
            let sensor_mesh = res
                .upload_mesh_data(
                    device,
                    &primitives::cuboid(
                        2.0 * SENSOR_HALF.x,
                        2.0 * SENSOR_HALF.y,
                        2.0 * SENSOR_HALF.z,
                    ),
                )
                .expect("sensor mesh upload");

            let materials = Materials::upload(res);

            rs.renderer.write().callback_resources.insert(renderer);
            Ok(Box::new(App::new(ball_mesh, floor_mesh, sensor_mesh, materials)))
        }),
    )
}

// ---------------------------------------------------------------------------

const SIM_HZ: f32 = 120.0;
const BALL_RADIUS: f32 = 0.4;
const BALL_COUNT: usize = 4;
const SENSOR_HALF: glam::Vec3 = glam::Vec3::new(2.0, 2.0, 0.4);
const SENSOR_Z: f32 = 2.5;

const SENSOR_COLOUR_IDLE: [f32; 3] = [0.3, 0.5, 0.9];
const SENSOR_COLOUR_HOT: [f32; 3] = [1.0, 0.7, 0.2];

// ---------------------------------------------------------------------------

const BALL_COLOURS: [[f32; 3]; 4] = [
    [0.9, 0.3, 0.3],
    [0.8, 0.8, 0.2],
    [0.3, 0.8, 0.3],
    [0.7, 0.3, 0.9],
];

/// Materials uploaded once at setup and shared by every node of a kind.
struct Materials {
    floor: MaterialId,
    sensor_idle: MaterialId,
    sensor_hot: MaterialId,
    balls: Vec<MaterialId>,
}

impl Materials {
    fn upload(res: &mut viewport_lib::DeviceResources) -> Self {
        let mut floor = Material::from_colour(Colour::from_linear_rgb_array([0.35, 0.35, 0.35]));
        floor.backface_policy = BackfacePolicy::Identical;
        let mut upload = |m: Material| res.upload_material(m).expect("material upload");
        Self {
            floor: upload(floor),
            sensor_idle: upload(Material::from_colour(Colour::from_linear_rgb_array(SENSOR_COLOUR_IDLE))),
            sensor_hot: upload(Material::from_colour(Colour::from_linear_rgb_array(SENSOR_COLOUR_HOT))),
            balls: BALL_COLOURS.iter().map(|&c| upload(Material::from_colour(Colour::from_linear_rgb_array(c)))).collect(),
        }
    }
}

struct App {
    camera: Camera,
    controller: OrbitCameraController,
    ball_mesh: MeshId,
    floor_mesh: MeshId,
    sensor_mesh: MeshId,
    materials: Materials,

    scene: Scene,
    selection: Selection,
    runtime: ViewportRuntime,
    rapier: RapierPlugin,

    floor_id: NodeId,
    sensor_id: NodeId,
    ball_ids: Vec<NodeId>,

    entered: u64,
    exited: u64,
    inside_now: i64,

    cursor_vp: Option<glam::Vec2>,
    left_held: bool,
    drag_started: bool,
    press_origin: Option<glam::Vec2>,
}

impl App {
    fn new(
        ball_mesh: MeshId,
        floor_mesh: MeshId,
        sensor_mesh: MeshId,
        materials: Materials,
    ) -> Self {
        let mut app = Self {
            camera: Camera {
                center: glam::Vec3::new(0.0, 0.0, 2.0),
                distance: 14.0,
                ..Camera::default()
            },
            controller: OrbitCameraController::viewport_primitives(),
            ball_mesh,
            floor_mesh,
            sensor_mesh,
            materials,
            scene: Scene::new(),
            selection: Selection::new(),
            runtime: ViewportRuntime::new(),
            rapier: RapierPlugin::new(),
            floor_id: NodeId::default(),
            sensor_id: NodeId::default(),
            ball_ids: Vec::new(),
            entered: 0,
            exited: 0,
            inside_now: 0,
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
        self.entered = 0;
        self.exited = 0;
        self.inside_now = 0;

        // ---- Floor: solid collider. ---------------------------------------
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

        // ---- Sensor cube: `sensor(true)` makes it a ghost collider that
        // generates events but no contact forces. -------------------------
        let sensor_pos = glam::Vec3::new(0.0, 0.0, SENSOR_Z);
        self.sensor_id = self.scene.add(
            Some(self.sensor_mesh),
            glam::Mat4::from_translation(sensor_pos),
            self.materials.sensor_idle,
        );
        self.rapier.add_body(
            self.sensor_id,
            RigidBodyBuilder::fixed().translation(sensor_pos).build(),
            ColliderBuilder::cuboid(SENSOR_HALF.x, SENSOR_HALF.y, SENSOR_HALF.z)
                .sensor(true)
                .active_events(ActiveEvents::COLLISION_EVENTS)
                .build(),
        );

        // ---- Falling balls. -----------------------------------------------
        // Both the sensor and the ball need ActiveEvents::COLLISION_EVENTS
        // for an event to be emitted -- rapier fires the event if *either*
        // collider has the flag, so we set it on the balls too to be safe.
        self.ball_ids.clear();
        for i in 0..BALL_COUNT {
            let fi = i as f32;
            let pos = glam::Vec3::new(
                (fi - 1.5) * 0.9,
                (fi * 0.7).sin() * 0.4,
                6.0 + fi * 0.6,
            );
            let id = self.scene.add(
                Some(self.ball_mesh),
                glam::Mat4::from_translation(pos),
                self.materials.balls[i % self.materials.balls.len()],
            );
            self.rapier.add_body(
                id,
                RigidBodyBuilder::dynamic().translation(pos).build(),
                ColliderBuilder::ball(BALL_RADIUS)
                    .restitution(0.2)
                    .active_events(ActiveEvents::COLLISION_EVENTS)
                    .build(),
            );
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
                ui.heading("Sensor Trigger");
                ui.add_space(4.0);
                ui.separator();
                ui.add_space(6.0);

                ui.label("Balls fall through the sensor cube,");
                ui.label("which records entries and exits");
                ui.label("without exerting force.");
                ui.add_space(10.0);

                egui::Grid::new("counts").num_columns(2).striped(true).show(ui, |ui| {
                    ui.label("Entries");
                    ui.label(self.entered.to_string());
                    ui.end_row();
                    ui.label("Exits");
                    ui.label(self.exited.to_string());
                    ui.end_row();
                    ui.label("Inside now");
                    ui.label(self.inside_now.to_string());
                    ui.end_row();
                });

                ui.add_space(12.0);
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

            // Drain contact events. We only care about events whose pair
            // involves the sensor; ball-on-ball and ball-on-floor are ignored.
            for ev in output.events.drain::<RapierContactEvent>() {
                let pair_has_sensor = ev.node_a == self.sensor_id || ev.node_b == self.sensor_id;
                if !pair_has_sensor {
                    continue;
                }
                if ev.started {
                    self.entered += 1;
                    self.inside_now += 1;
                } else {
                    self.exited += 1;
                    self.inside_now -= 1;
                }
            }

            let alpha = self.runtime.alpha();
            let snapshots = self.runtime.snapshots();

            let mut items: Vec<SceneRenderItem> = self
                .ball_ids
                .iter()
                .filter_map(|&id| {
                    let node = self.scene.node(id)?;
                    let world = node.world_transform();
                    let model = snapshots
                        .interpolated(id, alpha)
                        .map(glam::Mat4::from)
                        .unwrap_or(world);
                    let mut item = SceneRenderItem::new(self.ball_mesh, node.material());
                    item.model = model.to_cols_array_2d();
                    Some(item)
                })
                .collect();

            // Sensor: tinted when something is inside, so the visual matches
            // the counter in the side panel.
            if let Some(node) = self.scene.node(self.sensor_id) {
                let material = if self.inside_now > 0 {
                    self.materials.sensor_hot
                } else {
                    self.materials.sensor_idle
                };
                let mut item = SceneRenderItem::new(self.sensor_mesh, material);
                item.model = node.world_transform().to_cols_array_2d();
                items.push(item);
            }

            if let Some(node) = self.scene.node(self.floor_id) {
                let mut item = SceneRenderItem::new(self.floor_mesh, node.material());
                item.model = node.world_transform().to_cols_array_2d();
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

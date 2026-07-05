//! Ten boxes stacked above a static floor. Demonstrates the basics: register
//! one fixed body and several dynamic bodies, then let the runtime step the
//! simulation each frame. Dynamic body transforms are written back to the
//! scene automatically by `RapierSimulatePlugin`.
//!
//! Controls: left-drag to orbit, right-drag to pan, scroll to zoom.
//!
//! Run with: `cargo run --example falling_stack`

use eframe::egui;
use rapier_viewport_plugin::{
    parry3d::shape::SharedShape,
    rapier3d::prelude::{ColliderBuilder, RigidBodyBuilder},
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
        "rapier: Falling Stack",
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

            let box_mesh = res
                .upload_mesh_data(device, &primitives::cuboid(1.0, 1.0, 1.0))
                .expect("box mesh upload");
            let floor_mesh = res
                .upload_mesh_data(device, &primitives::plane(20.0, 20.0))
                .expect("floor mesh upload");

            rs.renderer.write().callback_resources.insert(renderer);
            Ok(Box::new(App::new(box_mesh, floor_mesh)))
        }),
    )
}

// ---------------------------------------------------------------------------

const BOX_COUNT: usize = 10;
const SIM_HZ: f32 = 120.0;

// ---------------------------------------------------------------------------

struct App {
    camera: Camera,
    controller: OrbitCameraController,
    box_mesh: MeshId,
    floor_mesh: MeshId,

    scene: Scene,
    selection: Selection,
    runtime: ViewportRuntime,
    rapier: RapierPlugin,

    floor_id: NodeId,
    box_ids: Vec<NodeId>,

    cursor_vp: Option<glam::Vec2>,
    left_held: bool,
    drag_started: bool,
    press_origin: Option<glam::Vec2>,
}

impl App {
    fn new(box_mesh: MeshId, floor_mesh: MeshId) -> Self {
        let mut app = Self {
            camera: Camera {
                center: glam::Vec3::new(0.0, 0.0, 5.0),
                distance: 22.0,
                ..Camera::default()
            },
            controller: OrbitCameraController::viewport_primitives(),
            box_mesh,
            floor_mesh,
            scene: Scene::new(),
            selection: Selection::new(),
            runtime: ViewportRuntime::new(),
            rapier: RapierPlugin::new(),
            floor_id: NodeId::default(),
            box_ids: Vec::new(),
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

        // ---- Floor: a fixed body with a halfspace collider (infinite plane). ----
        // Add the scene node first so it can be rendered, then bind a rapier
        // body+collider to the same NodeId via `add_body`.
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

        let colours = [
            [0.9, 0.3, 0.3],
            [0.9, 0.6, 0.2],
            [0.8, 0.8, 0.2],
            [0.3, 0.8, 0.3],
            [0.2, 0.7, 0.8],
            [0.3, 0.4, 0.9],
            [0.7, 0.3, 0.9],
            [0.9, 0.3, 0.7],
            [0.6, 0.9, 0.5],
            [1.0, 0.6, 0.5],
        ];

        // ---- Stack of dynamic boxes. ---------------------------------------
        // Each gets a rapier dynamic rigid body and a cuboid collider. The
        // initial position goes on the RigidBodyBuilder; the simulate plugin
        // writes the body's pose back to the scene node every step.
        self.box_ids.clear();
        let half = glam::Vec3::splat(0.5);
        for i in 0..BOX_COUNT {
            let z = 5.0 + i as f32 * 1.05;
            let fi = i as f32;
            let jitter_x = (fi * 2.3).sin() * 0.12;
            let jitter_y = (fi * 1.7).cos() * 0.12;
            let pos = glam::Vec3::new(jitter_x, jitter_y, z);
            // Small lateral drift so adjacent boxes don't land perfectly centred.
            let vx = (fi * 1.1 + 0.5).sin() * 0.5;
            let vy = (fi * 0.8 + 1.2).cos() * 0.5;
            let colour = colours[i % colours.len()];
            let id = self.scene.add(
                Some(self.box_mesh),
                glam::Mat4::from_translation(pos),
                Material::from_colour(colour),
            );
            let body = RigidBodyBuilder::dynamic()
                .translation(pos)
                .linvel(glam::Vec3::new(vx, vy, 0.0))
                .build();
            let collider = ColliderBuilder::cuboid(half.x, half.y, half.z).build();
            self.rapier.add_body(id, body, collider);
            self.box_ids.push(id);
        }

        // ---- Wire the plugin into the runtime. ----------------------------
        // `into_plugins` splits the handle into the Prepare-phase and
        // Simulate-phase RuntimePlugins. Cloning self.rapier first keeps the
        // handle around so we can still add/remove bodies later.
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
            .exact_width(180.0)
            .show(ctx, |ui| {
                ui.add_space(8.0);
                ui.heading("Falling Stack");
                ui.add_space(4.0);
                ui.separator();
                ui.add_space(6.0);

                ui.label(format!("Bodies: {}", self.box_ids.len() + 1));
                ui.add_space(8.0);

                if ui.button("Reset").clicked() {
                    self.reset();
                }

                ui.add_space(8.0);
                ui.separator();
                ui.add_space(4.0);
                ui.label("Left-drag: orbit");
                ui.label("Right-drag: pan");
                ui.label("Scroll: zoom");
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

            // Step the runtime once per frame. The Prepare plugin syncs
            // fixed/kinematic transforms from the scene; the Simulate plugin
            // advances rapier and writes dynamic body poses back to the scene.
            let mut frame_ctx = RuntimeFrameContext::default();
            frame_ctx.dt = dt;
            frame_ctx.camera = self.camera.clone();
            frame_ctx.viewport_size = glam::Vec2::new(rect.width(), rect.height());
            self.runtime
                .step(&mut self.scene, &mut self.selection, &frame_ctx);

            let alpha = self.runtime.alpha();
            let snapshots = self.runtime.snapshots();

            let mut items: Vec<SceneRenderItem> = self
                .box_ids
                .iter()
                .filter_map(|&id| {
                    let world = self.scene.node(id)?.world_transform();
                    let model = snapshots
                        .interpolated(id, alpha)
                        .map(glam::Mat4::from)
                        .unwrap_or(world);
                    let mat = *self.scene.node(id)?.material();
                    let mut item = SceneRenderItem::default();
                    item.mesh_id = self.box_mesh;
                    item.model = model.to_cols_array_2d();
                    item.material = mat;
                    Some(item)
                })
                .collect();

            if let Some(node) = self.scene.node(self.floor_id) {
                let mut floor_item = SceneRenderItem::default();
                floor_item.mesh_id = self.floor_mesh;
                floor_item.model = node.world_transform().to_cols_array_2d();
                floor_item.material = *node.material();
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

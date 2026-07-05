//! Three colour-coded layers of balls. Red balls only collide with red,
//! green with green, blue with blue. Everything collides with the floor.
//!
//! Demonstrates rapier's `InteractionGroups`:
//!
//! - `memberships`: which group(s) this collider belongs to.
//! - `filter`: which group(s) this collider will collide with.
//! - Both are 32-bit bitmasks; a pair interacts iff each side's
//!   `memberships` overlaps the other's `filter`.
//!
//! Set on the collider via `ColliderBuilder::collision_groups(...)`.
//!
//! Run with: `cargo run --example collision_groups`

use eframe::egui;
use rapier_viewport_plugin::{
    parry3d::shape::SharedShape,
    rapier3d::prelude::{ColliderBuilder, Group, InteractionGroups, RigidBodyBuilder},
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
        "rapier: Collision Groups",
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
            let wall_mesh = res
                .upload_mesh_data(
                    device,
                    &primitives::cuboid(2.0 * PIT_HALF, 2.0 * WALL_THICK, 2.0 * WALL_HEIGHT),
                )
                .expect("wall mesh upload");

            rs.renderer.write().callback_resources.insert(renderer);
            Ok(Box::new(App::new(ball_mesh, floor_mesh, wall_mesh)))
        }),
    )
}

// ---------------------------------------------------------------------------

const SIM_HZ: f32 = 120.0;
const BALL_RADIUS: f32 = 0.35;
const BALLS_PER_GROUP: usize = 6;

/// Half-width of the containment pit. The walls keep all colours in a
/// shared volume so they can't drift apart -- which is what makes the
/// filtering visible (different colours intersect; same colours can't).
const PIT_HALF: f32 = 1.4;
const WALL_HEIGHT: f32 = 1.8;
const WALL_THICK: f32 = 0.15;

const COLOUR_RED: [f32; 3] = [0.9, 0.3, 0.3];
const COLOUR_GREEN: [f32; 3] = [0.3, 0.8, 0.4];
const COLOUR_BLUE: [f32; 3] = [0.3, 0.5, 0.95];

// ---------------------------------------------------------------------------

struct App {
    camera: Camera,
    controller: OrbitCameraController,
    ball_mesh: MeshId,
    floor_mesh: MeshId,
    wall_mesh: MeshId,

    scene: Scene,
    selection: Selection,
    runtime: ViewportRuntime,
    rapier: RapierPlugin,

    floor_id: NodeId,
    wall_ids: Vec<NodeId>,
    ball_ids: Vec<NodeId>,

    cursor_vp: Option<glam::Vec2>,
    left_held: bool,
    drag_started: bool,
    press_origin: Option<glam::Vec2>,
}

impl App {
    fn new(ball_mesh: MeshId, floor_mesh: MeshId, wall_mesh: MeshId) -> Self {
        let mut app = Self {
            camera: Camera {
                center: glam::Vec3::new(0.0, 0.0, 1.0),
                distance: 9.0,
                ..Camera::default()
            },
            controller: OrbitCameraController::viewport_primitives(),
            ball_mesh,
            floor_mesh,
            wall_mesh,
            scene: Scene::new(),
            selection: Selection::new(),
            runtime: ViewportRuntime::new(),
            rapier: RapierPlugin::new(),
            floor_id: NodeId::default(),
            wall_ids: Vec::new(),
            ball_ids: Vec::new(),
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

        // ---- Floor: belongs to every group and filters every group, so it
        // collides with every ball regardless of layer. -------------------
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
            ColliderBuilder::new(SharedShape::halfspace(glam::Vec3::Z))
                .collision_groups(InteractionGroups::all())
                .build(),
        );

        // ---- Pit walls: in every group so they constrain every ball. -----
        // Keeping the balls in a small shared volume is what makes the
        // filtering effect visible -- different-colour balls end up at the
        // same XY position, intersecting each other, because they don't
        // push each other apart.
        self.wall_ids.clear();
        let wall_colour = Material::from_colour([0.5, 0.5, 0.55]);
        let wall_z = WALL_HEIGHT;
        let walls = [
            (glam::Vec3::new(0.0, PIT_HALF, wall_z), glam::Quat::IDENTITY),
            (glam::Vec3::new(0.0, -PIT_HALF, wall_z), glam::Quat::IDENTITY),
            (
                glam::Vec3::new(PIT_HALF, 0.0, wall_z),
                glam::Quat::from_rotation_z(std::f32::consts::FRAC_PI_2),
            ),
            (
                glam::Vec3::new(-PIT_HALF, 0.0, wall_z),
                glam::Quat::from_rotation_z(std::f32::consts::FRAC_PI_2),
            ),
        ];
        for (pos, rot) in walls {
            let id = self.scene.add(
                Some(self.wall_mesh),
                glam::Mat4::from_rotation_translation(rot, pos),
                wall_colour,
            );
            self.rapier.add_body(
                id,
                RigidBodyBuilder::fixed()
                    .translation(pos)
                    .rotation(glam::Quat::to_scaled_axis(rot))
                    .build(),
                ColliderBuilder::cuboid(PIT_HALF, WALL_THICK, WALL_HEIGHT)
                    .collision_groups(InteractionGroups::all())
                    .build(),
            );
            self.wall_ids.push(id);
        }

        // ---- Three colours of balls, all dropped into the same pit. ------
        // Spawning in an interleaved column means balls of different colours
        // occupy the *same* vertical space. Same-colour balls collide and
        // stack; different-colour balls pass through each other and end up
        // intersecting in the pile -- which is precisely the effect the
        // demo is here to show.
        let groups_by_layer: [(Group, [f32; 3]); 3] = [
            (Group::GROUP_1, COLOUR_RED),
            (Group::GROUP_2, COLOUR_GREEN),
            (Group::GROUP_3, COLOUR_BLUE),
        ];

        self.ball_ids.clear();
        let total = BALLS_PER_GROUP * groups_by_layer.len();
        for i in 0..total {
            let (group, colour) = groups_by_layer[i % groups_by_layer.len()];
            // collision_groups: this collider is in `group`; it filters
            // `group` too -- only collides with same-group colliders (the
            // walls and floor are in InteractionGroups::all() so they hit
            // every colour).
            let groups = InteractionGroups::new(
                group,
                group,
                rapier_viewport_plugin::rapier3d::prelude::InteractionTestMode::And,
            );

            let fi = i as f32;
            // A small spiral of starting positions so the balls don't all
            // share an exact XY (which would just produce an unstable
            // pile). They still share enough space to interleave.
            let angle = fi * 0.7;
            let radius = 0.15 + (fi * 0.2).sin().abs() * 0.1;
            let pos = glam::Vec3::new(
                angle.cos() * radius,
                angle.sin() * radius,
                BALL_RADIUS + 0.3 + fi * (BALL_RADIUS * 1.6),
            );

            let id = self.scene.add(
                Some(self.ball_mesh),
                glam::Mat4::from_translation(pos),
                Material::from_colour(colour),
            );
            self.rapier.add_body(
                id,
                RigidBodyBuilder::dynamic().translation(pos).build(),
                ColliderBuilder::ball(BALL_RADIUS)
                    .restitution(0.3)
                    .friction(0.6)
                    .collision_groups(groups)
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
            .exact_width(240.0)
            .show(ctx, |ui| {
                ui.add_space(8.0);
                ui.heading("Collision Groups");
                ui.add_space(4.0);
                ui.separator();
                ui.add_space(6.0);

                ui.label("Each colour is a separate bitmask layer.");
                ui.label("Balls only collide with their own colour;");
                ui.label("the floor is in every layer.");
                ui.add_space(10.0);

                ui.colored_label(egui::Color32::from_rgb(230, 77, 77), "Red    GROUP_1");
                ui.colored_label(egui::Color32::from_rgb(77, 204, 102), "Green  GROUP_2");
                ui.colored_label(egui::Color32::from_rgb(77, 128, 242), "Blue   GROUP_3");

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
            self.runtime
                .step(&mut self.scene, &mut self.selection, &frame_ctx);

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
                    let mut item = SceneRenderItem::default();
                    item.mesh_id = self.ball_mesh;
                    item.model = model.to_cols_array_2d();
                    item.material = *node.material();
                    Some(item)
                })
                .collect();

            for &wall_id in &self.wall_ids {
                if let Some(node) = self.scene.node(wall_id) {
                    let mut item = SceneRenderItem::default();
                    item.mesh_id = self.wall_mesh;
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

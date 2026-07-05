//! A ray is cast from the camera straight ahead every frame. The body it
//! hits is highlighted; click or press space to mark it permanently black.
//!
//! Demonstrates the **escape-hatch** pattern: the plugin's curated API does
//! not include a raycast wrapper. Instead, `with_state` gives temporary
//! immutable access to the underlying `RapierState`, where we construct a
//! `QueryPipeline` via `BroadPhaseBvh::as_query_pipeline(...)` and call
//! rapier's own `cast_ray_and_get_normal` directly. The hit collider is
//! translated back to a `NodeId` through `state.node_for_collider(handle)`.
//!
//! For UI mouse-click selection use viewport-lib's pick API instead -- this
//! is the pattern for game-logic raycasts (AI sight-lines, projectile
//! pre-checks, character grounding).
//!
//! Controls: left-drag to orbit (aim the ray), left-click or space to
//! blacken the aimed body, right-drag to pan, scroll to zoom.
//!
//! Run with: `cargo run --example ray_probe`

use std::collections::HashSet;

use eframe::egui;
use rapier_viewport_plugin::{
    parry3d::{query::Ray, shape::SharedShape},
    rapier3d::{
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
        "rapier: Ray Probe",
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
            let sphere_mesh = res
                .upload_mesh_data(device, &primitives::sphere(0.5, 24, 12))
                .expect("sphere mesh upload");
            let floor_mesh = res
                .upload_mesh_data(device, &primitives::plane(20.0, 20.0))
                .expect("floor mesh upload");
            let marker_mesh = res
                .upload_mesh_data(device, &primitives::sphere(0.1, 16, 8))
                .expect("marker mesh upload");

            rs.renderer.write().callback_resources.insert(renderer);
            Ok(Box::new(App::new(
                box_mesh,
                sphere_mesh,
                floor_mesh,
                marker_mesh,
            )))
        }),
    )
}

// ---------------------------------------------------------------------------

const SIM_HZ: f32 = 120.0;
const RAY_MAX_DISTANCE: f32 = 200.0;

const HIGHLIGHT_COLOUR: [f32; 3] = [1.0, 0.95, 0.3];
const BLACKENED_COLOUR: [f32; 3] = [0.04, 0.04, 0.04];

// ---------------------------------------------------------------------------

struct App {
    camera: Camera,
    controller: OrbitCameraController,
    box_mesh: MeshId,
    sphere_mesh: MeshId,
    floor_mesh: MeshId,
    marker_mesh: MeshId,

    scene: Scene,
    selection: Selection,
    runtime: ViewportRuntime,
    rapier: RapierPlugin,

    floor_id: NodeId,
    body_ids: Vec<NodeId>,
    body_colours: Vec<[f32; 3]>,

    aimed: Option<HitInfo>,
    blackened: HashSet<NodeId>,
}

#[derive(Clone, Copy)]
struct HitInfo {
    node: NodeId,
    point: glam::Vec3,
    normal: glam::Vec3,
    distance: f32,
}

impl App {
    fn new(box_mesh: MeshId, sphere_mesh: MeshId, floor_mesh: MeshId, marker_mesh: MeshId) -> Self {
        let mut app = Self {
            camera: Camera {
                center: glam::Vec3::new(0.0, 0.0, 0.6),
                distance: 14.0,
                ..Camera::default()
            },
            controller: OrbitCameraController::viewport_primitives(),
            box_mesh,
            sphere_mesh,
            floor_mesh,
            marker_mesh,
            scene: Scene::new(),
            selection: Selection::new(),
            runtime: ViewportRuntime::new(),
            rapier: RapierPlugin::new(),
            floor_id: NodeId::default(),
            body_ids: Vec::new(),
            body_colours: Vec::new(),
            aimed: None,
            blackened: HashSet::new(),
        };
        app.reset();
        app
    }

    fn reset(&mut self) {
        self.scene = Scene::new();
        self.selection = Selection::new();
        self.rapier = RapierPlugin::new();
        self.aimed = None;
        self.blackened.clear();

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

        self.body_ids.clear();
        self.body_colours.clear();
        let palette = [
            [0.9, 0.3, 0.3],
            [0.9, 0.6, 0.2],
            [0.8, 0.8, 0.2],
            [0.3, 0.8, 0.3],
            [0.2, 0.7, 0.8],
            [0.3, 0.4, 0.9],
            [0.7, 0.3, 0.9],
            [0.9, 0.3, 0.7],
            [0.5, 0.9, 0.7],
        ];

        // 3x3 grid of mixed boxes and spheres. Includes the centre so the
        // forward ray from the camera always has a target near origin.
        let grid: &[(f32, f32, bool)] = &[
            (-2.5, -2.5, true),
            (0.0, -2.5, false),
            (2.5, -2.5, true),
            (-2.5, 0.0, false),
            (0.0, 0.0, true),
            (2.5, 0.0, false),
            (-2.5, 2.5, true),
            (0.0, 2.5, false),
            (2.5, 2.5, true),
        ];

        for (i, &(x, y, is_box)) in grid.iter().enumerate() {
            let colour = palette[i % palette.len()];
            let pos = glam::Vec3::new(x, y, 0.6);
            let (mesh, collider) = if is_box {
                (
                    self.box_mesh,
                    ColliderBuilder::cuboid(0.5, 0.5, 0.5).friction(0.7).build(),
                )
            } else {
                (
                    self.sphere_mesh,
                    ColliderBuilder::ball(0.5).friction(0.7).build(),
                )
            };

            let id = self.scene.add(
                Some(mesh),
                glam::Mat4::from_translation(pos),
                Material::from_colour(colour),
            );
            let body = RigidBodyBuilder::dynamic().translation(pos).build();
            self.rapier.add_body(id, body, collider);
            self.body_ids.push(id);
            self.body_colours.push(colour);
        }

        let (prepare, simulate) = self.rapier.clone().into_plugins();
        self.runtime = ViewportRuntime::new()
            .with_fixed_timestep(FixedTimestep::new(SIM_HZ))
            .with_plugin(prepare)
            .with_plugin(simulate);
    }

    fn probe(&mut self) {
        let eye = self.camera.eye_position();
        let dir = (self.camera.center - eye).normalize_or_zero();
        if dir == glam::Vec3::ZERO {
            self.aimed = None;
            return;
        }

        // Translate the floor NodeId to a rapier ColliderHandle *before*
        // entering `with_state`, since `with_state` holds the mutex for the
        // duration of the closure and calling another plugin method inside
        // would deadlock.
        let floor_collider = self.rapier.collider_handle(self.floor_id);

        // The escape-hatch raycast. Three things happening here:
        //   1. Build a QueryFilter that excludes the floor.
        //   2. Borrow a transient QueryPipeline from the broad phase.
        //   3. Cast, then translate the hit ColliderHandle back to a NodeId.
        self.aimed = self.rapier.with_state(|s| {
            let qp = s.broad_phase.as_query_pipeline(
                s.narrow_phase.query_dispatcher(),
                &s.bodies,
                &s.colliders,
                QueryFilter {
                    exclude_collider: floor_collider,
                    ..Default::default()
                },
            );
            let ray = Ray::new(eye, dir);
            qp.cast_ray_and_get_normal(&ray, RAY_MAX_DISTANCE, true)
                .and_then(|(handle, hit)| {
                    let node = s.node_for_collider(handle)?;
                    Some(HitInfo {
                        node,
                        distance: hit.time_of_impact,
                        point: eye + dir * hit.time_of_impact,
                        normal: hit.normal,
                    })
                })
        });
    }
}

// ---------------------------------------------------------------------------

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Read commit intent (click or space) up front, before the side panel
        // or viewport ui.input closures.
        let space_pressed = ctx.input(|i| i.key_pressed(egui::Key::Space));

        egui::SidePanel::left("controls")
            .exact_width(240.0)
            .show(ctx, |ui| {
                ui.add_space(8.0);
                ui.heading("Ray Probe");
                ui.add_space(4.0);
                ui.separator();
                ui.add_space(6.0);

                ui.label("A ray is cast from the camera every frame.");
                ui.add_space(4.0);
                ui.label("Click or space: blacken the aimed body.");
                ui.add_space(10.0);

                ui.label(egui::RichText::new("Aim").strong());
                if let Some(hit) = self.aimed {
                    egui::Grid::new("aim").num_columns(2).striped(true).show(ui, |ui| {
                        ui.label("Node");
                        ui.label(format!("{:?}", hit.node));
                        ui.end_row();
                        ui.label("Distance");
                        ui.label(format!("{:.3} m", hit.distance));
                        ui.end_row();
                        ui.label("Point");
                        ui.label(format!(
                            "({:+.2}, {:+.2}, {:+.2})",
                            hit.point.x, hit.point.y, hit.point.z
                        ));
                        ui.end_row();
                        ui.label("Normal");
                        ui.label(format!(
                            "({:+.2}, {:+.2}, {:+.2})",
                            hit.normal.x, hit.normal.y, hit.normal.z
                        ));
                        ui.end_row();
                    });
                } else {
                    ui.label("(no body in line of sight)");
                }

                ui.add_space(10.0);
                ui.label(format!("Blackened: {}", self.blackened.len()));

                ui.add_space(10.0);
                ui.separator();
                ui.add_space(6.0);
                if ui.button("Reset").clicked() {
                    self.reset();
                }
            });

        egui::CentralPanel::default().show(ctx, |ui| {
            let dt = ctx.input(|i| i.stable_dt).min(0.25);

            let (rect, response) =
                ui.allocate_exact_size(ui.available_size(), egui::Sense::click_and_drag());

            self.controller.begin_frame(ViewportContext {
                hovered: response.hovered(),
                focused: response.has_focus(),
                viewport_size: [rect.width(), rect.height()],
            });

            let mut click_commit = false;

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
                            // Drive the "commit" action from the *click* (egui's
                            // synthesised click event), not from press/release,
                            // so an orbit-drag never commits.
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

            if response.clicked() {
                click_commit = true;
            }

            self.controller.apply_to_camera(&mut self.camera);
            self.camera.set_aspect_ratio(rect.width(), rect.height());

            // Cast the camera-forward ray *after* the camera has been updated
            // for this frame, so the aim is consistent with what the user sees.
            self.probe();

            if click_commit || space_pressed {
                if let Some(hit) = self.aimed {
                    self.blackened.insert(hit.node);
                }
            }

            let mut frame_ctx = RuntimeFrameContext::default();
            frame_ctx.dt = dt;
            frame_ctx.camera = self.camera.clone();
            frame_ctx.viewport_size = glam::Vec2::new(rect.width(), rect.height());
            self.runtime
                .step(&mut self.scene, &mut self.selection, &frame_ctx);

            let alpha = self.runtime.alpha();
            let snapshots = self.runtime.snapshots();
            let aimed_node = self.aimed.map(|h| h.node);

            let mut items: Vec<SceneRenderItem> = self
                .body_ids
                .iter()
                .enumerate()
                .filter_map(|(i, &id)| {
                    let node = self.scene.node(id)?;
                    let world = node.world_transform();
                    let model = snapshots
                        .interpolated(id, alpha)
                        .map(glam::Mat4::from)
                        .unwrap_or(world);
                    let mesh_id = node.mesh_id()?;
                    let colour = if self.blackened.contains(&id) {
                        BLACKENED_COLOUR
                    } else if Some(id) == aimed_node {
                        HIGHLIGHT_COLOUR
                    } else {
                        self.body_colours[i]
                    };
                    let mut item = SceneRenderItem::default();
                    item.mesh_id = mesh_id;
                    item.model = model.to_cols_array_2d();
                    item.material = Material::from_colour(colour);
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

            // Small white marker at the hit point shows exactly where the ray
            // landed on the highlighted body.
            if let Some(hit) = self.aimed {
                let mut marker = SceneRenderItem::default();
                marker.mesh_id = self.marker_mesh;
                marker.model = glam::Mat4::from_translation(hit.point).to_cols_array_2d();
                marker.material = Material::from_colour([1.0, 1.0, 1.0]);
                items.push(marker);
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

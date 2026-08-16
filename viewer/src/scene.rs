//! What gets drawn.
//!
//! # The model is a placeholder
//!
//! A grey box with an amber block on it, standing in for the board until a
//! photo of the real one is mapped onto it. The amber block is not decoration
//! — a plain box has no visible front, top or handedness, so without it you
//! cannot tell whether the yaw axis is even connected.

use packet::glam;
use three_d::*;

use crate::cloud;

/// Half the size of the board model, in metres — roughly the proportions of
/// the real thing rather than a measurement of it. The exact numbers arrive
/// with the photo that eventually goes on top.
///
/// X is the short dimension and Y the long one, matching the firmware's body
/// frame: X right (across the width), Y forward (along the length). Getting
/// this backwards doesn't break anything at rest — it only shows up as pitch
/// and roll swapping on screen, which is a hard bug to spot without physically
/// tilting the board and comparing.
const BOARD_HALF_EXTENTS: glam::Vec3 = glam::Vec3::new(0.033, 0.048, 0.005);

const MARKER_HALF_EXTENTS: glam::Vec3 = glam::Vec3::new(0.010, 0.010, 0.003);

// The brand palette, from `bitwise-brand-colors.md`.
const INK: (f32, f32, f32) = (0.055, 0.071, 0.102);
const GRATICULE: Srgba = Srgba::new(0x1F, 0x28, 0x36, 255);
const MUTED: Srgba = Srgba::new(0x85, 0x93, 0xA6, 255);
const AMBER: Srgba = Srgba::new(0xF0, 0xA3, 0x2C, 255);
const CYAN: Srgba = Srgba::new(0x5C, 0xC6, 0xD8, 255);
const GREEN: Srgba = Srgba::new(0x4A, 0xDE, 0x80, 255);
const GREEN_DIM: Srgba = Srgba::new(0x86, 0xEF, 0xAC, 255);

/// Radius of the offset arrow's head, in the scaled units `cloud` produces.
/// Thicker than a sample dot on purpose — it is the one thing on that screen
/// the whole run is about.
const ARROW_THICKNESS: f32 = 0.022;

pub struct Scene {
    camera: Camera,
    body: Gm<Mesh, PhysicalMaterial>,
    marker: Gm<Mesh, PhysicalMaterial>,
    key: DirectionalLight,
    fill: AmbientLight,
}

impl Scene {
    pub fn new(context: &Context, viewport: Viewport) -> Self {
        // The world frame matches the firmware's: X east, Y north, Z up. So
        // the camera's "up" is +Z, and sitting it at negative Y puts the
        // viewer south of the board looking north at it — the same way you
        // would be holding it.
        let camera = Camera::new_perspective(
            viewport,
            vec3(0.0, -0.24, 0.13),
            vec3(0.0, 0.0, 0.0),
            vec3(0.0, 0.0, 1.0),
            degrees(45.0),
            0.01,
            10.0,
        );

        Self {
            camera,
            body: block(context, MUTED),
            marker: block(context, AMBER),
            key: DirectionalLight::new(context, 2.0, Srgba::WHITE, vec3(-0.4, 0.6, -1.0)),
            fill: AmbientLight::new(context, 0.4, Srgba::WHITE),
        }
    }

    /// Put the board where the packets say it is, and draw the frame.
    pub fn draw(&mut self, frame_input: &FrameInput, rotation: glam::Quat) {
        self.camera.set_viewport(frame_input.viewport);

        // The one line that makes Shot 22's claim true: this is `vector`, the
        // same crate the firmware links.
        let placement = vector::model_matrix(rotation, glam::Vec3::ZERO);

        self.body.set_transformation(to_render_matrix(
            placement * glam::Mat4::from_scale(BOARD_HALF_EXTENTS),
        ));

        // The marker that makes the model's orientation readable at a glance:
        // sitting on top, toward the front.
        let marker_offset = glam::Vec3::new(0.0, BOARD_HALF_EXTENTS.y * 0.5, BOARD_HALF_EXTENTS.z);
        self.marker.set_transformation(to_render_matrix(
            placement
                * glam::Mat4::from_translation(marker_offset)
                * glam::Mat4::from_scale(MARKER_HALF_EXTENTS),
        ));

        frame_input
            .screen()
            .clear(ClearState::color_and_depth(INK.0, INK.1, INK.2, 1.0, 1.0))
            .render(
                &self.camera,
                (&self.body).into_iter().chain(&self.marker),
                &[&self.key, &self.fill],
            );
    }
}

/// The calibration view: the samples, where they should be, and the distance
/// between those two things.
///
/// # Nothing here fits anything
///
/// The board solves on every packet and sends the answer, so the offset, the
/// radius and the residual are all read straight off the wire. A second fit in
/// the browser would be a fit of the same data by different code, free to
/// disagree with the one the board actually adopted — and a screen disagreeing
/// with the firmware about the number the firmware is using is about the worst
/// bug this page could have.
///
/// `radius == 0.0` is the board saying it has no usable fit yet, which is
/// ordinary for the first seconds of a run. The sphere and the offset arrow
/// simply do not appear until there is one.
///
/// # The camera drifts until you touch it
///
/// It circles on its own so the cloud reads as a ball with nobody's hand in
/// shot, and the first drag hands it over to [`OrbitControl`] for the rest of
/// the run. There is no control to click and no mode to be in: touching it is
/// the whole gesture, and starting a new run puts it back to drifting.
pub struct CalibrationScene {
    camera: Camera,
    orbit: OrbitControl,
    /// Set by the first drag. Until then the camera drifts.
    driven: bool,
    angle: f32,
    extent: cloud::Extent,
    /// 0 raw, 1 corrected. Linear — see [`cloud::advance`].
    snap: f32,
    snap_target: f32,
    /// How many samples are already in the instance buffer, so a frame that
    /// brought nothing new does not rebuild it.
    plotted: usize,
    dots: Gm<InstancedMesh, PhysicalMaterial>,
    axes: Gm<InstancedMesh, PhysicalMaterial>,
    arrow: Gm<Mesh, PhysicalMaterial>,
    ball: Gm<Mesh, PhysicalMaterial>,
    wedges: Vec<Gm<Mesh, PhysicalMaterial>>,
    key: DirectionalLight,
    fill: AmbientLight,
}

impl CalibrationScene {
    pub fn new(context: &Context, viewport: Viewport) -> Self {
        let camera = Camera::new_perspective(
            viewport,
            to_render_vec(cloud::camera_position(0.0)),
            vec3(0.0, 0.0, 0.0),
            vec3(0.0, 0.0, 1.0),
            degrees(45.0),
            0.01,
            100.0,
        );

        Self {
            camera,
            orbit: OrbitControl::new(vec3(0.0, 0.0, 0.0), 1.0, 12.0),
            driven: false,
            angle: 0.0,
            extent: cloud::Extent::default(),
            snap: 0.0,
            snap_target: 0.0,
            plotted: 0,
            dots: instanced(context, CpuMesh::sphere(8), AMBER, &Instances::default()),
            axes: instanced(context, CpuMesh::cylinder(12), CYAN, &origin_cross()),
            // `CpuMesh::arrow` gives its head a radius of 1 and its shaft the
            // radius passed in, so anything at or above 1 here is a rod with a
            // dent in the end rather than an arrow.
            arrow: solid(context, CpuMesh::arrow(0.82, 0.4, 16), GREEN),
            ball: translucent(context, CpuMesh::sphere(24), MUTED),
            wedges: (0..cloud::SECTORS)
                .map(|_| solid(context, CpuMesh::cube(), GRATICULE))
                .collect(),
            key: DirectionalLight::new(context, 2.0, Srgba::WHITE, vec3(-0.4, 0.6, -1.0)),
            fill: AmbientLight::new(context, 0.5, Srgba::WHITE),
        }
    }

    /// A new run has started. Everything the last one accumulated goes.
    pub fn begin(&mut self) {
        self.extent = cloud::Extent::default();
        self.snap = 0.0;
        self.snap_target = 0.0;
        self.plotted = 0;
        self.driven = false;
        self.angle = 0.0;
    }

    /// Show the corrected cloud, or the raw one.
    pub fn set_snap(&mut self, corrected: bool) {
        self.snap_target = if corrected { 1.0 } else { 0.0 };
    }

    /// Whether the view is currently showing, or heading toward, the corrected
    /// cloud. What the space bar toggles against.
    pub fn snapped(&self) -> bool {
        self.snap_target > 0.5
    }

    /// `offset` is the correction to apply when snapped — the adopted one after
    /// a solve, and `None` at every other moment, which pins the cloud where
    /// the magnetometer put it.
    pub fn draw(
        &mut self,
        frame_input: &mut FrameInput,
        samples: &[glam::Vec3],
        latest: Option<&packet::Calibration>,
        offset: Option<glam::Vec3>,
    ) {
        self.camera.set_viewport(frame_input.viewport);

        // Any drag or scroll takes the camera, permanently, until the next run.
        if self
            .orbit
            .handle_events(&mut self.camera, &mut frame_input.events)
        {
            self.driven = true;
        }
        if !self.driven {
            self.angle = cloud::drift(self.angle, frame_input.elapsed_time);
            self.camera.set_view(
                to_render_vec(cloud::camera_position(self.angle)),
                vec3(0.0, 0.0, 0.0),
                vec3(0.0, 0.0, 1.0),
            );
        }

        self.snap = cloud::advance(self.snap, self.snap_target, frame_input.elapsed_time);
        let snap = cloud::ease(self.snap);

        // Only ever grows, so this is a catch-up over whatever arrived since
        // the last frame rather than a rescan of the whole cloud.
        self.extent
            .widen_all(&samples[self.plotted.min(samples.len())..]);
        let scale = self.extent.scale();

        // Rebuilt whenever the cloud grew *or* the snap is moving, because the
        // snap moves every dot. Standing still with nothing new costs nothing.
        if samples.len() != self.plotted || self.snap != self.snap_target {
            let correction = offset.unwrap_or(glam::Vec3::ZERO);
            self.dots.set_instances(&Instances {
                transformations: samples
                    .iter()
                    .map(|sample| {
                        let at = cloud::place(*sample, correction, snap, scale);
                        to_render_matrix(
                            glam::Mat4::from_translation(at)
                                * glam::Mat4::from_scale(glam::Vec3::splat(cloud::DOT_RADIUS)),
                        )
                    })
                    .collect(),
                ..Default::default()
            });
            self.plotted = samples.len();
        }

        // Amber is the measured field; Green Dim is a signal derived from
        // another on-screen signal, which is exactly what a corrected sample
        // is. Crossing between them over the snap makes them the same dots.
        self.dots.material.albedo = mix(AMBER, GREEN_DIM, snap);

        // The sphere and the arrow follow the packet's own offset, not the
        // adopted one: mid-run that is the estimate settling onto the cloud,
        // and once a run has landed the two are the same number anyway, because
        // the offset that was adopted came out of the terminal packet.
        let fit = latest.filter(|packet| packet.radius > 0.0);
        let centre = fit.map(|packet| cloud::centre(packet.offset, snap, scale));

        if let (Some(packet), Some(centre)) = (fit, centre) {
            self.arrow.set_transformation(to_render_matrix(reaching(
                glam::Vec3::ZERO,
                centre,
                ARROW_THICKNESS,
            )));
            self.ball.set_transformation(to_render_matrix(
                glam::Mat4::from_translation(centre)
                    * glam::Mat4::from_scale(glam::Vec3::splat(packet.radius * scale)),
            ));

            for (bit, wedge) in self.wedges.iter_mut().enumerate() {
                let bit = bit as u32;
                let at = centre + cloud::sector_direction(bit) * packet.radius * scale;
                wedge.set_transformation(to_render_matrix(
                    glam::Mat4::from_translation(at)
                        * glam::Mat4::from_scale(glam::Vec3::new(0.03, 0.03, 0.008)),
                ));
                // The viewer's version of the ring's blink: the dark wedges are
                // the bearings still to be turned to.
                wedge.material.albedo = if cloud::visited(packet.sectors, bit) {
                    AMBER
                } else {
                    GRATICULE
                };
            }
        }

        let screen = frame_input.screen();
        screen.clear(ClearState::color_and_depth(INK.0, INK.1, INK.2, 1.0, 1.0));

        let mut objects: Vec<&dyn Object> = vec![&self.dots, &self.axes];
        if centre.is_some() {
            objects.push(&self.arrow);
            objects.push(&self.ball);
            objects.extend(self.wedges.iter().map(|wedge| wedge as &dyn Object));
        }

        screen.render(&self.camera, objects, &[&self.key, &self.fill]);
    }
}

/// The three arms of the origin marker, each a cylinder running the length of
/// its axis and centred on nothing at all — which is the point. This is where
/// the cloud is supposed to be.
fn origin_cross() -> Instances {
    const ARM: f32 = 0.16;
    const THICKNESS: f32 = 0.004;

    Instances {
        transformations: [glam::Vec3::X, glam::Vec3::Y, glam::Vec3::Z]
            .into_iter()
            .map(|axis| to_render_matrix(reaching(-axis * ARM, axis * ARM, THICKNESS)))
            .collect(),
        ..Default::default()
    }
}

/// A transform putting a `CpuMesh::cylinder` or `CpuMesh::arrow` — both of which
/// run from the origin along +X with unit radius — between two points.
fn reaching(from: glam::Vec3, to: glam::Vec3, thickness: f32) -> glam::Mat4 {
    let along = to - from;
    let length = along.length();
    if length < f32::EPSILON {
        // Degenerate rather than an error: a zero offset means the cloud is
        // already centred, and an arrow of no length is the honest picture of
        // that. `from_rotation_arc` would divide by zero here.
        return glam::Mat4::from_scale(glam::Vec3::ZERO);
    }

    glam::Mat4::from_translation(from)
        * glam::Mat4::from_quat(glam::Quat::from_rotation_arc(glam::Vec3::X, along / length))
        * glam::Mat4::from_scale(glam::Vec3::new(length, thickness, thickness))
}

/// Blend two palette colours. Used only for the snap, where the dots have to
/// stay visibly the same dots while they change what they mean.
fn mix(from: Srgba, to: Srgba, t: f32) -> Srgba {
    let lerp = |a: u8, b: u8| (a as f32 + (b as f32 - a as f32) * t.clamp(0.0, 1.0)) as u8;
    Srgba::new(
        lerp(from.r, to.r),
        lerp(from.g, to.g),
        lerp(from.b, to.b),
        255,
    )
}

fn solid(context: &Context, mesh: CpuMesh, albedo: Srgba) -> Gm<Mesh, PhysicalMaterial> {
    Gm::new(
        Mesh::new(context, &mesh),
        PhysicalMaterial::new_opaque(
            context,
            &CpuMaterial {
                albedo,
                ..Default::default()
            },
        ),
    )
}

/// The fitted sphere, drawn through the points rather than over them — solid,
/// it would hide the half of the cloud behind it, and the far side of the cloud
/// is where the gaps in a sweep are.
fn translucent(context: &Context, mesh: CpuMesh, albedo: Srgba) -> Gm<Mesh, PhysicalMaterial> {
    Gm::new(
        Mesh::new(context, &mesh),
        PhysicalMaterial::new_transparent(
            context,
            &CpuMaterial {
                albedo: Srgba::new(albedo.r, albedo.g, albedo.b, 48),
                ..Default::default()
            },
        ),
    )
}

fn instanced(
    context: &Context,
    mesh: CpuMesh,
    albedo: Srgba,
    instances: &Instances,
) -> Gm<InstancedMesh, PhysicalMaterial> {
    Gm::new(
        InstancedMesh::new(context, instances, &mesh),
        PhysicalMaterial::new_opaque(
            context,
            &CpuMaterial {
                albedo,
                ..Default::default()
            },
        ),
    )
}

/// glam's vector into three-d's.
fn to_render_vec(v: glam::Vec3) -> Vec3 {
    vec3(v.x, v.y, v.z)
}

/// `CpuMesh::cube()` is two units across, so a scale of the half extents gives
/// a box of exactly those half extents.
fn block(context: &Context, albedo: Srgba) -> Gm<Mesh, PhysicalMaterial> {
    Gm::new(
        Mesh::new(context, &CpuMesh::cube()),
        PhysicalMaterial::new_opaque(
            context,
            &CpuMaterial {
                albedo,
                ..Default::default()
            },
        ),
    )
}

/// glam's column-major array into three-d's matrix type.
fn to_render_matrix(m: glam::Mat4) -> Mat4 {
    let c = m.to_cols_array();
    Mat4::new(
        c[0], c[1], c[2], c[3], //
        c[4], c[5], c[6], c[7], //
        c[8], c[9], c[10], c[11], //
        c[12], c[13], c[14], c[15],
    )
}

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
const MUTED: Srgba = Srgba::new(0x85, 0x93, 0xA6, 255);
const AMBER: Srgba = Srgba::new(0xF0, 0xA3, 0x2C, 255);

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

/// The calibration view — **a stub, on purpose.**
///
/// 2.1 delivers the mode and the state that feeds it; what goes on this screen
/// is 2.2, and it is a different kind of work. Everything it needs is already
/// arriving: `Run::samples` is the point cloud, and the board's own packet
/// carries the fitted offset, radius, residual and spread, so nothing here has
/// to fit anything.
///
/// Until then it clears to the background, which is honest — the status line
/// says what is happening and the model is not sitting there pretending to
/// track a board that is being waved through the air.
pub fn draw_calibration(frame_input: &FrameInput, _samples: &[glam::Vec3]) {
    frame_input
        .screen()
        .clear(ClearState::color_and_depth(INK.0, INK.1, INK.2, 1.0, 1.0));
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

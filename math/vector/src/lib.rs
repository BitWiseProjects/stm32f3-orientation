//! Geometry and projection — model space in, normalized device coordinates out.
//!
//! This is the half of the maths that draws things, kept separate from
//! [`attitude`](../attitude/index.html) because it is useful without a sensor
//! anywhere in sight. The browser uses it to place a box on screen.
//!
//! # Where the output stops
//!
//! At **normalized device coordinates**: X and Y both in `-1..1`, with the
//! centre of the screen at the origin. That is the boundary, and it is
//! deliberate — this crate never learns how big anything is in pixels.
//! Mapping `-1..1` onto the canvas is the caller's job.
//!
//! Nothing here allocates, so a model lives on the stack and the whole
//! pipeline runs on a chip with no heap.
//!
//! # Winding through the pipeline
//!
//! ```text
//!   model space --[model matrix]--> world --[view]--> eye --[projection]--> clip
//!                                                                            |
//!                                       normalized device coords <--[divide]-+
//! ```
//!
//! [`Mesh`] holds the model, [`mvp`] composes the three matrices, and
//! [`project_edge`] runs one line all the way through — including throwing
//! away anything behind the eye, which is the step that quietly ruins a
//! renderer when it is missing.

#![cfg_attr(not(test), no_std)]

use glam::{Mat4, Quat, Vec2, Vec3, Vec4};

// Re-exported so callers do not have to depend on glam separately and risk
// ending up on a different version of it.
pub use glam;

/// Anything closer to the eye than this is behind it as far as we care.
///
/// Points at exactly zero depth project to infinity, so a line reaching the
/// eye has to be cut somewhere; cutting it just short is the cheap and stable
/// answer.
const NEAR_EPSILON: f32 = 1e-4;

/// A box drawn as twelve edges.
///
/// Eight corners and twelve lines is the entire model for this episode — the
/// board is a rectangle, and a rectangle is a very short list of numbers.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mesh {
    pub vertices: [Vec3; 8],
    pub edges: [[u8; 2]; 12],
}

impl Mesh {
    /// A box centred on the origin, `half_extents` in each direction.
    pub fn cuboid(half_extents: Vec3) -> Self {
        let Vec3 { x, y, z } = half_extents;

        Self {
            vertices: [
                Vec3::new(-x, -y, -z),
                Vec3::new(x, -y, -z),
                Vec3::new(x, y, -z),
                Vec3::new(-x, y, -z),
                Vec3::new(-x, -y, z),
                Vec3::new(x, -y, z),
                Vec3::new(x, y, z),
                Vec3::new(-x, y, z),
            ],
            edges: [
                // the face nearest -Z
                [0, 1],
                [1, 2],
                [2, 3],
                [3, 0],
                // the face nearest +Z
                [4, 5],
                [5, 6],
                [6, 7],
                [7, 4],
                // the four struts between them
                [0, 4],
                [1, 5],
                [2, 6],
                [3, 7],
            ],
        }
    }

    /// Iterate the edges as pairs of model-space points.
    pub fn edge_points(&self) -> impl Iterator<Item = (Vec3, Vec3)> + '_ {
        self.edges
            .iter()
            .map(move |[a, b]| (self.vertices[*a as usize], self.vertices[*b as usize]))
    }
}

/// Place a model in the world: rotate it, then move it.
///
/// The rotation is the orientation quaternion straight off the sensor fusion,
/// which is what makes the board on the desk and the box on the screen the
/// same object.
pub fn model_matrix(orientation: Quat, translation: Vec3) -> Mat4 {
    Mat4::from_rotation_translation(orientation, translation)
}

/// Where the eye is and what it is looking at.
pub fn view_matrix(eye: Vec3, target: Vec3, up: Vec3) -> Mat4 {
    glam::camera::rh::view::look_at_mat4(eye, target, up)
}

/// A perspective projection: things further away get smaller.
///
/// `fov_y` is the vertical field of view in radians, `aspect` is width over
/// height.
///
/// The OpenGL depth convention, because WebGL is what actually rasterizes this
/// on the browser end. It makes no difference to anything this crate returns —
/// the output is X and Y, and `w` is the same either way — but picking one
/// deliberately beats inheriting whichever a default happened to be.
pub fn projection_matrix(fov_y: f32, aspect: f32, near: f32, far: f32) -> Mat4 {
    glam::camera::rh::proj::opengl::perspective(fov_y, aspect, near, far)
}

/// Compose the three, in the order they apply.
pub fn mvp(model: Mat4, view: Mat4, projection: Mat4) -> Mat4 {
    projection * view * model
}

/// One model-space point, all the way to clip space.
pub fn to_clip(mvp: &Mat4, point: Vec3) -> Vec4 {
    *mvp * point.extend(1.0)
}

/// The perspective divide. Only meaningful once the point is in front of the eye.
pub fn to_ndc(clip: Vec4) -> Vec2 {
    Vec2::new(clip.x / clip.w, clip.y / clip.w)
}

/// Cut a clip-space line at the eye plane, or reject it entirely.
///
/// `w` is how far in front of the eye a point ended up, so a non-positive `w`
/// is a point at or behind the viewer. Dividing by it produces a coordinate
/// that is not merely wrong but wrong in an eye-catching way — the line whips
/// across the screen and back. Cutting the line where it crosses is the fix,
/// and the reason the function returns an `Option` at all.
pub fn clip_near(a: Vec4, b: Vec4) -> Option<(Vec4, Vec4)> {
    let (a_visible, b_visible) = (a.w > NEAR_EPSILON, b.w > NEAR_EPSILON);

    match (a_visible, b_visible) {
        (true, true) => Some((a, b)),
        (false, false) => None,
        // Exactly one end is in front, so the line crosses the plane once.
        // Walk from the visible end toward the other until `w` hits the limit.
        (true, false) => Some((a, a + (b - a) * crossing(a.w, b.w))),
        (false, true) => Some((b + (a - b) * crossing(b.w, a.w), b)),
    }
}

/// How far along a segment `w` falls to `NEAR_EPSILON`, given the two ends.
fn crossing(from: f32, to: f32) -> f32 {
    (from - NEAR_EPSILON) / (from - to)
}

/// One edge, from model space to a pair of screen points.
///
/// `None` means the edge is entirely behind the eye and should not be drawn.
pub fn project_edge(mvp: &Mat4, a: Vec3, b: Vec3) -> Option<(Vec2, Vec2)> {
    let (a, b) = clip_near(to_clip(mvp, a), to_clip(mvp, b))?;
    Some((to_ndc(a), to_ndc(b)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::PI;

    /// An eye 3 units back along +Z, looking at the origin, 60 degrees vertical.
    fn looking_at_origin() -> Mat4 {
        mvp(
            Mat4::IDENTITY,
            view_matrix(Vec3::new(0.0, 0.0, 3.0), Vec3::ZERO, Vec3::Y),
            projection_matrix(PI / 3.0, 1.0, 0.1, 100.0),
        )
    }

    #[test]
    fn a_cuboid_has_twelve_edges_and_no_repeats() {
        let mesh = Mesh::cuboid(Vec3::splat(1.0));
        assert_eq!(mesh.edge_points().count(), 12);

        for [a, b] in mesh.edges {
            assert_ne!(a, b, "an edge joined a vertex to itself");
        }
    }

    #[test]
    fn every_edge_joins_neighbouring_corners() {
        // Neighbours on a box differ in exactly one coordinate. This catches a
        // typo in the edge table that would otherwise draw a diagonal through
        // the middle of the box and look almost right.
        let mesh = Mesh::cuboid(Vec3::new(2.0, 3.0, 4.0));

        for (a, b) in mesh.edge_points() {
            let differing = [a.x != b.x, a.y != b.y, a.z != b.z]
                .iter()
                .filter(|d| **d)
                .count();
            assert_eq!(differing, 1, "{a:?} and {b:?} are not neighbours");
        }
    }

    #[test]
    fn the_centre_of_the_model_lands_in_the_centre_of_the_screen() {
        let ndc = to_ndc(to_clip(&looking_at_origin(), Vec3::ZERO));
        assert!(ndc.length() < 1e-5, "expected the origin, got {ndc:?}");
    }

    #[test]
    fn further_away_is_smaller() {
        let mvp = looking_at_origin();
        let near = to_ndc(to_clip(&mvp, Vec3::new(0.5, 0.0, 0.0)));
        let far = to_ndc(to_clip(&mvp, Vec3::new(0.5, 0.0, -5.0)));

        assert!(
            far.x.abs() < near.x.abs(),
            "the far point should sit closer to the centre: near {near:?} far {far:?}"
        );
    }

    #[test]
    fn rotating_the_model_moves_it_on_screen() {
        let view = view_matrix(Vec3::new(0.0, 0.0, 3.0), Vec3::ZERO, Vec3::Y);
        let projection = projection_matrix(PI / 3.0, 1.0, 0.1, 100.0);
        let corner = Vec3::new(1.0, 0.0, 0.0);

        let upright = to_ndc(to_clip(&mvp(Mat4::IDENTITY, view, projection), corner));
        let turned = to_ndc(to_clip(
            &mvp(
                model_matrix(Quat::from_rotation_z(PI / 2.0), Vec3::ZERO),
                view,
                projection,
            ),
            corner,
        ));

        // A quarter turn about the view axis takes a point on +X to +Y.
        assert!(upright.x > 0.1 && upright.y.abs() < 1e-5, "{upright:?}");
        assert!(turned.y > 0.1 && turned.x.abs() < 1e-5, "{turned:?}");
    }

    #[test]
    fn an_edge_entirely_behind_the_eye_is_dropped() {
        let mvp = looking_at_origin();
        let behind = project_edge(&mvp, Vec3::new(0.0, 0.0, 20.0), Vec3::new(1.0, 0.0, 20.0));
        assert!(behind.is_none());
    }

    #[test]
    fn an_edge_crossing_the_eye_plane_is_cut_not_dropped() {
        let mvp = looking_at_origin();

        // From well in front of the eye to well behind it.
        let crossing = project_edge(&mvp, Vec3::new(0.3, 0.0, 0.0), Vec3::new(0.3, 0.0, 20.0));
        let (near, cut) = crossing.expect("the visible part should have survived");

        assert!(near.is_finite() && cut.is_finite(), "{near:?} {cut:?}");
        // The cut end is at the eye plane, where the perspective divide blows
        // any offset up enormously — which is exactly what it is there to
        // stop happening halfway across the screen.
        assert!(cut.x.abs() > near.x.abs());
    }

    #[test]
    fn clipping_keeps_the_visible_end_untouched() {
        let visible = Vec4::new(1.0, 2.0, 3.0, 4.0);
        let hidden = Vec4::new(1.0, 2.0, 3.0, -1.0);

        let (a, _) = clip_near(visible, hidden).unwrap();
        assert_eq!(a, visible);

        let (_, b) = clip_near(hidden, visible).unwrap();
        assert_eq!(b, visible);
    }
}

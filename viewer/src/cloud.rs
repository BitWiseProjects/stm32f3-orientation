//! The arithmetic behind the calibration view.
//!
//! Everything on that screen is a point in nanotesla put somewhere on screen,
//! and this module is the "somewhere". It holds no `three_d` types and touches
//! no GPU, which is the whole reason it is a separate file: `scene.rs` cannot
//! be tested without a browser and a WebGL context, and this can be tested with
//! `cargo test`.
//!
//! # One scale, and it only ever zooms out
//!
//! Samples arrive in nanotesla, so they are tens of thousands of units long,
//! and the picture has to fit in a frame a couple of units across. The obvious
//! move is to scale by the fitted radius — and it is the wrong one, because
//! early in a run the fit is either absent or nonsense, and a scale that jumps
//! about makes the cloud pulse like a heartbeat while you are trying to wave
//! the board.
//!
//! So the scale comes from the samples themselves: the furthest sample **from
//! the origin**, not from the cloud's centre. Distance from the origin is the
//! right measure because the origin is on screen too — it is half the picture,
//! the point the cloud is supposed to be centred on and visibly is not.
//!
//! And it is a running maximum, so the view only ever widens. A cloud that
//! zoomed back in every time you turned the board away from its furthest
//! bearing would be unwatchable.

use packet::glam::Vec3;

/// How long the snap takes, in milliseconds.
///
/// Slow enough to read as a movement rather than a cut, short enough that
/// nobody waits for it. The point is that you see the *same* cloud arrive at
/// the origin — a cut would leave open whether the second picture is even the
/// same data.
pub const SNAP_MS: f64 = 600.0;

/// How fast the camera drifts when nobody has touched it, in radians per
/// second. A full turn takes about forty seconds.
pub const DRIFT_RATE: f32 = 0.15;

/// How high the camera sits, as an angle above the horizontal plane.
///
/// Not zero: a camera level with the equator sees the sample ring edge-on and
/// the sphere reads as a disc. About twenty degrees is enough to show it is a
/// ball without looking down at it from above.
pub const ELEVATION: f32 = 0.35;

/// Camera distance from the origin, in the scaled units this module produces —
/// where the furthest sample sits at 1.0.
pub const CAMERA_DISTANCE: f32 = 3.0;

/// How big a sample dot is drawn, in the same units.
pub const DOT_RADIUS: f32 = 0.012;

/// The smallest extent the view will scale to, in nanotesla.
///
/// Without a floor, the first few samples of a run — or a single sample at the
/// origin — divide by something near zero. This is far below any real field, so
/// it never binds on live data; it exists so the arithmetic cannot explode
/// before the second packet arrives.
pub const MIN_EXTENT_NT: f32 = 1000.0;

/// How many bearing sectors [`packet::Calibration::sectors`] reports.
///
/// Eight, and they are **horizontal** — this is a ring around the equator, not
/// a shading of the whole sphere. Coverage of the sphere is what
/// [`packet::Calibration::spread`] measures; these eight bits only know which
/// way the board has been pointed. Drawing them as a ring is honest about that;
/// dimming patches all over the sphere would claim knowledge that is not in the
/// packet.
pub const SECTORS: u32 = 8;

/// The running scale of the view: how far the furthest sample has ever been
/// from the origin, in nanotesla.
///
/// Feed every packet through [`widen`] and divide by the result to get scaled
/// units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Extent(f32);

impl Default for Extent {
    fn default() -> Self {
        Self(MIN_EXTENT_NT)
    }
}

impl Extent {
    /// Grow to take in a new sample. Never shrinks.
    pub fn widen(&mut self, sample: Vec3) {
        let reach = sample.length();
        if reach.is_finite() && reach > self.0 {
            self.0 = reach;
        }
    }

    /// Take in a whole cloud at once — used when the scene is rebuilt from a
    /// run that is already under way.
    pub fn widen_all(&mut self, samples: &[Vec3]) {
        for sample in samples {
            self.widen(*sample);
        }
    }

    /// Scaled units per nanotesla.
    pub fn scale(&self) -> f32 {
        1.0 / self.0
    }
}

/// Where a raw sample belongs on screen.
///
/// `snap` runs 0 to 1: at 0 the sample sits where the magnetometer said it was,
/// at 1 the offset has been subtracted and the cloud is centred on the origin.
/// In between is the animation, and the fact that it is one subtraction
/// scaled by a number between 0 and 1 is the point being made — the correction
/// really is just this.
pub fn place(sample: Vec3, offset: Vec3, snap: f32, scale: f32) -> Vec3 {
    (sample - offset * snap) * scale
}

/// Where the middle of the cloud is, under the same snap.
///
/// The tip of the offset vector, and the centre of the fitted sphere, so those
/// two travel with the cloud instead of being left behind by it.
pub fn centre(offset: Vec3, snap: f32, scale: f32) -> Vec3 {
    offset * (1.0 - snap) * scale
}

/// Move `current` toward `target` at the [`SNAP_MS`] rate.
///
/// Linear, then shaped by [`ease`] when it is used. Keeping the stored value
/// linear is what lets the snap be reversed halfway through without a jump —
/// easing on the way in and out of a stored eased value compounds.
pub fn advance(current: f32, target: f32, elapsed_ms: f64) -> f32 {
    let step = (elapsed_ms / SNAP_MS) as f32;
    if target > current {
        (current + step).min(target)
    } else {
        (current - step).max(target)
    }
}

/// Smoothstep, so the cloud leaves and arrives without a jolt.
pub fn ease(t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// The direction of one of the eight bearing sectors.
///
/// Bit 0 is centred on +X and they count anticlockwise, which is `Fit::sectors`
/// in `math/magcal`. It has to match, or the lit wedges point at bearings the
/// board was never turned to. The wedges being *centred* on their bearing
/// rather than starting at it is the same rule that lets the firmware's eight
/// LEDs sit one per sector.
pub fn sector_direction(bit: u32) -> Vec3 {
    let angle = core::f32::consts::TAU * (bit % SECTORS) as f32 / SECTORS as f32;
    Vec3::new(angle.cos(), angle.sin(), 0.0)
}

/// Whether that sector has been visited.
pub fn visited(mask: u8, bit: u32) -> bool {
    mask & (1 << (bit % SECTORS)) != 0
}

/// Where the camera sits for a given drift angle.
///
/// Circling the origin rather than the cloud: the origin is the fixed thing in
/// this picture and the cloud moves onto it during the snap. Orbiting the cloud
/// would drag the whole frame sideways as it went.
pub fn camera_position(angle: f32) -> Vec3 {
    Vec3::new(
        CAMERA_DISTANCE * ELEVATION.cos() * angle.cos(),
        CAMERA_DISTANCE * ELEVATION.cos() * angle.sin(),
        CAMERA_DISTANCE * ELEVATION.sin(),
    )
}

/// How far the camera has drifted after `elapsed_ms`, wrapped to one turn.
pub fn drift(angle: f32, elapsed_ms: f64) -> f32 {
    (angle + DRIFT_RATE * (elapsed_ms / 1000.0) as f32).rem_euclid(core::f32::consts::TAU)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A believable hard-iron case: a 47,000 nT field with the centre pushed
    /// about 30,000 nT off the origin, which is roughly what this board does.
    const OFFSET: Vec3 = Vec3::new(9265.5, -28_189.5, -11_732.9);
    const RADIUS: f32 = 46_917.9;

    fn a_sphere_about(centre: Vec3, radius: f32) -> Vec<Vec3> {
        let mut out = Vec::new();
        for ring in 0..9 {
            let tilt = core::f32::consts::PI * (ring as f32 + 0.5) / 9.0;
            for step in 0..16 {
                let bearing = core::f32::consts::TAU * step as f32 / 16.0;
                out.push(
                    centre
                        + radius
                            * Vec3::new(
                                tilt.sin() * bearing.cos(),
                                tilt.sin() * bearing.sin(),
                                tilt.cos(),
                            ),
                );
            }
        }
        out
    }

    #[test]
    fn the_view_only_ever_widens() {
        let mut extent = Extent::default();
        extent.widen(Vec3::new(0.0, 0.0, 40_000.0));
        let after_far = extent.scale();

        // Every sample of a real sweep that comes back closer to the origin.
        for _ in 0..50 {
            extent.widen(Vec3::new(0.0, 0.0, 100.0));
        }

        assert_eq!(extent.scale(), after_far);
    }

    #[test]
    fn an_empty_run_still_has_a_usable_scale() {
        // The first frame of a run happens before any sample has arrived, and
        // it divides by this.
        let extent = Extent::default();
        assert!(extent.scale().is_finite());
        assert!(extent.scale() > 0.0);
    }

    #[test]
    fn a_sample_at_the_origin_does_not_collapse_the_scale() {
        let mut extent = Extent::default();
        extent.widen(Vec3::ZERO);
        assert_eq!(extent.scale(), 1.0 / MIN_EXTENT_NT);
    }

    #[test]
    fn a_broken_sample_cannot_poison_the_scale() {
        // A corrupt packet that passed its checksum, or a fit that divided by
        // zero somewhere upstream. The scale is a running maximum, so a single
        // infinity would be permanent.
        let mut extent = Extent::default();
        extent.widen(Vec3::new(30_000.0, 0.0, 0.0));
        extent.widen(Vec3::new(f32::INFINITY, 0.0, 0.0));
        extent.widen(Vec3::new(f32::NAN, 0.0, 0.0));
        assert_eq!(extent.scale(), 1.0 / 30_000.0);
    }

    #[test]
    fn the_whole_cloud_fits_inside_the_frame() {
        // The claim the scale exists to make: with the offset this large, every
        // sample and the origin both land within one unit of the middle.
        let samples = a_sphere_about(OFFSET, RADIUS);
        let mut extent = Extent::default();
        extent.widen_all(&samples);
        let scale = extent.scale();

        for sample in &samples {
            assert!(place(*sample, OFFSET, 0.0, scale).length() <= 1.0 + 1e-4);
        }
    }

    #[test]
    fn the_snap_moves_the_cloud_onto_the_origin() {
        let samples = a_sphere_about(OFFSET, RADIUS);
        let mut extent = Extent::default();
        extent.widen_all(&samples);
        let scale = extent.scale();

        // Before: the cloud's middle is a long way off the origin — that
        // distance is the entire diagnosis.
        assert!(centre(OFFSET, 0.0, scale).length() > 0.4);

        // After: every corrected sample is the same distance out, and that
        // distance is the field strength. A sphere centred on nothing.
        for sample in &samples {
            let corrected = place(*sample, OFFSET, 1.0, scale);
            assert!((corrected.length() - RADIUS * scale).abs() < 1e-3);
        }
        assert_eq!(centre(OFFSET, 1.0, scale), Vec3::ZERO);
    }

    #[test]
    fn the_snap_takes_the_time_it_says_it_does() {
        let mut snap = 0.0;
        let mut elapsed = 0.0;
        // Sixty frames a second, which is what the render loop runs at.
        while snap < 1.0 && elapsed < 10_000.0 {
            snap = advance(snap, 1.0, 1000.0 / 60.0);
            elapsed += 1000.0 / 60.0;
        }
        assert_eq!(snap, 1.0);
        assert!((elapsed - SNAP_MS).abs() < 40.0, "took {elapsed} ms");
    }

    #[test]
    fn the_snap_reverses_from_wherever_it_had_got_to() {
        // Pressing space twice in quick succession. The stored value is linear
        // precisely so that this cannot jump.
        let part_way = advance(0.0, 1.0, SNAP_MS / 3.0);
        assert!(part_way > 0.0 && part_way < 1.0);

        let back = advance(part_way, 0.0, SNAP_MS / 6.0);
        assert!(back > 0.0 && back < part_way);
        assert_eq!(advance(back, 0.0, SNAP_MS), 0.0);
    }

    #[test]
    fn the_snap_never_overshoots_on_a_long_frame() {
        // A tab that was backgrounded, or a garbage collection pause. Left
        // unclamped this puts the cloud somewhere past the origin.
        assert_eq!(advance(0.0, 1.0, 60_000.0), 1.0);
        assert_eq!(advance(1.0, 0.0, 60_000.0), 0.0);
    }

    #[test]
    fn easing_leaves_both_ends_alone() {
        assert_eq!(ease(0.0), 0.0);
        assert_eq!(ease(1.0), 1.0);
        assert_eq!(ease(-5.0), 0.0);
        assert_eq!(ease(5.0), 1.0);
        assert!((ease(0.5) - 0.5).abs() < 1e-6);
    }

    #[test]
    fn the_eight_wedges_ring_the_equator_anticlockwise_from_x() {
        // Bit 0 on +X, counting anticlockwise, matching `magcal::Fit::sectors`.
        // If this drifts, the lit wedges point at bearings the board was never
        // turned to, which is worse than showing nothing.
        assert!((sector_direction(0) - Vec3::X).length() < 1e-6);
        assert!((sector_direction(2) - Vec3::Y).length() < 1e-6);
        assert!((sector_direction(4) + Vec3::X).length() < 1e-6);
        assert!((sector_direction(6) + Vec3::Y).length() < 1e-6);

        for bit in 0..SECTORS {
            let direction = sector_direction(bit);
            assert!((direction.length() - 1.0).abs() < 1e-6);
            assert_eq!(direction.z, 0.0, "the wedges are a ring, not a sphere");
        }
    }

    #[test]
    fn a_sector_mask_reads_the_way_the_firmware_wrote_it() {
        // The mask a run leaves when the board was turned everywhere except one
        // bearing — the gap is the thing the ring is on screen to show.
        let mask = 0b1111_0111u8;
        assert!(!visited(mask, 3));
        for bit in [0, 1, 2, 4, 5, 6, 7] {
            assert!(visited(mask, bit), "bit {bit}");
        }
        assert!(!visited(0x00, 0));
        assert!(visited(0xFF, 7));
    }

    #[test]
    fn the_camera_keeps_its_distance_and_its_height_all_the_way_round() {
        let mut angle = 0.0;
        for _ in 0..600 {
            let position = camera_position(angle);
            assert!((position.length() - CAMERA_DISTANCE).abs() < 1e-3);
            assert!(position.z > 0.0, "the camera never goes under the floor");
            angle = drift(angle, 1000.0 / 60.0);
        }
    }

    #[test]
    fn the_drift_wraps_instead_of_growing_without_bound() {
        // A view left open all afternoon. An angle that only ever grows loses
        // its precision to the exponent long before anyone notices.
        let mut angle = 0.0;
        for _ in 0..60 * 60 * 60 {
            angle = drift(angle, 1000.0 / 60.0);
        }
        assert!((0.0..core::f32::consts::TAU).contains(&angle), "{angle}");
    }
}

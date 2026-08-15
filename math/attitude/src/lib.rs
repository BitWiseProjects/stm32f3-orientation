//! Turning three imperfect sensors into one orientation.
//!
//! Nothing in here talks to hardware, allocates, or knows what time it is.
//! Rates and fields go in, an orientation quaternion comes out, and `dt` is a
//! parameter rather than something this crate goes and looks up. That is what
//! lets the same source compile for the chip and for the browser — and what
//! lets the tests at the bottom of this file run on your laptop.
//!
//! # The world frame
//!
//! East-North-Up, right-handed: **X east, Y north, Z up.** The orientation
//! quaternion maps board-frame vectors into that frame, so
//! `orientation * v_board == v_world`.
//!
//! # What the filter is doing
//!
//! The gyro measures *rate* and knows nothing about where it is, so its angle
//! has to be built up by adding rates over time — and every error in every
//! sample is kept forever. That is drift, and it is not a bug; it is what
//! integration does.
//!
//! The other two sensors *measure*, which means they can be wrong but never
//! get progressively wronger:
//!
//! - the accelerometer, held still, feels only gravity, so it always knows
//!   which way is down;
//! - the magnetometer always knows which way is north.
//!
//! So each one is turned into a small correction — how far the estimate's own
//! idea of "up" (or "north") has slid away from what the sensor actually
//! reports — and that correction is added to the gyro rate before integrating.
//! The gyro handles anything fast; the other two slowly pull it back toward
//! reality. Turn the gains up and the estimate follows the noisy sensors; turn
//! them down and it follows the drifting one.
//!
//! The correction is a cross product of two unit vectors, so it points along
//! the axis the estimate would have to turn about to agree with the sensor,
//! and its length is the sine of how far off it is.
//!
//! # What this does not do, and what that costs
//!
//! The correction is proportional to the current error and nothing else —
//! there is no running estimate of the gyro's bias. That is the simplest thing
//! that works, and it is what the episode explains, but it has a consequence
//! worth stating plainly:
//!
//! **A constant gyro bias leaves a constant angle error.** The estimate settles
//! where the correction exactly cancels the bias, which for a bias `b` and gain
//! `k` is an error of about `asin(b / k)` — a few degrees for the numbers this
//! board produces. It stops there and stays there.
//!
//! That is a completely different failure from drift. Drift has no fixed point
//! at all and walks away forever; this settles. Getting rid of the remaining
//! offset means estimating the bias as well, which is a bigger idea than this
//! episode takes on.

#![cfg_attr(not(test), no_std)]

use glam::Quat;

// Re-exported so callers do not have to depend on glam separately and risk
// ending up on a different version of it — which would make the `Quat` this
// crate returns a different type from the `Quat` they were expecting.
pub use glam;
pub use glam::{Quat as Orientation, Vec3};

/// Which way is up, in the world frame.
pub const WORLD_UP: Vec3 = Vec3::Z;

/// Which way is north, in the world frame.
pub const WORLD_NORTH: Vec3 = Vec3::Y;

/// How hard each measuring sensor pulls the estimate back.
///
/// Units are effectively radians per second of correction per unit of error,
/// so these are comparable to the gyro rates they are added to. Bigger numbers
/// mean faster correction and more of the sensor's noise; smaller numbers mean
/// a smoother result that takes longer to recover.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Gains {
    /// Pull toward the accelerometer's idea of down. This one corrects tilt.
    pub accel: f32,
    /// Pull toward the magnetometer's idea of north. This one corrects heading.
    pub mag: f32,
}

impl Default for Gains {
    fn default() -> Self {
        Self {
            accel: 1.0,
            mag: 0.5,
        }
    }
}

/// The running orientation estimate.
#[derive(Clone, Copy, Debug)]
pub struct Attitude {
    orientation: Quat,
    gains: Gains,
}

impl Default for Attitude {
    fn default() -> Self {
        Self::new()
    }
}

impl Attitude {
    /// A fresh estimate, sitting level and facing north.
    pub fn new() -> Self {
        Self {
            orientation: Quat::IDENTITY,
            gains: Gains::default(),
        }
    }

    pub fn with_gains(gains: Gains) -> Self {
        Self {
            orientation: Quat::IDENTITY,
            gains,
        }
    }

    /// The current estimate. `orientation * v_board == v_world`.
    pub fn orientation(&self) -> Quat {
        self.orientation
    }

    pub fn gains(&self) -> Gains {
        self.gains
    }

    pub fn set_gains(&mut self, gains: Gains) {
        self.gains = gains;
    }

    /// Gyro only — no correction from anything.
    ///
    /// This is the honest version of "just add up the rates", and it drifts.
    /// Watching it drift is the whole middle of the episode, so it is a real
    /// entry point rather than a test convenience.
    pub fn integrate(&mut self, gyro_rad_s: Vec3, dt: f32) {
        self.update(gyro_rad_s, None, None, dt);
    }

    /// One step of the filter.
    ///
    /// `gyro_rad_s` is body-frame angular rate in radians per second.
    /// `accel` and `mag` are body-frame vectors in any units at all — only
    /// their directions are used, so raw sensor counts are as good as SI.
    /// Pass `None` for either to skip that correction.
    pub fn update(&mut self, gyro_rad_s: Vec3, accel: Option<Vec3>, mag: Option<Vec3>, dt: f32) {
        let mut rate = gyro_rad_s;

        // The estimate's own idea of up and of north, rotated into the board's
        // frame. That is the form the sensors report in, so it is the only
        // form the two can be compared in.
        let into_body = self.orientation.inverse();
        let estimated_up = into_body * WORLD_UP;

        if let Some(measured_up) = accel.and_then(direction_of) {
            rate += self.gains.accel * measured_up.cross(estimated_up);
        }

        if let Some(measured_field) = mag.and_then(direction_of) {
            // The earth's field is not "north" — at most latitudes it dips
            // steeply into the ground, so comparing it against WORLD_NORTH
            // directly would report a huge permanent error and drag the tilt
            // estimate down with it.
            //
            // So: rotate the measurement into the world frame, then rebuild a
            // reference that has the same dip but is squarely north. Whatever
            // is left over between them is heading error and nothing else.
            let in_world = self.orientation * measured_field;
            let horizontal = Vec3::new(in_world.x, in_world.y, 0.0).length();
            let reference = Vec3::new(0.0, horizontal, in_world.z);

            let estimated_field = into_body * reference;
            let error = measured_field.cross(estimated_field);

            // Keep only the part of that error that turns the board about its
            // own vertical axis, and throw the rest away.
            //
            // Two vectors separated by a pure heading error do not, in
            // general, have a cross product that points straight up — so
            // without this the magnetometer would also be quietly voting on
            // tilt, and then spend the whole time arguing with the
            // accelerometer about it. Gravity owns tilt; north owns heading.
            rate += self.gains.mag * estimated_up * error.dot(estimated_up);
        }

        // Compose the small rotation this step's rate produces onto the
        // estimate. Renormalizing each step keeps rounding error from slowly
        // inflating the quaternion into something that scales as well as
        // rotates.
        self.orientation = (self.orientation * Quat::from_scaled_axis(rate * dt)).normalize();
    }

    /// Back to level and facing north.
    pub fn reset(&mut self) {
        self.orientation = Quat::IDENTITY;
    }
}

/// Unit vector, or `None` if there is no usable direction in it.
///
/// A zero-length or NaN reading is a sensor that did not answer, and feeding
/// it into a cross product would poison the estimate rather than fail loudly.
fn direction_of(v: Vec3) -> Option<Vec3> {
    let length = v.length();
    if length.is_finite() && length > 1e-9 {
        Some(v / length)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::PI;

    /// Rotate `v` by the estimate and see where it ends up. Handy for asking
    /// "which way does the estimate think the board's own X axis points?"
    fn to_world(a: &Attitude, v: Vec3) -> Vec3 {
        a.orientation() * v
    }

    /// A field that dips steeply into the ground, as it does at most
    /// latitudes — this is what a magnetometer actually reports, and using a
    /// flat one in tests would hide the whole reason `update` reprojects it.
    fn earth_field(yaw: f32) -> Vec3 {
        let level = Vec3::new(0.0, 0.5, -0.85).normalize();
        // The board is yawed, so the field appears rotated the other way in
        // the board's own frame.
        Quat::from_rotation_z(-yaw) * level
    }

    fn gravity(tilt_about_x: f32) -> Vec3 {
        Quat::from_rotation_x(-tilt_about_x) * WORLD_UP
    }

    #[test]
    fn sitting_still_stays_put() {
        let mut a = Attitude::new();
        for _ in 0..1000 {
            a.integrate(Vec3::ZERO, 0.005);
        }
        assert!(a.orientation().angle_between(Quat::IDENTITY) < 1e-4);
    }

    #[test]
    fn integrating_a_rate_gives_an_angle() {
        // A quarter turn per second, for one second, about Z.
        let mut a = Attitude::new();
        let rate = Vec3::new(0.0, 0.0, PI / 2.0);
        for _ in 0..1000 {
            a.integrate(rate, 0.001);
        }

        let expected = Quat::from_rotation_z(PI / 2.0);
        assert!(
            a.orientation().angle_between(expected) < 1e-3,
            "got {:?}",
            a.orientation()
        );
    }

    #[test]
    fn a_tiny_bias_drifts_without_bound() {
        // Half a degree per second of bias — "nothing", by any standard — and
        // no correction. This is Act 5's whole premise, as an assertion.
        let bias = Vec3::new(0.0, 0.0, 0.5f32.to_radians());

        let mut a = Attitude::new();
        for _ in 0..(200 * 60) {
            a.integrate(bias, 1.0 / 200.0);
        }

        let drifted = a.orientation().angle_between(Quat::IDENTITY).to_degrees();
        assert!(
            (drifted - 30.0).abs() < 0.5,
            "a minute of half a degree per second should be ~30 degrees, got {drifted}"
        );
    }

    #[test]
    fn gravity_pulls_tilt_back() {
        // Start the estimate believing it is tilted 30 degrees when it is not.
        let mut a = Attitude::new();
        a.orientation = Quat::from_rotation_x(30f32.to_radians());

        let level = WORLD_UP;
        for _ in 0..(200 * 20) {
            a.update(Vec3::ZERO, Some(level), None, 1.0 / 200.0);
        }

        let believed_up = to_world(&a, level);
        let error = believed_up.angle_between(WORLD_UP).to_degrees();
        assert!(error < 1.0, "tilt should have been corrected, off by {error}");
    }

    #[test]
    fn gravity_cannot_fix_heading() {
        // The key claim of the episode, made checkable: spin the board flat and
        // the accelerometer reads exactly the same thing at every heading, so
        // it cannot possibly know the heading is wrong.
        let mut a = Attitude::new();
        a.orientation = Quat::from_rotation_z(40f32.to_radians());

        for _ in 0..(200 * 30) {
            a.update(Vec3::ZERO, Some(WORLD_UP), None, 1.0 / 200.0);
        }

        let heading = to_world(&a, Vec3::Y).truncate().to_angle();
        let north = Vec3::Y.truncate().to_angle();
        let error = (heading - north).to_degrees().abs();
        assert!(
            error > 39.0,
            "gravity must not have corrected heading, but the error fell to {error}"
        );
    }

    #[test]
    fn the_magnetometer_fixes_heading() {
        // Same starting error as above. The only change is that north is now
        // an input, which is Act 5's second fix.
        let mut a = Attitude::new();
        a.orientation = Quat::from_rotation_z(40f32.to_radians());

        let field = earth_field(0.0);
        for _ in 0..(200 * 60) {
            a.update(Vec3::ZERO, Some(WORLD_UP), Some(field), 1.0 / 200.0);
        }

        let heading = to_world(&a, Vec3::Y).truncate().to_angle();
        let error = heading.to_degrees() - 90.0;
        assert!(
            error.abs() < 1.0,
            "heading should have come back to north, off by {error}"
        );
    }

    #[test]
    fn the_magnetometer_leaves_tilt_alone() {
        // The heading correction is deliberately restricted to the vertical
        // axis. Feeding a steeply dipping field into a level board must not
        // tip the estimate over, or the two corrections would spend the whole
        // time undoing each other.
        let mut a = Attitude::new();
        a.orientation = Quat::from_rotation_z(40f32.to_radians());

        let field = earth_field(0.0);
        for _ in 0..(200 * 60) {
            a.update(Vec3::ZERO, None, Some(field), 1.0 / 200.0);
        }

        let tilt = to_world(&a, WORLD_UP).angle_between(WORLD_UP).to_degrees();
        assert!(tilt < 0.1, "north should not have tipped the board, tilt {tilt}");
    }

    #[test]
    fn fusion_holds_against_a_drifting_gyro() {
        // The same bias that walks 30 degrees away in a minute on its own,
        // now with both measuring sensors switched on.
        //
        // The claim being checked is *boundedness*, not zero error. A
        // proportional-only filter settles at a small offset rather than at
        // nothing — see the crate docs. What matters is that it settles: the
        // second minute must not be any worse than the first, which is exactly
        // what integration alone can never manage.
        let bias = Vec3::new(0.3f32.to_radians(), 0.0, 0.5f32.to_radians());
        let field = earth_field(0.0);

        let mut a = Attitude::new();
        let run_for = |a: &mut Attitude, seconds: u32| {
            for _ in 0..(200 * seconds) {
                a.update(bias, Some(WORLD_UP), Some(field), 1.0 / 200.0);
            }
            a.orientation().angle_between(Quat::IDENTITY).to_degrees()
        };

        let after_one_minute = run_for(&mut a, 60);
        let after_two = run_for(&mut a, 60);

        assert!(
            after_one_minute < 8.0,
            "should have settled to a small offset, got {after_one_minute}"
        );
        assert!(
            (after_two - after_one_minute).abs() < 0.5,
            "should have stopped moving: {after_one_minute} then {after_two}"
        );
    }

    #[test]
    fn a_dead_sensor_is_ignored_rather_than_believed() {
        let mut a = Attitude::new();
        a.update(Vec3::ZERO, Some(Vec3::ZERO), Some(Vec3::NAN), 0.005);
        assert!(a.orientation().is_finite());
        assert!(a.orientation().angle_between(Quat::IDENTITY) < 1e-6);
    }

    #[test]
    fn tilt_and_heading_are_corrected_together() {
        let mut a = Attitude::new();
        a.orientation = Quat::from_rotation_z(25f32.to_radians())
            * Quat::from_rotation_x(20f32.to_radians());

        for _ in 0..(200 * 40) {
            a.update(Vec3::ZERO, Some(gravity(0.0)), Some(earth_field(0.0)), 1.0 / 200.0);
        }

        let error = a.orientation().angle_between(Quat::IDENTITY).to_degrees();
        assert!(error < 1.5, "should have converged to level and north, off by {error}");
    }
}

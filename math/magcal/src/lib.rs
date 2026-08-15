//! Finding out how much your compass is lying, and by how much in which
//! direction.
//!
//! A magnetometer sitting in a clean field reports a vector of constant length
//! whichever way you turn it. Only the direction changes. So every reading you
//! could ever take lies on **a sphere centred on the origin**, and that is not
//! a modelling choice — it is what "measuring a fixed field" means.
//!
//! Put a magnet, a speaker, a screw or a magnetised solder joint on the board,
//! and it adds its own field to every reading. That field turns *with* the
//! board, so it is a constant in the board's own frame: the same vector added
//! to every sample. The sphere is still a sphere, and it is still the same
//! size — **it has just moved off the origin.** That displacement is the whole
//! problem, it is called hard iron, and this crate exists to measure it.
//!
//! A field fixed in the *room* cannot do this. Turn the board in place and any
//! static room field sweeps around a circle centred on the origin, changing the
//! radius but never the centre. Only a source that physically turns with the
//! board can displace it — which is why the answer travels with the board and
//! can be compiled in.
//!
//! # It looks nonlinear and it is not
//!
//! The obvious way to write the problem down is
//!
//! ```text
//! |m - c|² = r²
//! ```
//!
//! which has the unknown centre `c` inside a square, and looks like it needs
//! an iterative solver. Expand it and it falls apart:
//!
//! ```text
//! |m|² - 2 m·c + |c|² = r²
//! 2·mx·cx + 2·my·cy + 2·mz·cz + k = |m|²        where  k = r² - |c|²
//! ```
//!
//! Four unknowns — `cx`, `cy`, `cz`, `k` — and every one of them appears
//! linearly. One sample is one row. The radius comes back at the end from
//! `r = sqrt(k + |c|²)`, so nothing was lost by hiding it inside `k`.
//!
//! That is ordinary linear least squares, which means it is a 4×4 system of
//! normal equations, which means **the samples can be thrown away as they
//! arrive.** [`Fit`] holds fourteen running sums and a counter. It does not
//! matter whether you feed it two hundred samples or two million; it is the
//! same fourteen numbers, and it never allocates.
//!
//! # Why you have to wave it around
//!
//! Spin the board flat on a desk and every sample has the same `mz`. That
//! column of the system is then a constant — and so is the column belonging to
//! `k`. Two identical columns make the matrix singular, and a singular matrix
//! has no unique answer: there are infinitely many centres that fit a circle
//! equally well, because a circle genuinely does not tell you where the centre
//! of a sphere is along the axis you never left.
//!
//! **This is the reason your phone asks you to wave it in a figure eight.** It
//! is not a ritual and it is not about collecting more data. Flat samples,
//! however many of them, cannot answer the question.
//!
//! [`Fit::spread`] is that fact as a number between 0 and 1 — 0 for samples
//! lying in a plane, 1 for samples spread evenly over a ball — and both
//! [`Fit::is_ready`] and [`Fit::solve`] are gated on it. A flat spin never
//! finishes, because it cannot.
//!
//! # What this does not correct
//!
//! Hard iron only: one offset, subtracted. **Soft iron** — a nearby ferrous
//! mass that distorts the field rather than adding to it — turns the sphere
//! into an ellipsoid, and fixing it needs a 3×3 matrix rather than a vector.
//! On the board this was written for, the locus is a sphere to about 3% of its
//! radius, so there is nothing there to correct and a nine-parameter fit would
//! only be nine ways to fit the noise.
//!
//! Nor does it help with a field that *changes*. Calibration measures a
//! constant, and a magnet someone waves past the board is not one.
//!
//! # Units
//!
//! Whatever you like. The first sample sets an internal scale and everything
//! after it is measured against that, so nanotesla, microtesla, gauss and raw
//! counts all give the same answer and all condition equally well. Results come
//! back in the units you fed in.
//!
//! The accumulators are `f64` regardless. On a Cortex-M4F that is software
//! emulation, and at fourteen multiply-accumulates per sample at 50 Hz it costs
//! well under a thousandth of the chip — against `f32`, where a sum of fourth
//! powers of field strength runs out of significant figures long before a
//! calibration run is over.

#![cfg_attr(not(test), no_std)]

use glam::Vec3;

// Re-exported so callers do not have to depend on glam separately and risk
// ending up on a different version of it — the same reasoning as `attitude`.
pub use glam;

/// Fewer samples than this and [`Fit::solve`] refuses outright.
pub const MIN_SAMPLES: u32 = 200;

/// Below this, [`Fit::spread`] reports zero rather than a number.
const MEANINGFUL_SAMPLES: u32 = 64;

/// Below this [`Fit::spread`], the samples are a plane and there is no unique
/// answer. [`Fit::solve`] refuses rather than returning a fitted-looking
/// number, because a confident wrong offset is worse than no offset.
///
/// **This number was set by real data, not by taste.** Four flat-spin captures
/// off the bench score between 0.002 and 0.025, hand-wobble and all — and the
/// one at 0.025 solves to a centre 50,000 nT below the board and an
/// offset-to-radius ratio of 2.08, against 0.91 from a two-dimensional fit of
/// the same log. That is the unconstrained axis running away, and it looks
/// exactly as confident as a good answer. The floor sits above all four.
pub const MIN_SPREAD: f32 = 0.05;

/// What [`Fit::is_ready`] wants before it says a run is finished on its own.
///
/// Deliberately well above [`MIN_SPREAD`]: the floor is what makes an answer
/// *possible*, this is what makes it *good*. The gap between them is what a
/// manual stop is for.
pub const GOOD_SAMPLES: u32 = 500;

/// The spread [`Fit::is_ready`] wants.
///
/// **A bench number, and it has moved twice.** 0.15 first, which a real
/// thirty-second wave only just reached; then 0.10, because a hand tilting a
/// board sweeps a *cap* rather than a ball and one measured run covered
/// elevations from only -77 deg to -15 deg.
///
/// **0.10 turned out to be far too generous, and three runs on one board show
/// why.** A run that tilted the board without ever turning it about its
/// vertical axis stopped at spread 0.099 — just over the line — and reported a
/// residual of 0.90% of the field, the *best* of the three. It was the worst
/// calibration of the three. Scored against a properly covered set of samples
/// its offset left 4.14% scatter, against 3.36% for the run that swept
/// properly; and scored against its own narrow samples, every candidate offset
/// tried — including the wrong ones — came out between 0.88% and 1.54%. Narrow
/// data cannot tell good offsets from bad ones, and its low residual says only
/// that the test was easy.
///
/// What separates them is coverage, which is what this number measures. A run
/// doing both motions — tilting *and* turning — reaches 0.44 inside thirty
/// seconds, so 0.30 sits well above what a one-motion run achieves and well
/// below what a real one does.
///
/// It relies on the caller giving the operator time to do both motions; see
/// the minimum run length in `03_calibrate.rs` and `07_fused.rs`.
pub const GOOD_SPREAD: f32 = 0.30;

/// The worst residual-to-radius ratio [`Fit::is_ready`] will call finished.
///
/// **This exists because [`spread`](Fit::spread) cannot tell a swept sphere
/// from a board that never moved.** Spread measures whether the samples are
/// isotropic, and sensor noise about a stationary point is a *perfect* little
/// isotropic ball — it scores near 1.0, better than any real hand sweep.
///
/// Caught on hardware: a run that began before the board was picked up scored
/// 0.98 spread, cleared the sample count, and announced a solved field of 790
/// nT. The earth manages about 50,000. Nothing in the sample distribution said
/// anything was wrong, and the fit was internally consistent — it really was
/// the best sphere through those points.
///
/// What gives it away is the scatter. That run sat at 42% of its own radius;
/// a real one on the same board is 2.5%. There is no tuning to agonise over
/// between those two numbers, which is why the threshold is a round 10%.
pub const MAX_RESIDUAL_FRACTION: f32 = 0.10;

/// A hard-iron correction: the offset to take off every reading.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Calibration {
    offset: Vec3,
}

impl Calibration {
    /// No correction at all — what an uncalibrated board is using, whether or
    /// not it says so.
    pub const NONE: Self = Self { offset: Vec3::ZERO };

    /// A correction measured somewhere else, e.g. compiled in from a previous
    /// run.
    pub const fn from_offset(offset: Vec3) -> Self {
        Self { offset }
    }

    /// Where the centre of the sphere sits, in the board's own frame.
    pub const fn offset(&self) -> Vec3 {
        self.offset
    }

    /// The field the earth is actually producing, with the board's own
    /// contribution taken back out.
    pub fn apply(&self, raw: Vec3) -> Vec3 {
        raw - self.offset
    }
}

/// Why a fit could not be turned into an answer.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FitError {
    /// Not enough samples yet. See [`MIN_SAMPLES`].
    TooFewSamples,
    /// The samples lie in a plane, or close enough to one. This is the flat
    /// spin, and it is the interesting failure — see the crate docs.
    Coplanar,
    /// The system would not eliminate. Coplanar samples are caught before this,
    /// so reaching it means something stranger: samples all in one spot, or a
    /// run so short that rounding dominates.
    Singular,
    /// A sphere was found and the samples are nowhere near it — the residual is
    /// more than [`MAX_RESIDUAL_FRACTION`] of the radius.
    ///
    /// **This is the failure [`Fit::spread`] cannot see, and it is not the flat
    /// spin.** Spread asks whether the cloud is equally wide in every direction,
    /// which sensor noise satisfies perfectly. A board left nearly still
    /// produces an isotropic ball of noise, scores about 0.89, sails past
    /// [`MIN_SPREAD`] — and then fits a sphere of a few hundred nT, which is the
    /// noise floor rather than the earth.
    ///
    /// Measured on hardware: 935 nT "field", 40% residual, an offset 82 times
    /// the radius. Every other check passed.
    ///
    /// The instruction it deserves is *"you did not move it enough"*, which is a
    /// different thing to tell someone than [`Coplanar`](Self::Coplanar)'s *"you
    /// kept it flat"*.
    Scattered,
}

/// What a finished calibration run found out.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Solution {
    /// The answer — hand this to the sensor read path.
    pub calibration: Calibration,
    /// How strong the earth's field is here, in the units you fed in. Worth
    /// looking at: it should be somewhere near 50,000 nT at the surface, and
    /// wildly away from that means the fit found something that is not the
    /// earth.
    pub radius: f32,
    /// Root-mean-square distance from a sample to the fitted sphere, in the
    /// same units. **Read it against [`radius`](Self::radius)** — a few percent
    /// is a clean hard-iron-only fit, and much more than that is soft iron, a
    /// moving field, or a wobbly hand.
    pub residual: f32,
    /// How many samples went into it.
    pub samples: u32,
    /// The [`Fit::spread`] at the moment it was solved.
    pub spread: f32,
}

impl Solution {
    /// Whether the sphere actually fits the samples it was fitted to.
    ///
    /// Every other check asks about the *inputs* — how many, how spread out.
    /// This is the only one that looks at the answer and asks whether it
    /// describes them, and on a run that was not moved enough it is the only
    /// one that says no.
    pub fn is_believable(&self) -> bool {
        self.residual <= MAX_RESIDUAL_FRACTION * self.radius
    }
}

/// A calibration run in progress.
///
/// Feed it samples with [`push`](Self::push), watch [`is_ready`](Self::is_ready),
/// then [`solve`](Self::solve). Constant memory, no allocation, and no sample
/// is ever stored.
#[derive(Clone, Copy, Debug, Default)]
pub struct Fit {
    /// Set from the first sample's magnitude, so the sums stay near unity
    /// whatever units the caller thinks in.
    scale: f64,
    n: u32,
    /// Σm
    sum: [f64; 3],
    /// Σ m⊗m, upper triangle: xx, xy, xz, yy, yz, zz
    moment: [f64; 6],
    /// Σ |m|² m
    weighted: [f64; 3],
    /// Σ |m|²
    square: f64,
    /// Σ |m|⁴ — only needed for the residual, which is why it is here at all.
    quartic: f64,
    /// Smallest and largest of each axis so far.
    ///
    /// Nothing in the fit uses these. Their midpoint is a rough centre, and a
    /// rough centre is all [`Fit::sectors`] needs to say which way the board
    /// is pointing while the run is still going.
    ///
    /// It is worth knowing that this midpoint **is** the whole of the min/max
    /// calibration method that a lot of projects stop at. Here it is accurate
    /// enough to drive eight LEDs and nowhere near accurate enough to be the
    /// answer, which is a fair summary of the method.
    low: [f64; 3],
    high: [f64; 3],
    /// Which 45° sectors of the horizontal plane have been visited, bit 0 being
    /// the one starting at +X and counting anticlockwise.
    sectors: u8,
}

impl Fit {
    pub fn new() -> Self {
        Self::default()
    }

    /// Throw the run away and start again.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// Take one reading.
    ///
    /// Non-finite and zero readings are dropped rather than believed. Neither
    /// is a measurement: the earth's field is never zero anywhere on the
    /// planet, so a zero is a sensor that did not answer, and letting one
    /// through would drag the centre toward the origin — which is exactly the
    /// quantity being measured.
    pub fn push(&mut self, sample: Vec3) {
        if !sample.is_finite() {
            return;
        }
        // `is_finite` above already ruled out NaN, so a plain comparison is
        // safe here in a way it is not elsewhere in this file.
        let length_squared = sample.length_squared();
        if length_squared <= 0.0 {
            return;
        }

        if self.n == 0 {
            self.scale = 1.0 / f64::from(libm::sqrtf(length_squared));
        }

        let x = f64::from(sample.x) * self.scale;
        let y = f64::from(sample.y) * self.scale;
        let z = f64::from(sample.z) * self.scale;
        let square = x * x + y * y + z * z;

        if self.n == 0 {
            self.low = [x, y, z];
            self.high = [x, y, z];
        } else {
            for (axis, value) in [x, y, z].into_iter().enumerate() {
                if value < self.low[axis] {
                    self.low[axis] = value;
                }
                if value > self.high[axis] {
                    self.high[axis] = value;
                }
            }
        }

        self.n += 1;
        self.sum[0] += x;
        self.sum[1] += y;
        self.sum[2] += z;
        self.moment[0] += x * x;
        self.moment[1] += x * y;
        self.moment[2] += x * z;
        self.moment[3] += y * y;
        self.moment[4] += y * z;
        self.moment[5] += z * z;
        self.weighted[0] += square * x;
        self.weighted[1] += square * y;
        self.weighted[2] += square * z;
        self.square += square;
        self.quartic += square * square;

        // Bearing is taken about the bounding box's midpoint, not about the
        // origin — the origin is exactly the point we do not trust yet, and on
        // a board whose offset is nearly as big as the field, bearings about
        // it would all crowd into one sector.
        //
        // Not the running mean either: the mean of a swept arc sits inside the
        // arc and lags a long way behind, which shifts sectors and leaves gaps
        // that never fill. The box midpoint settles far sooner.
        let dx = x - (self.low[0] + self.high[0]) / 2.0;
        let dy = y - (self.low[1] + self.high[1]) / 2.0;
        if dx != 0.0 || dy != 0.0 {
            // `floor`, not a cast — a cast truncates toward zero, which folds
            // the two sectors either side of +X into one and leaves a gap on
            // the far side that no amount of turning ever fills.
            //
            // The half-step is what centres sector 0 on +X instead of starting
            // it there. Eight lights sit *at* the eight bearings, not between
            // them, so bins that straddle them are the ones a ring can show.
            let turns = libm::atan2(dy, dx) * (4.0 / core::f64::consts::PI);
            let sector = (libm::floor(turns + 0.5) as i32).rem_euclid(8) as u8;
            self.sectors |= 1 << sector;
        }
    }

    pub fn samples(&self) -> u32 {
        self.n
    }

    /// Which 45° bearing sectors have been visited, as a bitmask — bit 0 is the
    /// sector centred on +X, counting anticlockwise, so bit `i` is the bearing
    /// `45·i` give or take 22.5°.
    ///
    /// Centred rather than starting there so that the bits line up one-to-one
    /// with eight lights spaced 45° apart.
    ///
    /// Eight bits, and the board happens to have eight LEDs. That is what this
    /// is for: showing the person waving the board which way they have not
    /// pointed it yet.
    ///
    /// **It is a display aid, not the gate.** Filling all eight is easy and
    /// proves almost nothing on its own — a flat spin fills them and still
    /// cannot be solved. [`spread`](Self::spread) is the gate.
    ///
    /// One quirk worth knowing, because it looks like a bug: bearings are
    /// measured about a centre that is itself still being estimated, so the
    /// first part of the very first turn gets filed under the wrong sector and
    /// leaves a gap. Waving the board for another second fills it. Nothing
    /// re-files an old sample, because no old sample is kept.
    pub fn sectors(&self) -> u8 {
        self.sectors
    }

    /// How three-dimensionally the samples are spread, from 0 to 1.
    ///
    /// 0 means they lie in a plane and the fit has no unique answer. 1 means
    /// they are spread evenly over a ball. A single flat spin scores 0 however
    /// long you do it for; a decent figure eight scores a few tenths.
    ///
    /// It is the determinant of the trace-normalized scatter matrix, times 27
    /// so that a ball comes out at exactly 1. That determinant goes to zero
    /// precisely when the samples become coplanar, which is precisely when the
    /// 4×4 system becomes singular — so this is not a proxy for the
    /// conditioning, it is the same fact with an easier name.
    pub fn spread(&self) -> f32 {
        // Below a few dozen samples this statistic is mostly noise, and noise
        // is isotropic — a handful of readings scores near 1.0 and looks like a
        // finished calibration. Nothing is gated on it that early, but anything
        // *displaying* it would lie, so it reads zero until it means something.
        if self.n < MEANINGFUL_SAMPLES {
            return 0.0;
        }
        let n = f64::from(self.n);
        let (mx, my, mz) = (self.sum[0] / n, self.sum[1] / n, self.sum[2] / n);

        // Scatter about the mean: Σm⊗m − n·mean⊗mean.
        let xx = self.moment[0] - n * mx * mx;
        let xy = self.moment[1] - n * mx * my;
        let xz = self.moment[2] - n * mx * mz;
        let yy = self.moment[3] - n * my * my;
        let yz = self.moment[4] - n * my * mz;
        let zz = self.moment[5] - n * mz * mz;

        let trace = xx + yy + zz;
        if !is_positive(trace) {
            return 0.0;
        }

        let det = xx * (yy * zz - yz * yz) - xy * (xy * zz - yz * xz) + xz * (xy * yz - yy * xz);
        let normalized = 27.0 * det / (trace * trace * trace);
        if normalized > 0.0 { normalized as f32 } else { 0.0 }
    }

    /// Whether the run has enough, and enough *kinds* of, samples to stop on
    /// its own.
    ///
    /// Stricter than what [`solve`](Self::solve) will accept, deliberately.
    /// This is "good"; solve's floor is "possible".
    ///
    /// Bearing coverage is deliberately **not** part of this. On real hardware
    /// all eight sectors fill within about twenty-five samples — half a second —
    /// so requiring them gates on nothing while looking like a safeguard. And
    /// it is not needed: a wave confined to one bearing is a wave in a plane,
    /// and [`spread`](Self::spread) already refuses those.
    ///
    /// **It does solve, every time it is asked.** Counting samples and
    /// measuring their spread are both properties of where the samples sit, and
    /// neither can tell a swept sphere from a stationary board buzzing with
    /// noise — see [`MAX_RESIDUAL_FRACTION`], which is the check that can. So
    /// "ready" means the answer has been worked out and is believable, not
    /// merely that the inputs look plausible. The cost is one 4×4 elimination
    /// per call, which is microseconds.
    pub fn is_ready(&self) -> bool {
        if self.n < GOOD_SAMPLES || self.spread() < GOOD_SPREAD {
            return false;
        }

        self.solve_for_use().is_ok()
    }

    /// Solve for an offset that is about to be **adopted**.
    ///
    /// [`solve`](Self::solve) answers "what sphere best fits these samples",
    /// which is the right question while a run is going: the estimate is shown
    /// settling onto the cloud, and an early one being poor is expected rather
    /// than wrong. This answers "is that sphere fit to be used", which is a
    /// different question and the only one that matters at the moment the
    /// answer replaces the correction the board is running on.
    ///
    /// **Nothing else asks it.** Sample count and spread are both properties of
    /// where the samples sit, and no property of the inputs can distinguish a
    /// swept sphere from a stationary board buzzing with noise. Only comparing
    /// the answer against the samples can.
    ///
    /// Missing this was a real bug, not a hypothetical one. Both stages ended a
    /// run with `if ready || press`, so stopping by hand routed straight past
    /// [`is_ready`](Self::is_ready) — the only thing performing this check — and
    /// the board adopted a 935 nT "earth field" with a 40% residual and told
    /// nobody. Everything downstream then looks like a fusion bug.
    pub fn solve_for_use(&self) -> Result<Solution, FitError> {
        let solution = self.solve()?;
        if solution.is_believable() {
            Ok(solution)
        } else {
            Err(FitError::Scattered)
        }
    }

    /// Solve for the centre.
    ///
    /// Refuses rather than guessing. A wrong offset installed confidently is
    /// worse than no offset at all, because everything downstream then looks
    /// like a fusion bug.
    pub fn solve(&self) -> Result<Solution, FitError> {
        if self.n < MIN_SAMPLES {
            return Err(FitError::TooFewSamples);
        }
        let spread = self.spread();
        if spread < MIN_SPREAD {
            return Err(FitError::Coplanar);
        }

        let n = f64::from(self.n);
        let [sx, sy, sz] = self.sum;
        let [mxx, mxy, mxz, myy, myz, mzz] = self.moment;
        let [wx, wy, wz] = self.weighted;

        // One row per sample of `[2mx, 2my, 2mz, 1]·x = |m|²`, squared up into
        // normal equations — which is exactly what the running sums are.
        let a = [
            [4.0 * mxx, 4.0 * mxy, 4.0 * mxz, 2.0 * sx],
            [4.0 * mxy, 4.0 * myy, 4.0 * myz, 2.0 * sy],
            [4.0 * mxz, 4.0 * myz, 4.0 * mzz, 2.0 * sz],
            [2.0 * sx, 2.0 * sy, 2.0 * sz, n],
        ];
        let b = [2.0 * wx, 2.0 * wy, 2.0 * wz, self.square];

        let solution = solve4(a, b).ok_or(FitError::Singular)?;
        let (cx, cy, cz, k) = (solution[0], solution[1], solution[2], solution[3]);

        let radius_squared = k + cx * cx + cy * cy + cz * cz;
        if !is_positive(radius_squared) {
            return Err(FitError::Singular);
        }
        let radius = libm::sqrt(radius_squared);

        // At the least-squares solution the normal equations give `xᵀAx = xᵀb`,
        // so the sum of squared residuals collapses to `Σ|m|⁴ − xᵀb` and needs
        // no second pass over samples we no longer have.
        //
        // Those residuals are algebraic — `r² − |m − c|²` rather than a
        // distance. For a point near the sphere that is about `2r` times the
        // distance, so dividing by `2r` puts the answer back in the caller's
        // units.
        let projection = solution
            .iter()
            .zip(b.iter())
            .map(|(x, b)| x * b)
            .sum::<f64>();
        let sum_squares = if self.quartic > projection {
            self.quartic - projection
        } else {
            0.0
        };
        let residual = libm::sqrt(sum_squares / n) / (2.0 * radius);

        // Back out of the internal scale, into whatever the caller measures in.
        let unscale = 1.0 / self.scale;
        Ok(Solution {
            calibration: Calibration::from_offset(Vec3::new(
                (cx * unscale) as f32,
                (cy * unscale) as f32,
                (cz * unscale) as f32,
            )),
            radius: (radius * unscale) as f32,
            residual: (residual * unscale) as f32,
            samples: self.n,
            spread,
        })
    }
}

/// Greater than zero, and not NaN.
///
/// Written out rather than left as `!(v > 0.0)` because the negation is doing
/// real work — a NaN that reached one of these guards would sail straight
/// through `v <= 0.0` and be treated as a perfectly good number.
fn is_positive(v: f64) -> bool {
    v > 0.0
}

/// Gaussian elimination with partial pivoting. `None` if it will not eliminate.
///
/// Four unknowns is small enough that a general solver would be more code than
/// this, and this one can say *why* it failed.
fn solve4(mut a: [[f64; 4]; 4], mut b: [f64; 4]) -> Option<[f64; 4]> {
    // Pivots are judged against the biggest number in the original matrix, not
    // against zero. "Small" only means anything relative to something.
    let mut largest = 0.0f64;
    for row in &a {
        for value in row {
            let magnitude = libm::fabs(*value);
            if magnitude > largest {
                largest = magnitude;
            }
        }
    }
    if !is_positive(largest) {
        return None;
    }
    let floor = largest * 1e-12;

    for column in 0..4 {
        let mut pivot = column;
        for row in (column + 1)..4 {
            if libm::fabs(a[row][column]) > libm::fabs(a[pivot][column]) {
                pivot = row;
            }
        }
        if libm::fabs(a[pivot][column]) < floor {
            return None;
        }
        a.swap(column, pivot);
        b.swap(column, pivot);

        for row in (column + 1)..4 {
            let factor = a[row][column] / a[column][column];
            if factor == 0.0 {
                continue;
            }
            // Indexing rather than iterating: this reads one row of `a` while
            // writing another, which an iterator over `a` cannot express.
            #[allow(clippy::needless_range_loop)]
            for k in column..4 {
                a[row][k] -= factor * a[column][k];
            }
            b[row] -= factor * b[column];
        }
    }

    let mut x = [0.0f64; 4];
    for row in (0..4).rev() {
        let mut value = b[row];
        for column in (row + 1)..4 {
            value -= a[row][column] * x[column];
        }
        x[row] = value / a[row][row];
    }

    if x.iter().all(|v| v.is_finite()) { Some(x) } else { None }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::PI;

    /// Points on a sphere of radius `radius` about `centre`, covering
    /// elevations from `-tilt` to `+tilt`. `tilt = 0` is a flat spin; `tilt =
    /// PI/2` is the whole ball.
    ///
    /// Rings are spaced evenly in the *sine* of elevation rather than in
    /// elevation, which is what makes the points cover the surface evenly.
    /// Spacing them in elevation instead crowds the poles, and a lopsided
    /// shell is not the ball that `spread` is defined against.
    fn shell(centre: Vec3, radius: f32, tilt: f32, rings: u32, per_ring: u32) -> Vec<Vec3> {
        let mut out = Vec::new();
        let extent = tilt.sin();
        for ring in 0..rings {
            let elevation = if rings == 1 {
                0.0
            } else {
                (-extent + 2.0 * extent * (ring as f32) / ((rings - 1) as f32)).asin()
            };
            for step in 0..per_ring {
                let azimuth = 2.0 * PI * (step as f32) / (per_ring as f32);
                out.push(
                    centre
                        + radius
                            * Vec3::new(
                                elevation.cos() * azimuth.cos(),
                                elevation.cos() * azimuth.sin(),
                                elevation.sin(),
                            ),
                );
            }
        }
        out
    }

    fn fit_of(samples: &[Vec3]) -> Fit {
        let mut fit = Fit::new();
        for s in samples {
            fit.push(*s);
        }
        fit
    }

    #[test]
    fn a_clean_sphere_gives_back_its_centre() {
        // The board's real numbers, near enough: an offset almost as big as
        // the field itself, which is the situation that makes a compass
        // useless rather than merely inaccurate.
        let centre = Vec3::new(20_701.0, -14_915.0, -3_000.0);
        let fit = fit_of(&shell(centre, 26_252.0, 1.0, 9, 32));

        let solution = fit.solve().expect("a well-covered sphere must solve");
        let error = (solution.calibration.offset() - centre).length();
        assert!(error < 1.0, "centre off by {error} nT");
        assert!((solution.radius - 26_252.0).abs() < 1.0, "radius {}", solution.radius);
        assert!(solution.residual < 1.0, "residual {}", solution.residual);
    }

    #[test]
    fn a_flat_spin_is_refused_however_long_you_do_it() {
        // This is the claim the episode makes about why phones ask for a
        // figure eight, as an assertion. Ten thousand samples, every bearing
        // covered, and it still cannot be solved — because a circle does not
        // know where the centre of a sphere is.
        let fit = fit_of(&shell(Vec3::new(20_701.0, -14_915.0, 0.0), 26_252.0, 0.0, 1, 10_000));

        assert_eq!(fit.samples(), 10_000);
        assert!(
            fit.sectors().count_ones() >= 6,
            "a flat spin does cover the bearings, {:#010b}",
            fit.sectors()
        );
        assert!(!fit.is_ready(), "and it must still never call itself finished");
        assert_eq!(fit.solve(), Err(FitError::Coplanar));
    }

    /// A board that barely moved: a filled ball of sensor noise about wherever
    /// it happens to be sitting, rather than a shell swept through the field.
    ///
    /// Filled, not a shell, and that is the whole point — a *shell* of noise
    /// would be a perfect little sphere and fit beautifully. Real noise has
    /// samples at every radius out to the noise floor, so no sphere passes
    /// through them and the residual is a large fraction of whatever radius
    /// gets chosen.
    fn a_ball_of_noise(centre: Vec3, floor: f32) -> Vec<Vec3> {
        let mut out = Vec::new();
        for layer in 1..=6 {
            out.extend(shell(centre, floor * layer as f32 / 6.0, PI / 2.0, 9, 16));
        }
        out
    }

    #[test]
    fn a_board_that_barely_moved_is_refused_although_it_looks_perfectly_isotropic() {
        // Measured on hardware, 2026-08-15: a run stopped by hand after being
        // slid around a desk fitted a 935 nT "earth field" with a 40% residual
        // and an offset 82 times the radius — and the board adopted it.
        //
        // Every check that looks at *where the samples sit* passed, because
        // sensor noise is isotropic and that is exactly what those checks
        // reward. This is `spread`'s documented blind spot, as an assertion.
        let sitting = Vec3::new(21_000.0, -14_915.0, -33_100.0);
        let fit = fit_of(&a_ball_of_noise(sitting, 900.0));

        assert!(fit.samples() >= MIN_SAMPLES, "{} samples", fit.samples());
        assert!(
            fit.spread() > 0.9,
            "noise is isotropic and scores well — that is the trap, not a bug: {}",
            fit.spread()
        );

        // The arithmetic does not fail. It returns a confident little sphere.
        let loose = fit.solve().expect("a noise ball still solves — that is the problem");
        assert!(loose.radius < 1000.0, "fitted radius {} nT", loose.radius);
        assert!(
            !loose.is_believable(),
            "residual {} against radius {}",
            loose.residual,
            loose.radius
        );

        // Only comparing the answer against the samples catches it.
        assert_eq!(fit.solve_for_use(), Err(FitError::Scattered));
        assert!(!fit.is_ready());
    }

    #[test]
    fn a_good_sweep_is_believable_and_solve_for_use_agrees_with_solve() {
        // The other half: the new check must not reject anything real. On a
        // properly swept run the two entry points return the identical answer.
        // Enough samples and enough sweep to clear GOOD_SAMPLES and
        // GOOD_SPREAD too, so this covers the automatic finish as well as the
        // manual one.
        let centre = Vec3::new(20_701.0, -14_915.0, -3_000.0);
        let fit = fit_of(&shell(centre, 26_252.0, PI / 2.0, 17, 32));

        let solution = fit.solve().expect("a well-covered sphere must solve");
        assert!(solution.is_believable());
        assert_eq!(fit.solve_for_use(), Ok(solution));
        assert!(fit.is_ready());
    }

    #[test]
    fn a_flat_spin_is_still_reported_as_flat_and_not_as_scattered() {
        // Two different failures wanting two different instructions. Coplanar
        // is caught in `solve`, so it must reach the caller through
        // `solve_for_use` unchanged rather than being flattened into the newer
        // error on the way past.
        let flat = fit_of(&shell(Vec3::new(20_701.0, -14_915.0, 0.0), 26_252.0, 0.0, 1, 512));

        assert_eq!(flat.solve(), Err(FitError::Coplanar));
        assert_eq!(flat.solve_for_use(), Err(FitError::Coplanar));
    }

    #[test]
    fn tilting_it_is_what_rescues_the_flat_spin() {
        // Same centre, same radius, same sample count. The only difference is
        // that the board left the plane.
        let centre = Vec3::new(20_701.0, -14_915.0, 0.0);
        let flat = fit_of(&shell(centre, 26_252.0, 0.0, 1, 512));
        let waved = fit_of(&shell(centre, 26_252.0, 0.6, 8, 64));

        assert_eq!(flat.solve(), Err(FitError::Coplanar));
        assert!(waved.solve().is_ok());
    }

    #[test]
    fn spread_is_zero_for_a_disc_and_one_for_a_ball() {
        let centre = Vec3::new(1000.0, 2000.0, -500.0);
        let disc = fit_of(&shell(centre, 26_000.0, 0.0, 1, 512));
        let ball = fit_of(&shell(centre, 26_000.0, PI / 2.0, 33, 64));

        assert!(disc.spread() < 1e-6, "a disc scored {}", disc.spread());
        assert!(ball.spread() > 0.9, "a ball scored {}", ball.spread());
    }

    #[test]
    fn calibration_puts_the_readings_back_on_a_sphere() {
        // The payoff, stated as a test: after correction, every reading has
        // the same length whichever way the board was pointing. That is what a
        // working compass *is*, and it is what the board cannot currently do.
        let centre = Vec3::new(20_701.0, -14_915.0, -3_000.0);
        let radius = 26_252.0;
        let samples = shell(centre, radius, 0.8, 9, 32);

        let before: Vec<f32> = samples.iter().map(|m| m.length()).collect();
        let worst_before = before.iter().cloned().fold(0.0f32, f32::max);
        let best_before = before.iter().cloned().fold(f32::MAX, f32::min);
        assert!(
            worst_before / best_before > 5.0,
            "uncalibrated readings should vary wildly, {best_before}..{worst_before}"
        );

        let calibration = fit_of(&samples).solve().unwrap().calibration;
        for sample in &samples {
            let corrected = calibration.apply(*sample).length();
            assert!(
                (corrected - radius).abs() < 1.0,
                "corrected reading {corrected} should be {radius}"
            );
        }
    }

    #[test]
    fn noise_lands_in_the_residual_and_not_in_the_centre() {
        let centre = Vec3::new(20_701.0, -14_915.0, -3_000.0);
        let radius = 26_252.0;

        // A deterministic wobble of ±800 nT along each sample's own radius —
        // about what this board's circle fit actually shows against a
        // 26,000 nT radius, and the reason `residual` exists at all.
        let mut samples = shell(centre, radius, 0.8, 13, 48);
        for (i, sample) in samples.iter_mut().enumerate() {
            let wobble = (((i * 37) % 11) as f32 / 10.0 - 0.5) * 1600.0;
            *sample += (*sample - centre).normalize() * wobble;
        }

        let solution = fit_of(&samples).solve().unwrap();
        let error = (solution.calibration.offset() - centre).length();
        assert!(error < 200.0, "centre should survive the noise, off by {error}");

        // Uniform noise over ±800 has an RMS of about 800/sqrt(3) ≈ 460.
        assert!(
            (300.0..700.0).contains(&solution.residual),
            "residual should report the noise, got {}",
            solution.residual
        );
    }

    #[test]
    fn units_do_not_change_the_answer() {
        let centre_nt = Vec3::new(20_701.0, -14_915.0, -3_000.0);
        let in_nanotesla = fit_of(&shell(centre_nt, 26_252.0, 0.8, 9, 32)).solve().unwrap();

        // The same geometry, measured in gauss.
        let scale = 1.0e-5;
        let in_gauss = fit_of(
            &shell(centre_nt, 26_252.0, 0.8, 9, 32)
                .into_iter()
                .map(|m| m * scale)
                .collect::<Vec<_>>(),
        )
        .solve()
        .unwrap();

        let converted = in_gauss.calibration.offset() / scale;
        let error = (converted - in_nanotesla.calibration.offset()).length();
        assert!(error < 1.0, "units changed the answer by {error} nT");
    }

    #[test]
    fn too_few_samples_is_its_own_answer() {
        let fit = fit_of(&shell(Vec3::ZERO, 26_000.0, 0.8, 4, 8));
        assert_eq!(fit.solve(), Err(FitError::TooFewSamples));
    }

    #[test]
    fn a_dead_sensor_is_dropped_rather_than_believed() {
        let centre = Vec3::new(20_701.0, -14_915.0, -3_000.0);
        let clean = shell(centre, 26_252.0, 0.8, 9, 32);

        let mut fit = Fit::new();
        for (i, sample) in clean.iter().enumerate() {
            if i % 10 == 0 {
                fit.push(Vec3::NAN);
                fit.push(Vec3::ZERO);
            }
            fit.push(*sample);
        }

        assert_eq!(fit.samples() as usize, clean.len(), "dead samples were counted");
        let error = (fit.solve().unwrap().calibration.offset() - centre).length();
        assert!(error < 2.0, "poisoned by dead samples, off by {error}");
    }

    #[test]
    fn a_run_that_never_starts_says_so_rather_than_dividing_by_zero() {
        let mut fit = Fit::new();
        for _ in 0..1000 {
            fit.push(Vec3::ZERO);
        }
        assert_eq!(fit.samples(), 0);
        assert_eq!(fit.spread(), 0.0);
        assert!(!fit.is_ready());
        assert_eq!(fit.solve(), Err(FitError::TooFewSamples));
    }

    #[test]
    fn sectors_fill_as_the_board_turns() {
        let centre = Vec3::new(20_701.0, -14_915.0, 0.0);
        let mut fit = Fit::new();
        assert_eq!(fit.sectors(), 0x00);

        for sample in shell(centre, 26_252.0, 0.0, 1, 4) {
            fit.push(sample);
        }
        assert!(fit.sectors().count_ones() >= 3 && fit.sectors() != 0xFF);

        for sample in shell(centre, 26_252.0, 0.0, 1, 64) {
            fit.push(sample);
        }
        assert_eq!(fit.sectors(), 0xFF);
    }

    #[test]
    fn a_good_figure_eight_calls_itself_finished() {
        let centre = Vec3::new(20_701.0, -14_915.0, -3_000.0);
        let fit = fit_of(&shell(centre, 26_252.0, 0.7, 16, 40));
        assert!(
            fit.is_ready(),
            "samples {}, sectors {:#010b}, spread {} (needs {})",
            fit.samples(),
            fit.sectors(),
            fit.spread(),
            GOOD_SPREAD
        );
        assert!(
            fit.spread() > GOOD_SPREAD * 1.3,
            "the threshold must be comfortably reachable, not scraped: spread {}",
            fit.spread()
        );
    }

    #[test]
    fn a_board_that_never_moved_is_not_finished_however_isotropic_it_looks() {
        // The failure this was written against, reproduced from the numbers a
        // real board reported: it sat still while the sensor jittered, so the
        // samples formed a tiny ball of noise about one point.
        //
        // Every distribution test passes. The noise is isotropic, so the spread
        // is *better* than any real hand sweep achieves — and there are plenty
        // of samples. Only the scatter gives it away.
        // Noise fills a ball rather than covering a shell, which is the whole
        // distinction: a swept sphere puts every sample on one surface, and a
        // motionless one scatters them through a volume. Nested shells at a
        // spread of radii is a filled ball without needing a random generator.
        let where_it_sat = Vec3::new(-15_000.0, -28_000.0, -54_500.0);
        let mut samples = Vec::new();
        for step in 1..=5 {
            let radius = 790.0 * (step as f32) / 5.0;
            samples.extend(shell(where_it_sat, radius, PI / 2.0, 8, 20));
        }
        let fit = fit_of(&samples);

        assert!(
            fit.spread() > GOOD_SPREAD,
            "the premise: a noise ball looks beautifully spread, got {}",
            fit.spread()
        );
        assert!(fit.samples() >= GOOD_SAMPLES, "and there are plenty of them");

        assert!(
            !fit.is_ready(),
            "a stationary board called itself finished — spread {}, samples {}",
            fit.spread(),
            fit.samples()
        );
    }

    #[test]
    fn a_run_is_finished_when_the_scatter_is_small_against_the_field() {
        // The same shape and the same sample count as the test above. The only
        // difference is that this one really is a sphere sweep, so the samples
        // sit *on* a surface rather than filling a volume.
        let centre = Vec3::new(9_265.0, -28_189.0, -11_732.0);
        let fit = fit_of(&shell(centre, 46_917.0, PI / 2.0, 20, 40));

        let solution = fit.solve().expect("a full ball must solve");
        assert!(
            solution.residual <= MAX_RESIDUAL_FRACTION * solution.radius,
            "residual {} against radius {}",
            solution.residual,
            solution.radius
        );
        assert!(fit.is_ready());
    }

    #[test]
    fn no_correction_is_the_identity() {
        let reading = Vec3::new(1.0, 2.0, 3.0);
        assert_eq!(Calibration::NONE.apply(reading), reading);
        assert_eq!(Calibration::default(), Calibration::NONE);
    }

    #[test]
    fn reset_forgets_everything() {
        let mut fit = fit_of(&shell(Vec3::ONE * 1000.0, 26_000.0, 0.8, 9, 32));
        assert!(fit.solve().is_ok());
        fit.reset();
        assert_eq!(fit.samples(), 0);
        assert_eq!(fit.sectors(), 0);
        assert_eq!(fit.solve(), Err(FitError::TooFewSamples));
    }
}

//! A calibration run, from the button press to whatever ended it.
//!
//! # Why this is not part of [`Link`](crate::serial::Link)
//!
//! `Link` holds the latest of everything and no history, which is exactly right
//! for an orientation — drawing one the board has already left is worse than
//! skipping it. A run is the opposite. It has a beginning, it accumulates, and
//! it ends, and the point cloud *is* the history. Putting a growing buffer in
//! `Link` would make its own doc comment untrue.
//!
//! # A run has three exits, and only two of them are announcements
//!
//! The board sends exactly one terminal packet — [`RunState::Solved`] or
//! [`RunState::Refused`] — at the moment a run ends, and then goes straight
//! back to its 2 Hz idle heartbeat.
//!
//! Exactly one. A single failed checksum loses it, and `parser.rs` exists
//! because failed checksums are routine. A board that resets, a pulled cable or
//! a closed port do the same thing more permanently. So there has to be a third
//! exit for "the run stopped and nobody said why", and the whole reason
//! [`RunState::Refused`] is on the wire is that this third exit must not be
//! allowed to speak for the second one.
//!
//! That was the earlier design: treat the idle heartbeat resuming as the
//! refusal signal. It reports a flat spin every time a solved packet gets
//! corrupted — a confident lie, and one that would be nearly impossible to
//! catch, because a flat spin is a plausible thing to have just done.
//! [`Outcome::Ended`] says less, and says it honestly.
//!
//! # Time is a parameter
//!
//! The silence timer needs a clock, and reaching for one here would make this
//! module untestable and drag `web_sys` into arithmetic that is otherwise pure.
//! So the caller passes milliseconds in. Both callers use `Date::now()`, and
//! they must keep using the same clock as each other — a monotonic frame timer
//! in one and a wall clock in the other would compare unrelated numbers.

use packet::glam::Vec3;
use packet::{FitStatus, RunState};

/// Stop accumulating past this many samples.
///
/// 160 seconds at the 50 Hz a run streams at, against runs that take 25 to 30.
/// Reaching it means something has gone wrong rather than that a very thorough
/// calibration is under way.
///
/// Past the cap it stops pushing rather than dropping the oldest. Thinning the
/// front of a point cloud would quietly misrepresent coverage, which is the one
/// thing the cloud is on screen to show.
pub const SAMPLE_CAP: usize = 8192;

/// How long a gap in the calibration stream means the run is over, in
/// milliseconds.
///
/// A run streams at 50 Hz and even an idle board sends at 2 Hz, so half a
/// second of nothing is already abnormal and two seconds is unambiguous. This
/// is not a timeout on the calibration — the board decides when that ends — it
/// is a timeout on hearing from the board at all.
pub const SILENCE_MS: f64 = 2000.0;

/// Which view the page should be showing.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum Mode {
    /// The board model. The ordinary state, and the one older firmware never
    /// leaves — stages 5 and 6 send no calibration packets at all.
    #[default]
    Model,
    /// A run is going: the point cloud.
    Calibrating,
    /// A run has finished. Back to the model, with [`Run::outcome`] holding
    /// what happened.
    Landed,
}

/// How a run finished.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Outcome {
    /// The board solved and adopted the offset in the packet.
    Solved(packet::Calibration),
    /// The board refused the fit and kept its old offset. The status says why.
    Refused(FitStatus),
    /// The run stopped without a terminal packet — dropped frame, reset board,
    /// unplugged cable, closed port. **Deliberately says nothing about why.**
    Ended,
}

/// The accumulated state of the current or most recent run.
pub struct Run {
    mode: Mode,
    samples: Vec<Vec3>,
    latest: Option<packet::Calibration>,
    outcome: Option<Outcome>,
    /// When the last calibration packet of any kind arrived.
    last_seen: f64,
    /// Set if [`SAMPLE_CAP`] was reached, so the display can admit the cloud is
    /// not the whole run rather than silently showing a truncated one.
    truncated: bool,
}

impl Default for Run {
    fn default() -> Self {
        Self {
            mode: Mode::default(),
            samples: Vec::new(),
            latest: None,
            outcome: None,
            // Not zero: a zero here with a wall-clock `now` would read as a gap
            // of fifty-six years, which is only harmless because the silence
            // check requires `Calibrating` to have been entered first.
            last_seen: f64::NEG_INFINITY,
            truncated: false,
        }
    }
}

impl Run {
    pub fn mode(&self) -> Mode {
        self.mode
    }

    /// The raw, uncorrected samples of this run, oldest first.
    pub fn samples(&self) -> &[Vec3] {
        &self.samples
    }

    /// How the last run finished.
    ///
    /// Read by the tests and by nothing else yet — the status line goes through
    /// [`status`](Self::status) instead. 2.2 is what draws an outcome, and until
    /// then this is the seam it will read through.
    #[allow(dead_code)]
    pub fn outcome(&self) -> Option<&Outcome> {
        self.outcome.as_ref()
    }

    /// Whether the cloud is the whole run or only the first [`SAMPLE_CAP`] of
    /// it. Same story as [`outcome`](Self::outcome): 2.2 has to show this
    /// rather than display a truncated cloud as if it were complete.
    #[allow(dead_code)]
    pub fn truncated(&self) -> bool {
        self.truncated
    }

    /// Take in one calibration packet. `now` is milliseconds on any clock, as
    /// long as it is the same one [`tick`](Self::tick) is given.
    pub fn push(&mut self, packet: packet::Calibration, now: f64) {
        self.last_seen = now;

        match packet.state {
            // The heartbeat between runs. Ignored while idle — it is the
            // ordinary state of a connected board, not an event. Arriving
            // *during* a run is the third exit: the terminal packet never came.
            RunState::Idle => {
                if self.mode == Mode::Calibrating {
                    self.finish(Outcome::Ended);
                }
                return;
            }

            // Entering on `state != Idle` rather than on packet type is what
            // makes this work at all: `07_fused` has been sending idle
            // calibration packets since it booted, so "the first calibration
            // packet" would fire within half a second of connecting.
            RunState::Collecting | RunState::Ready => {
                if self.mode != Mode::Calibrating {
                    self.begin();
                }
            }

            RunState::Solved | RunState::Refused => {}
        }

        self.record(packet.sample);
        self.latest = Some(packet);

        match packet.state {
            RunState::Solved => self.finish(Outcome::Solved(packet)),
            RunState::Refused => self.finish(Outcome::Refused(packet.fit)),
            _ => {}
        }
    }

    /// Called every frame. Ends a run that has gone quiet.
    pub fn tick(&mut self, now: f64) {
        if self.mode == Mode::Calibrating && now - self.last_seen > SILENCE_MS {
            self.finish(Outcome::Ended);
        }
    }

    /// The port closed. Faster and more certain than waiting for the silence
    /// timer, and the same conclusion.
    pub fn disconnected(&mut self) {
        if self.mode == Mode::Calibrating {
            self.finish(Outcome::Ended);
        }
    }

    /// The line under the canvas, or `None` to leave the packet counter alone.
    ///
    /// Formatting lives here rather than in `dom` because it is the one part of
    /// the display that can be tested, and because getting the refusal wording
    /// right is the point of the whole exercise.
    pub fn status(&self) -> Option<String> {
        match self.mode {
            Mode::Model => None,

            Mode::Calibrating => {
                let packet = self.latest.as_ref()?;
                let ready = if packet.state == RunState::Ready {
                    " · ready — press again to finish"
                } else {
                    ""
                };
                Some(format!(
                    "calibrating · {} samples · spread {:.3}{ready}",
                    packet.samples, packet.spread
                ))
            }

            Mode::Landed => Some(match self.outcome.as_ref()? {
                Outcome::Solved(packet) => format!(
                    "solved · offset {:.0}, {:.0}, {:.0} nT · field {:.0} nT · scatter {:.1}%",
                    packet.offset.x,
                    packet.offset.y,
                    packet.offset.z,
                    packet.radius,
                    100.0 * packet.residual / packet.radius,
                ),

                // The board keeps its old correction in every one of these, and
                // the text says so, because "not solved" on its own reads as
                // "the compass is now broken".
                Outcome::Refused(FitStatus::Coplanar) => {
                    "not solved — the samples are flat. Tip and roll it, not just turn it. \
                     Old correction kept"
                        .to_string()
                }
                // Not the flat spin, and the instruction is different: a board
                // that barely moved makes an isotropic ball of sensor noise,
                // which passes every check that looks at where the samples sit.
                Outcome::Refused(FitStatus::Scattered) => {
                    "not solved — the samples are not on a sphere. It barely moved. \
                     Old correction kept"
                        .to_string()
                }
                Outcome::Refused(FitStatus::TooFewSamples) => {
                    "not solved — too few samples. Old correction kept".to_string()
                }
                Outcome::Refused(_) => {
                    "not solved — the fit would not solve. Old correction kept".to_string()
                }

                // Says nothing about why, because it does not know. The board
                // may well have solved and had the packet corrupted on the way.
                Outcome::Ended => {
                    "the run ended without an answer — check the board".to_string()
                }
            }),
        }
    }

    fn begin(&mut self) {
        self.samples.clear();
        self.outcome = None;
        self.truncated = false;
        self.mode = Mode::Calibrating;
    }

    fn finish(&mut self, outcome: Outcome) {
        self.outcome = Some(outcome);
        self.mode = Mode::Landed;
    }

    fn record(&mut self, sample: Vec3) {
        if self.samples.len() < SAMPLE_CAP {
            self.samples.push(sample);
        } else {
            self.truncated = true;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn packet(state: RunState, fit: FitStatus) -> packet::Calibration {
        packet::Calibration {
            sample: Vec3::new(21_000.0, -14_915.0, -33_100.0),
            offset: Vec3::new(9265.5, -28_189.5, -11_732.9),
            radius: 46_917.9,
            residual: 1158.5,
            spread: 0.1234,
            samples: 1512,
            sectors: 0b1110_1111,
            state,
            fit,
        }
    }

    fn idle() -> packet::Calibration {
        packet(RunState::Idle, FitStatus::Ok)
    }

    fn collecting() -> packet::Calibration {
        packet(RunState::Collecting, FitStatus::TooFewSamples)
    }

    /// Drive a run up to but not including its ending. Returns the clock.
    fn a_run_in_progress(run: &mut Run) -> f64 {
        let mut now = 1000.0;
        run.push(idle(), now);
        for _ in 0..40 {
            now += 20.0;
            run.push(collecting(), now);
        }
        now
    }

    #[test]
    fn an_idle_board_does_not_put_the_page_into_calibration_mode() {
        // The failure the whole `state != Idle` rule exists to prevent:
        // `07_fused` sends idle calibration packets at 2 Hz from boot, so
        // triggering on packet *type* would enter the mode on connect.
        let mut run = Run::default();
        for tick in 0..20 {
            run.push(idle(), 1000.0 + tick as f64 * 500.0);
        }

        assert_eq!(run.mode(), Mode::Model);
        assert_eq!(run.status(), None);
        assert!(run.samples().is_empty());
    }

    #[test]
    fn a_run_starts_on_the_first_packet_that_is_not_idle() {
        let mut run = Run::default();
        run.push(idle(), 1000.0);
        assert_eq!(run.mode(), Mode::Model);

        run.push(collecting(), 1020.0);
        assert_eq!(run.mode(), Mode::Calibrating);
        assert_eq!(run.samples().len(), 1);
    }

    #[test]
    fn solving_lands_with_the_offset() {
        let mut run = Run::default();
        let now = a_run_in_progress(&mut run);

        let solved = packet(RunState::Solved, FitStatus::Ok);
        run.push(solved, now + 20.0);

        assert_eq!(run.mode(), Mode::Landed);
        assert_eq!(run.outcome(), Some(&Outcome::Solved(solved)));
        // The terminal packet's own sample belongs to the run too.
        assert_eq!(run.samples().len(), 41);
        assert!(run.status().unwrap().starts_with("solved · offset"));
    }

    #[test]
    fn a_refusal_lands_with_its_reason_and_never_claims_the_offset_changed() {
        let mut run = Run::default();
        let now = a_run_in_progress(&mut run);

        run.push(packet(RunState::Refused, FitStatus::Coplanar), now + 20.0);

        assert_eq!(run.mode(), Mode::Landed);
        assert_eq!(run.outcome(), Some(&Outcome::Refused(FitStatus::Coplanar)));

        let status = run.status().unwrap();
        assert!(status.contains("flat"), "{status}");
        assert!(status.contains("Old correction kept"), "{status}");
        assert!(!status.contains("solved ·"), "{status}");
    }

    #[test]
    fn a_lost_terminal_packet_ends_the_run_without_naming_a_reason() {
        // The bug the explicit `Refused` variant was added to prevent. The
        // board solved; the packet was corrupted; the idle heartbeat resumed.
        // Reading that as a flat spin would be a lie, and this is the test that
        // pins it.
        let mut run = Run::default();
        let now = a_run_in_progress(&mut run);

        run.push(idle(), now + 500.0);

        assert_eq!(run.mode(), Mode::Landed);
        assert_eq!(run.outcome(), Some(&Outcome::Ended));

        let status = run.status().unwrap();
        assert!(status.contains("without an answer"), "{status}");
        assert!(!status.contains("flat"), "{status}");
        assert!(!status.contains("too few"), "{status}");
    }

    #[test]
    fn a_board_that_stops_talking_ends_the_run() {
        let mut run = Run::default();
        let now = a_run_in_progress(&mut run);

        // Still inside the window: a run that is merely between packets is not
        // over, and ending it here would abort every real calibration.
        run.tick(now + SILENCE_MS - 1.0);
        assert_eq!(run.mode(), Mode::Calibrating);

        run.tick(now + SILENCE_MS + 1.0);
        assert_eq!(run.mode(), Mode::Landed);
        assert_eq!(run.outcome(), Some(&Outcome::Ended));
    }

    #[test]
    fn the_silence_timer_cannot_fire_before_a_run_has_started() {
        // `last_seen` starts at negative infinity, so a naive elapsed-time
        // check would be true on the very first frame.
        let mut run = Run::default();
        run.tick(1000.0);
        run.tick(f64::MAX);

        assert_eq!(run.mode(), Mode::Model);
        assert_eq!(run.outcome(), None);
    }

    #[test]
    fn closing_the_port_ends_a_run_but_leaves_a_finished_one_alone() {
        let mut run = Run::default();
        let now = a_run_in_progress(&mut run);
        run.disconnected();
        assert_eq!(run.outcome(), Some(&Outcome::Ended));

        let mut solved_run = Run::default();
        let now = now.max(a_run_in_progress(&mut solved_run));
        let solved = packet(RunState::Solved, FitStatus::Ok);
        solved_run.push(solved, now + 20.0);
        solved_run.disconnected();

        // Disconnecting after the answer arrived must not overwrite it.
        assert_eq!(solved_run.outcome(), Some(&Outcome::Solved(solved)));
    }

    #[test]
    fn a_second_run_clears_the_first_ones_cloud() {
        let mut run = Run::default();
        let mut now = a_run_in_progress(&mut run);
        run.push(packet(RunState::Solved, FitStatus::Ok), now + 20.0);
        assert_eq!(run.samples().len(), 41);

        now += 1000.0;
        run.push(collecting(), now);

        assert_eq!(run.mode(), Mode::Calibrating);
        assert_eq!(run.samples().len(), 1, "the previous cloud was kept");
        assert_eq!(run.outcome(), None, "the previous answer was kept");
    }

    #[test]
    fn joining_a_stream_that_is_already_calibrating_still_works() {
        // Connecting mid-run: no idle packet was ever seen, and the first thing
        // to arrive is a collecting one.
        let mut run = Run::default();
        run.push(collecting(), 1000.0);

        assert_eq!(run.mode(), Mode::Calibrating);
        assert!(run.status().unwrap().starts_with("calibrating ·"));
    }

    #[test]
    fn the_cloud_stops_growing_at_the_cap_rather_than_dropping_its_oldest() {
        let mut run = Run::default();
        let mut now = 1000.0;

        for _ in 0..SAMPLE_CAP + 100 {
            now += 1.0;
            run.push(collecting(), now);
        }

        assert_eq!(run.samples().len(), SAMPLE_CAP);
        assert!(run.truncated());
        // Oldest first, and the oldest is still the first one pushed.
        assert_eq!(run.samples()[0], collecting().sample);
    }

    #[test]
    fn a_run_survives_the_trip_through_raw_bytes() {
        // The one seam nothing else covers: `serial.rs` feeds chunks to
        // `Parser` and pushes what comes out into `Run`. Everything either side
        // is tested, so this is about the join — and about arriving in chunks
        // that do not line up with packets, which is the only way bytes ever
        // arrive off a serial port.
        use crate::parser::Parser;

        let mut stream = Vec::new();
        let mut expected_samples = 0;

        stream.extend_from_slice(&idle().encode());
        for _ in 0..30 {
            stream.extend_from_slice(&collecting().encode());
            expected_samples += 1;
            // Three orientation packets to every calibration one, as the board
            // actually sends them.
            for _ in 0..3 {
                stream.extend_from_slice(
                    &packet::Orientation {
                        rotation: packet::glam::Quat::IDENTITY,
                    }
                    .encode(),
                );
            }
        }
        stream.extend_from_slice(&packet(RunState::Refused, FitStatus::Coplanar).encode());
        expected_samples += 1;
        stream.extend_from_slice(&idle().encode());

        let mut parser = Parser::default();
        let mut run = Run::default();
        let mut now = 1000.0;

        // Seven bytes at a time, which lines up with nothing — every packet
        // gets split across at least one chunk boundary.
        for chunk in stream.chunks(7) {
            now += 1.0;
            for decoded in parser.feed(chunk) {
                if let packet::Packet::Calibration(c) = decoded {
                    run.push(c, now);
                }
            }
        }

        assert_eq!(parser.bad, 0, "a good stream was rejected");
        assert_eq!(run.mode(), Mode::Landed);
        assert_eq!(run.outcome(), Some(&Outcome::Refused(FitStatus::Coplanar)));
        assert_eq!(run.samples().len(), expected_samples);

        // The idle packet after the refusal is the heartbeat resuming, and it
        // must not overwrite the answer with `Ended`.
        assert!(run.status().unwrap().contains("flat"));
    }

    #[test]
    fn every_run_state_is_handled() {
        // Not a formality. `push` matches on `RunState` twice, and a variant
        // added to the wire without a decision here would fall through its
        // catch-all arm and silently do nothing.
        //
        // The `expected` match has no wildcard, so adding a variant stops this
        // file compiling. That is the reminder — a list on its own would just
        // go quietly out of date.
        for state in [
            RunState::Idle,
            RunState::Collecting,
            RunState::Ready,
            RunState::Solved,
            RunState::Refused,
        ] {
            let mut run = Run::default();
            run.push(packet(state, FitStatus::Ok), 1000.0);

            let expected = match state {
                RunState::Idle => Mode::Model,
                RunState::Collecting | RunState::Ready => Mode::Calibrating,
                RunState::Solved | RunState::Refused => Mode::Landed,
            };
            assert_eq!(run.mode(), expected, "{state:?} went to the wrong mode");
        }
    }
}

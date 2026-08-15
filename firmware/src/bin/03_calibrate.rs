//! Stage 3 — measuring how much the compass lies, and taking it back off.
//!
//! Stage 2 built a compass and the compass does not work. Turn the board
//! through a full circle and the lit LED does not hold still; at one bearing it
//! flickers between neighbours as though it cannot make up its mind, which is
//! exactly what is happening — at that bearing there is almost no signal left
//! to make it up from.
//!
//! Nothing is wrong with the code in stage 2. The sensor is reporting the field
//! it is actually sitting in, and that field is not the earth's alone.
//!
//! ```text
//! cargo run --bin 03_calibrate
//! ```
//!
//! **PASS**, in four parts:
//!
//! 1. At boot it behaves exactly like stage 2 — which is to say, badly. Turn it
//!    and watch the light wander. That is the same failure, reproduced here on
//!    purpose so the fix has something to fix.
//! 2. **Press the blue USER button and turn the board where it stands** — tip
//!    it, roll it right over, every way up, but keep it in one spot. The ring
//!    stops showing a heading and becomes a bar that fills as the run gets
//!    closer to solvable. It blinks while there is work left and goes solid
//!    when there is not.
//! 3. **Spin it flat on the desk instead, and it never finishes.** However
//!    long you do it for. That is not the board being fussy — see below.
//!    **Nor does carrying it around the desk work**, for a different reason:
//!    the field is not the same in two places, so the readings are not samples
//!    of one sphere and the fit has nothing true to find. Measured, not
//!    guessed — a carried run fitted a 20,900 nT field with a quarter of it as
//!    scatter, where turning it in place fitted 46,900 nT to 2.5%.
//! 4. When it solves, it prints the answer and goes back to being a compass.
//!    Now turn the board through a full circle: **the lit LED stays pointing
//!    the same way in the room.** That is what stage 2 promised.
//!
//! # What is wrong with the compass
//!
//! Something on this board is magnetic — a component, a solder joint, a bit of
//! plating — and it turns with the board. So it adds the same vector to every
//! reading, in the board's own frame, forever.
//!
//! Every reading a magnetometer could take in a clean field lies on a sphere
//! centred on the origin: turn the sensor and only the direction changes,
//! never the length. Add a constant to all of them and **the sphere moves off
//! the origin.** It is still a sphere and still the same size. It is just in
//! the wrong place, and every angle measured from the origin is therefore wrong
//! by a different amount depending on which way you are facing.
//!
//! On this board the displacement is about two thirds of the sphere's own
//! radius — and in the horizontal plane, which is the part a compass steers by,
//! it is slightly *larger* than the radius. That is why one bearing has almost
//! nothing left, and it is a broken compass rather than an inaccurate one.
//!
//! **A magnet on the desk cannot do this.** Turn the board in place under any
//! field that is fixed in the *room* and the readings still sweep a circle
//! centred on the origin — the radius changes, the centre does not. Only
//! something that physically turns with the board can move it. Which is the
//! good news: the answer belongs to this board, travels with it, and can be
//! compiled in.
//!
//! # Why waving it is not superstition
//!
//! Finding the centre of a sphere from points on its surface is a least-squares
//! fit, and `magcal` has the whole argument for why it is linear and why it
//! needs no memory. The part that matters at the bench is the failure:
//!
//! **Spin the board flat and every sample has the same vertical component.**
//! That makes two columns of the system identical, the system singular, and the
//! answer non-unique — there are infinitely many sphere centres that fit one
//! circle equally well, because a circle genuinely does not know where the
//! centre is along the axis you never left.
//!
//! This is why your phone asks you to wave it in a figure eight. Not to collect
//! more data. Flat samples, however many, cannot answer the question, and the
//! board refuses rather than inventing a number.
//!
//! # The two constants at the bottom of the bench, still
//!
//! [`HEADING_SENSE`] and [`HEADING_OFFSET_DEG`] are carried over from stage 2
//! unchanged, and they are **deliberately still both at their do-nothing
//! values.** Do not tune them until a calibration run has been done and the
//! light holds still: any value fitted before that is fitting the hard iron,
//! and will be wrong the moment the hard iron is gone.
//!
//! # What is deliberately not here
//!
//! The part's own offset cancellation. The AGR can measure and subtract the
//! *die's* zero error using set/reset pulses through an internal coil, and it
//! is left switched off, because turning it on would split the constant this
//! stage measures into two constants measured two different ways. There is one
//! offset to remove and it does not matter which parts of the board contributed
//! to it.
//!
//! The accelerometer, too. Stage 2 asked it who it was and never read a byte
//! from it, and neither does this. The difference between that file and this
//! one is the calibration and nothing else.

#![no_main]
#![no_std]

use defmt_rtt as _;
use lsm303agr::{Lsm303agr, MagMode, MagOutputDataRate};
use magcal::glam::Vec3;
use magcal::{Calibration, Fit, FitError, GOOD_SAMPLES, GOOD_SPREAD};
use panic_probe as _;
use stm32f3xx_hal::delay::Delay;
use stm32f3xx_hal::hal::blocking::delay::DelayMs;
use stm32f3xx_hal::hal::digital::v2::InputPin;
use stm32f3xx_hal::i2c::I2c;
use stm32f3xx_hal::{self as hal, prelude::*};

/// What the magnetometer's `WHO_AM_I` must read on an LSM303AGR.
const EXPECTED_MAG_ID: u8 = 0x40;

/// And the accelerometer's, which is a separate device on a separate address.
const EXPECTED_ACCEL_ID: u8 = 0x33;

/// The hard iron this board carries, in nanotesla, in the board's own frame.
///
/// **These are one particular board's numbers, not the part's and not yours.**
/// If you are running this on your own Discovery, the ones below are wrong for
/// it — press the button, wave it about, and paste in what it prints.
///
/// Zeros mean "no correction", which is what stage 2 was doing without saying
/// so. Leaving them at zero is a perfectly good way to see the problem first.
///
/// # Where these came from
///
/// **The mean of two thirty-second runs**, taken minutes apart, each tipping,
/// rolling *and* turning the board on one spot. Individually they reported
/// `(7823.8, -29652.6, -7859.7)` and `(8301.4, -31547.4, -10226.0)` — about
/// 3,000 nT apart.
///
/// That disagreement is not the fit being unreliable. Scored against each
/// other's samples the two offsets are nearly interchangeable, costing 0.24 and
/// 0.63 percentage points of extra scatter; the minimum is simply shallow, so
/// the centre wanders while the quality barely moves. Their mean has the best
/// worst case of the three — 3.65% against 3.86% and 3.99% — which is the
/// honest reason to average rather than to pick a favourite.
///
/// An earlier run that only *tilted* the board reported the best residual of
/// the lot, 0.90%, and was the worst offset of the lot. See [`GOOD_SPREAD`] in
/// `magcal` for why, because it is the whole reason that threshold is 0.30.
const MAG_OFFSET_NT: [f32; 3] = [8062.6, -30600.0, -9042.9];

/// Which way the heading runs. `-1.0` if the ring turns with the board
/// instead of against it — see the note at the top of this file.
const HEADING_SENSE: f32 = 1.0;

/// Rotation between the sensor's idea of zero and the ring's North.
const HEADING_OFFSET_DEG: f32 = 0.0;

/// Display update interval. The magnetometer is configured for 50 Hz below,
/// so this is not throwing readings away.
const STEP_MS: u16 = 20;

/// How many consecutive reads of the same button level count as real.
///
/// A switch does not close once; its contacts bounce apart and back together
/// for a few milliseconds, and a program reading fast enough sees that as a
/// burst of presses. Three steps is 60 ms, which is far longer than the bounce
/// and far shorter than anyone can notice.
const DEBOUNCE_STEPS: u8 = 3;

/// Ring blink half-period while the fit still needs more, in display steps.
const BLINK_STEPS: u32 = 8;

/// The shortest a calibration run is allowed to be, in seconds.
///
/// **Not a quality threshold — a human one.** `magcal` decides whether the
/// samples can be solved, and it will happily say yes before there has been
/// time to perform the whole instruction. But the instruction is two motions —
/// tip and roll it, *and* turn it right round — and a run that ends after the
/// first one gets an answer that fits its own samples perfectly and disagrees
/// with the next run. Measured on this board: two runs a few minutes apart
/// disagreed by 4,300 nT in offset and 7,600 nT in field, both with excellent
/// residuals, because one of them never got the second motion.
///
/// So the board will not call itself finished before this. Pressing the button
/// still stops it by hand at any point.
const MIN_RUN_SECONDS: u32 = 30;
const MIN_RUN_STEPS: u32 = MIN_RUN_SECONDS * 1000 / STEP_MS as u32;

/// Light exactly the LEDs whose bit is set. See stage 1 for what `BSRR` is.
fn show(pattern: u8) {
    let set = u32::from(pattern) << 8;
    let reset = u32::from(!pattern) << 24;

    // SAFETY: BSRR is write-only, its bits act independently, and the eight
    // pins it touches were configured as outputs and belong to nothing else.
    unsafe {
        (*hal::pac::GPIOE::ptr()).bsrr.write(|w| w.bits(set | reset));
    }
}

/// Fold an angle into `0..360`.
///
/// `f32::rem_euclid` lives in the standard library, which is not here.
fn wrap_degrees(degrees: f32) -> f32 {
    let wrapped = libm::fmodf(degrees, 360.0);
    if wrapped < 0.0 { wrapped + 360.0 } else { wrapped }
}

/// Ring position — 0 is North, counting clockwise — to its bit in the byte.
///
/// PE8 is the NW position and PE9 is North, so North is bit 1 and everything
/// else follows round from there.
fn led_bit(position: u32) -> u8 {
    1u8 << ((position + 1) % 8)
}

/// The byte that points at a heading.
///
/// The circle is cut into sixteen, not eight. Land on an LED and one lights;
/// land between two and both light, which reads as "somewhere in here". The
/// ring has eight lights and sixteen things it can say.
fn ring_pattern(heading_deg: f32) -> u8 {
    let sector = libm::roundf(heading_deg / 22.5) as i32;
    let sector = sector.rem_euclid(16);
    let position = (sector / 2) as u32;

    if sector % 2 == 0 {
        led_bit(position)
    } else {
        led_bit(position) | led_bit((position + 1) % 8)
    }
}

/// The ring as a bar: how much of a calibration run is done, in eighths.
///
/// **This started out as a coverage display** — one light per bearing visited —
/// and the board threw it out. All eight bearings fill within about half a
/// second of waving, so the ring went solid immediately and then said nothing
/// for the next thirty seconds. It was showing the easy half of the problem.
///
/// The hard half is getting the samples *out of a plane*, and that is what this
/// shows instead: a bar that grows as the run gets closer to being solvable, so
/// a full ring means finished and a half ring means half way.
fn progress_pattern(fraction: f32) -> u8 {
    let lit = if fraction <= 0.0 {
        0
    } else if fraction >= 1.0 {
        8
    } else {
        (fraction * 8.0) as u32 + 1
    };

    let mut pattern = 0u8;
    for position in 0..lit {
        pattern |= led_bit(position);
    }
    pattern
}

/// A button, with the bouncing taken out.
///
/// Reports the moment of pressing rather than the state, because "start
/// calibrating" should happen once per press and not fifty times a second for
/// as long as a finger is resting on it.
struct Button {
    settled: bool,
    candidate: bool,
    agreed: u8,
}

impl Button {
    fn new(level: bool) -> Self {
        Self {
            settled: level,
            candidate: level,
            agreed: 0,
        }
    }

    /// Feed it this step's reading. `true` exactly once per press.
    fn pressed(&mut self, level: bool) -> bool {
        if level == self.candidate {
            self.agreed = self.agreed.saturating_add(1);
        } else {
            self.candidate = level;
            self.agreed = 1;
        }

        if self.agreed >= DEBOUNCE_STEPS && self.candidate != self.settled {
            self.settled = self.candidate;
            return self.settled;
        }
        false
    }
}

/// What the program is currently doing.
enum Mode {
    /// Being a compass, using whatever correction it currently has.
    Compass,
    /// Collecting samples for a new one.
    Calibrating {
        fit: Fit,
        /// The step the run began on, so it can be held open for
        /// [`MIN_RUN_SECONDS`].
        started: u32,
    },
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let dp = hal::pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        .use_hse(8.MHz())
        .bypass_hse()
        .sysclk(72.MHz())
        .freeze(&mut flash.acr);

    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);
    let mut gpioe = dp.GPIOE.split(&mut rcc.ahb);

    gpioe.pe8.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    gpioe.pe9.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    gpioe.pe10.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    gpioe.pe11.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    gpioe.pe12.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    gpioe.pe13.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    gpioe.pe14.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    gpioe.pe15.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);

    // The blue USER button, and the first time this project has read a pin
    // rather than driven one. It is the same peripheral and the same idea
    // pointing the other way: stage 1 wrote to `BSRR`, this reads `IDR`, and
    // the HAL is doing that underneath.
    //
    // Pulled down, because a pin with nothing connected does not read low — it
    // reads whatever charge happens to be on it, and changes its mind when you
    // move your hand nearby. On this board the button connects the pin to 3V3,
    // so the resistor is what makes "not pressed" mean something.
    let button_pin = gpioa
        .pa0
        .into_pull_down_input(&mut gpioa.moder, &mut gpioa.pupdr);

    // I2C pins are open-drain: every device on the bus can only pull the line
    // down, never drive it up, and a pull-up resistor does the rest. That is
    // what lets several chips share two wires without destroying each other.
    let scl = gpiob
        .pb6
        .into_af_open_drain::<4>(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
    let sda = gpiob
        .pb7
        .into_af_open_drain::<4>(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);

    let i2c = I2c::new(dp.I2C1, (scl, sda), 100_000.Hz(), clocks, &mut rcc.apb1);
    let mut delay = Delay::new(cp.SYST, clocks);

    let mut compass = Lsm303agr::new_with_i2c(i2c);

    defmt::println!("stage 3 - calibrating the compass");
    defmt::println!("=================================");

    // Step one, always: ask both chips who they are.
    let mag_id = compass.magnetometer_id().unwrap();
    let accel_id = compass.accelerometer_id().unwrap();
    defmt::println!(
        "magnetometer  = {:#04x} (expect {:#04x})",
        mag_id.raw(),
        EXPECTED_MAG_ID
    );
    defmt::println!(
        "accelerometer = {:#04x} (expect {:#04x})",
        accel_id.raw(),
        EXPECTED_ACCEL_ID
    );
    if mag_id.raw() != EXPECTED_MAG_ID || accel_id.raw() != EXPECTED_ACCEL_ID {
        defmt::println!("!! wrong answer - stop here, nothing below this line means anything");
    }

    compass.init().unwrap();
    compass
        .set_mag_mode_and_odr(&mut delay, MagMode::HighResolution, MagOutputDataRate::Hz50)
        .unwrap();

    // Out of one-shot mode: keep measuring on your own, and I will read
    // whatever is latest whenever I get round to it.
    let mut compass = compass.into_mag_continuous().ok().unwrap();

    let mut calibration = Calibration::from_offset(Vec3::from_array(MAG_OFFSET_NT));

    defmt::println!("");
    if calibration.offset() == Vec3::ZERO {
        defmt::println!("no correction compiled in - this will behave exactly like stage 2");
    } else {
        let o = calibration.offset();
        defmt::println!(
            "compiled-in correction {=f32} {=f32} {=f32} nT",
            o.x,
            o.y,
            o.z
        );
        defmt::println!("that is ONE BOARD'S hard iron. If this is not that board, recalibrate.");
    }
    defmt::println!("");
    defmt::println!("Press the blue USER button, then TURN THE BOARD WHERE IT IS.");
    defmt::println!("Tip it, roll it right over, every way up - but keep it in one spot.");
    defmt::println!("Carrying it around the desk measures a different field in each place");
    defmt::println!("and fits a sphere to something that is not one.");
    defmt::println!("Spinning it flat is not enough either, and will never finish.");
    defmt::println!("");

    let mut mode = Mode::Compass;
    let mut button = Button::new(button_pin.is_high().unwrap_or(false));
    let mut ticks: u32 = 0;

    loop {
        let field = compass.magnetic_field().unwrap();

        // Nanotesla, straight from the driver — 150 of them per count, which is
        // the one number this sensor's datasheet exists to tell you. The earth
        // manages about 50,000 nT, so expect tens of thousands here.
        let raw = Vec3::new(
            field.x_nt() as f32,
            field.y_nt() as f32,
            field.z_nt() as f32,
        );

        // On this board the button pulls the pin up to 3V3, so high is pressed.
        // That is a fact about the Discovery's schematic, not about buttons.
        let press = button.pressed(button_pin.is_high().unwrap_or(false));
        ticks = ticks.wrapping_add(1);

        match &mut mode {
            Mode::Compass => {
                // The whole of the fix, in one line. Everything else in this
                // file exists to work out what to put in it.
                let corrected = calibration.apply(raw);

                // atan2 rather than atan, because atan cannot tell which
                // quadrant it is in and would fold the circle in half.
                let angle = libm::atan2f(corrected.y, corrected.x).to_degrees();
                let heading = wrap_degrees(angle * HEADING_SENSE + HEADING_OFFSET_DEG);
                show(ring_pattern(heading));

                if ticks % 50 == 0 {
                    defmt::println!(
                        "field {=f32} {=f32} {=f32} nT   heading {=f32} deg",
                        corrected.x,
                        corrected.y,
                        corrected.z,
                        heading
                    );
                }

                if press {
                    defmt::println!("");
                    defmt::println!(
                        "calibrating for at least {} s - tip and roll it, AND turn it",
                        MIN_RUN_SECONDS
                    );
                    defmt::println!("right round, all without moving it off its spot.");
                    mode = Mode::Calibrating {
                        fit: Fit::new(),
                        started: ticks,
                    };
                }
            }

            Mode::Calibrating { fit, started } => {
                // Raw, not corrected. The sphere being measured is the one the
                // sensor actually reports; correcting first would be measuring
                // the answer against itself.
                fit.push(raw);

                let elapsed = ticks.wrapping_sub(*started);
                let long_enough = elapsed >= MIN_RUN_STEPS;
                let ready = long_enough && fit.is_ready();

                // Whichever requirement is furthest from being met is the one
                // worth showing, so the bar never promises more than the run
                // has actually got. The clock counts as a requirement — without
                // it the bar would fill long before the run could end and then
                // sit there full, which reads as a hang rather than progress.
                let by_samples = fit.samples() as f32 / GOOD_SAMPLES as f32;
                let by_spread = fit.spread() / GOOD_SPREAD;
                let by_clock = elapsed as f32 / MIN_RUN_STEPS as f32;
                let lit = progress_pattern(by_samples.min(by_spread).min(by_clock));

                // Blinking while there is still work to do, solid when done.
                let blink_off = !ready && (ticks / BLINK_STEPS) % 2 == 0;
                show(if blink_off { 0 } else { lit });

                if ticks % 25 == 0 {
                    defmt::println!(
                        "samples {=u32}  bearings {=u8:#010b}  spread {=f32}",
                        fit.samples(),
                        fit.sectors(),
                        fit.spread()
                    );
                }

                if ready || press {
                    if press && !ready {
                        defmt::println!("stopped by hand");
                    }
                    // `solve_for_use`, not `solve`. A manual stop is allowed to
                    // skip the sample count and spread that an automatic finish
                    // waits for; it is not allowed to skip the check that the
                    // sphere found actually describes the samples.
                    match fit.solve_for_use() {
                        Ok(solution) => {
                            let o = solution.calibration.offset();
                            calibration = solution.calibration;
                            defmt::println!("");
                            defmt::println!("SOLVED from {=u32} samples", solution.samples);
                            defmt::println!(
                                "  offset    {=f32} {=f32} {=f32} nT",
                                o.x,
                                o.y,
                                o.z
                            );
                            defmt::println!("  field     {=f32} nT", solution.radius);
                            defmt::println!(
                                "  scatter   {=f32} nT ({=f32} of the field)",
                                solution.residual,
                                solution.residual / solution.radius
                            );
                            defmt::println!("");
                            defmt::println!("paste this into the top of 03_calibrate.rs:");
                            defmt::println!(
                                "  const MAG_OFFSET_NT: [f32; 3] = [{=f32}, {=f32}, {=f32}];",
                                o.x,
                                o.y,
                                o.z
                            );
                            defmt::println!("");
                            defmt::println!("now turn the board - the lit LED should stay put");
                        }
                        Err(FitError::Coplanar) => {
                            defmt::println!("");
                            defmt::println!("NOT SOLVED - the samples are flat.");
                            defmt::println!("A circle does not say where the centre of a sphere is.");
                            defmt::println!("Tip and roll it as well as turning it. Old correction kept.");
                        }
                        Err(FitError::Scattered) => {
                            defmt::println!("");
                            defmt::println!("NOT SOLVED - the samples are not on a sphere.");
                            defmt::println!("That usually means it barely moved. Sitting still,");
                            defmt::println!("the sensor's own noise makes a tiny ball that passes");
                            defmt::println!("every other check. Move it more. Old correction kept.");
                        }
                        Err(FitError::TooFewSamples) => {
                            defmt::println!("NOT SOLVED - too few samples. Old correction kept.");
                        }
                        Err(FitError::Singular) => {
                            defmt::println!("NOT SOLVED - the fit would not solve. Old correction kept.");
                        }
                    }
                    defmt::println!("");
                    mode = Mode::Compass;
                }
            }
        }

        delay.delay_ms(STEP_MS);
    }
}
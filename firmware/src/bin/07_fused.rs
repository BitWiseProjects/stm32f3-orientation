//! Stage 7 — three imperfect sensors covering for each other.
//!
//! Stage 6 stopped the board tipping over and left the heading sliding. That
//! was not a shortcut: gravity physically cannot see a flat spin, so no amount
//! of care with the accelerometer would have fixed it. It needs a second thing
//! that measures — one pointing in a fixed *horizontal* direction.
//!
//! ```text
//! cargo run --bin 07_fused
//! ```
//!
//! **PASS** — everything stage 6 already held onto, plus the one thing it
//! could not do:
//!
//! 1. Put the board down, hands off, leave it. The model **holds**.
//! 2. Tilt it — still fast, still smooth. Nothing was traded away.
//! 3. Spin it flat on the desk. The heading **comes back** rather than sliding.
//!    This is the test stage 6 fails, and the reason this stage exists.
//! 4. A magnet upsets the heading, briefly, because the magnetometer is an
//!    input again — stage 6 ignored one completely. It recovers when the
//!    magnet leaves.
//! 5. **Press the blue USER button, then tip, roll and turn the board — every
//!    way up, but keep it in one spot.** The ring becomes a progress bar, the
//!    browser fills with the samples as they are taken, and when it solves the
//!    board carries straight on with the offset it just measured.
//!
//!    Both halves of that instruction are load-bearing, and they fail
//!    differently. **Keep it flat and it never finishes** — flat samples lie in
//!    a plane, and a circle does not say where the centre of a sphere is, so
//!    the board refuses rather than inventing a number. **Carry it around the
//!    desk and it finishes, wrongly** — the field is not the same in two
//!    places, so the readings are not samples of one sphere and there is
//!    nothing true to find. The board can detect the first and cannot detect
//!    the second, which is what makes the second worth saying out loud.
//!
//!    Stage 3 has the whole argument, and did this on its own. What is new here
//!    is that it happens without ever leaving the fusion.
//!
//! # Why gravity was never going to be enough
//!
//! Stage 6 has the argument for gravity in full, and it still holds here
//! unchanged: trust the gyro for anything fast, and let the accelerometer pull
//! it slowly back toward reality.
//!
//! What that cannot touch is heading. When the board tips, the direction of
//! down changes *relative to the board* — there is something to correct
//! against. Spin the board flat and gravity does not move at all. The
//! accelerometer reads precisely the same thing at every heading, so it
//! physically cannot see that rotation, let alone fix it.
//!
//! What is needed is something pointing in a fixed *horizontal* direction —
//! which is stage 2, brought back as the third input.
//!
//! The maths for all of that is in the `attitude` crate, not in this file, and
//! it is the same code the browser is running. It also has unit tests, which
//! is a strange and wonderful thing to be able to say about firmware: `cargo
//! test` in `math/` proves gravity cannot fix yaw, on your laptop, in
//! milliseconds.
//!
//! # What still is not perfect
//!
//! The correction is proportional to the error and nothing else — there is no
//! running estimate of the gyro's bias. So a constant bias leaves a constant
//! error of a few degrees rather than none at all. It settles there and stays.
//! That is a completely different animal from drift, which never settles
//! anywhere, and closing the last few degrees is a bigger idea than this
//! project takes on.
//!
//! # The orientation packet is unchanged, and a second one joins it
//!
//! The orientation packet is byte for byte identical to stages 5 and 6. The
//! viewer cannot tell which of the three is running, which is the point — flash
//! one, then the next, and the only thing that changed is whether it holds
//! still.
//!
//! During a calibration run this stage also sends the **calibration packet**,
//! carrying one raw magnetometer sample and everything the fit knows at the
//! moment that sample was taken. Both are defined in the `packet` crate, which
//! the browser compiles too.
//!
//! **It goes out at 50 Hz, not once per tick, and the reason is the wire.** At
//! 115200 baud a byte costs 0.087 ms and this loop's tick is 5 ms. The 19-byte
//! orientation packet is 1.65 ms of that, which is comfortable. Adding 46 more
//! bytes to the same tick is 5.6 ms of writing into a 5 ms tick, and the write
//! blocks — so every fourth tick carries both, and the other three carry only
//! orientation.
//!
//! Even that fourth tick runs long, and the loop drops to about 193 Hz while a
//! run is going. Nothing downstream cares, because **`dt` is measured rather
//! than assumed** — the filter takes a slightly larger step and is otherwise
//! unaffected. It lasts for the thirty seconds of a deliberate mode.
//!
//! When no run is going the calibration packet still goes out, at about 2 Hz,
//! so the page can say which offset is in use without waiting for someone to
//! press a button.
//!
//! **Both ways a run can end are announced.** Solving sends one `Solved`
//! packet and a refusal sends one `Refused` packet, each at the moment it
//! happens, because the board is back to fusing on the very next tick and the
//! far end would otherwise see nothing but the idle heartbeat resuming. A
//! receiver must not read that resumption as failure — a solved packet losing
//! its checksum looks identical, and calling that a flat spin would be a
//! confident lie.
//!
//! # The magnetometer steps out of the fusion while it is being calibrated
//!
//! During a run the board is being tipped and rolled, and its magnetometer is
//! by definition not yet corrected. Feeding that into the filter would drag the
//! heading around exactly while nobody is looking at the model. So for those
//! thirty seconds the correction is dropped and this stage behaves like stage 6
//! — gravity only, heading free to slide. It picks the magnetometer back up the
//! instant the fit solves, with the offset the run just measured.

#![no_main]
#![no_std]

use attitude::{Attitude, Vec3};
use defmt_rtt as _;
use i3g4250d::{I3G4250D, Odr, Scale};
use lsm303agr::{AccelMode, AccelOutputDataRate, Lsm303agr, MagMode, MagOutputDataRate};
use magcal::{Calibration, Fit, FitError, GOOD_SAMPLES, GOOD_SPREAD, Solution};
use packet::ORIENTATION_LEN as PACKET_LEN;
use packet::{CALIBRATION_LEN, FitStatus, RunState};
use panic_probe as _;
use stm32f3xx_hal::delay::Delay;
use stm32f3xx_hal::hal::digital::v2::InputPin;
use stm32f3xx_hal::hal::spi::MODE_3;
use stm32f3xx_hal::i2c::I2c;
use stm32f3xx_hal::serial::Serial;
use stm32f3xx_hal::spi::{config::Config, Spi};
use stm32f3xx_hal::timer::{MonoTimer, Timer};
use stm32f3xx_hal::{self as hal, block, prelude::*};

const GYRO_ID: u8 = 0xD3;

/// See `02_compass.rs` for why these are the AGR's values and not the DLHC's.
const EXPECTED_MAG_ID: u8 = 0x40;
const EXPECTED_ACCEL_ID: u8 = 0x33;

const SAMPLE_HZ: u32 = 200;
const BAUD: u32 = 115_200;

const DEGREES_TO_RADIANS: f32 = core::f32::consts::PI / 180.0;

/// The hard iron this board carries, in nanotesla, in the **magnetometer's own
/// frame** — the frame [`mag_to_body`] takes its input in, not the one it hands
/// back.
///
/// **These are one particular board's numbers.** Stage 3 is where they come
/// from and where the argument for them lives; they are repeated here rather
/// than shared because each of these files is meant to be readable on its own,
/// and the one constant this half of the episode is about should not be off in
/// another file.
///
/// Zeros mean "no correction", which is what stage 6 was doing without saying
/// so.
const MAG_OFFSET_NT: [f32; 3] = [8062.6, -30600.0, -9042.9];

/// Send a calibration packet every this many ticks during a run — 200 Hz over
/// four is 50 Hz. See the note about the wire at the top of this file.
const CAL_EVERY_TICKS: u32 = 4;

/// And this often when there is no run, so the page knows which offset is in
/// use. 200 Hz over a hundred is 2 Hz.
const IDLE_CAL_EVERY_TICKS: u32 = 100;

/// How many consecutive reads of the same button level count as real. At 200 Hz
/// this is 15 ms, comfortably past the contact bounce and far below noticing.
const DEBOUNCE_STEPS: u8 = 3;

/// Ring blink half-period while the fit still needs more, in ticks.
const BLINK_TICKS: u32 = 32;

/// The shortest a calibration run is allowed to be, in seconds.
///
/// **Not a quality threshold — a human one.** `magcal` decides whether the
/// samples can be solved, and it is quite capable of saying yes after seven
/// seconds. But the instruction is *tip, roll and turn it*, which is two
/// separate motions, and seven seconds is not enough time to perform both. A
/// run that ends early ends having sampled whichever motion you started with,
/// which is how you get an answer that fits its own data perfectly and
/// disagrees with the last one.
///
/// So the board will not call itself finished before this, however good the fit
/// already looks. Pressing the button still stops it by hand at any point.
const MIN_RUN_SECONDS: u32 = 30;
const MIN_RUN_TICKS: u32 = MIN_RUN_SECONDS * SAMPLE_HZ;

/// The three sensors do not have to agree with each other about which way is
/// X, and neither of them has to agree with the board.
///
/// The fusion works in one frame — **X right, Y forward, Z up** — where an
/// identity orientation means the board is lying level and facing north. These
/// three functions are where each chip's own axes get turned into that, and
/// they are the only place in this program that knows about it.
///
/// They start as identity because that is the honest starting point, and they
/// are settled by looking at the board rather than by reasoning about package
/// drawings. With the board lying flat:
///
/// - `accel` should read about `(0, 0, +1)` g. If gravity turns up on a
///   different axis, or negative, this is where to fix it.
/// - Tilt the far edge up: `accel.y` should go negative, `accel.z` stays
///   positive.
/// - Turn the board clockwise seen from above: `gyro.z` should read negative,
///   because that is a clockwise turn about an axis pointing up.
/// - `mag` should have its largest horizontal component pointing at north.
///
/// Stages 2 and 4 print the raw numbers you need for all four checks.
fn gyro_to_body(v: Vec3) -> Vec3 {
    v
}

fn accel_to_body(v: Vec3) -> Vec3 {
    v
}

fn mag_to_body(v: Vec3) -> Vec3 {
    v
}

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

fn wrap_degrees(degrees: f32) -> f32 {
    let wrapped = libm::fmodf(degrees, 360.0);
    if wrapped < 0.0 { wrapped + 360.0 } else { wrapped }
}

fn led_bit(position: u32) -> u8 {
    1u8 << ((position + 1) % 8)
}

fn ring_pattern(degrees: f32) -> u8 {
    let sector = libm::roundf(degrees / 22.5) as i32;
    let sector = sector.rem_euclid(16);
    let position = (sector / 2) as u32;

    if sector % 2 == 0 {
        led_bit(position)
    } else {
        led_bit(position) | led_bit((position + 1) % 8)
    }
}

/// Pack an orientation into the nineteen bytes stage 5 describes.
///
/// The layout, the byte order and the checksum all live in the `packet` crate,
/// which the browser compiles too.
fn packet(orientation: attitude::Orientation) -> [u8; PACKET_LEN] {
    packet::Orientation {
        rotation: orientation,
    }
    .encode()
}

/// The ring as a bar: how much of a calibration run is done, in eighths.
///
/// Stage 3 has the reason this is a progress bar and not the coverage display
/// it started as — every bearing fills within half a second, so the coverage
/// ring was showing the easy half of the problem and then saying nothing.
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

/// A button, with the bouncing taken out. Stage 3 explains the debounce.
///
/// Reports the moment of pressing rather than the state, because "start
/// calibrating" should happen once per press and not two hundred times a second
/// for as long as a finger is resting on it.
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

    /// Feed it this tick's reading. `true` exactly once per press.
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
    /// Fusing all three sensors — the thing this stage is for.
    Fused,
    /// Collecting samples for a new offset. The magnetometer is out of the
    /// fusion for the duration; see the top of this file.
    Calibrating {
        fit: Fit,
        /// The tick the run began on, so it can be held open for
        /// [`MIN_RUN_SECONDS`].
        started: u32,
    },
}

/// The maths crate's error, as the byte that goes on the wire.
///
/// Two enums rather than one because `packet` describes the wire and nothing
/// else — a receiver can read the format without linking `magcal`. This is the
/// one place the two meet.
fn wire_status(error: FitError) -> FitStatus {
    match error {
        FitError::TooFewSamples => FitStatus::TooFewSamples,
        FitError::Coplanar => FitStatus::Coplanar,
        FitError::Singular => FitStatus::Singular,
        FitError::Scattered => FitStatus::Scattered,
    }
}

/// The one packet that says a run just finished, with the answer in it.
///
/// Sent once, immediately on solving. Without it [`RunState::Solved`] would
/// never appear on the wire at all — the board goes back to fusing on the same
/// tick it solves, so the next packet out is already an idle one, and the far
/// end would have to infer that anything had happened.
fn solved_packet(sample: Vec3, solution: &Solution) -> [u8; CALIBRATION_LEN] {
    packet::Calibration {
        sample,
        offset: solution.calibration.offset(),
        radius: solution.radius,
        residual: solution.residual,
        spread: solution.spread,
        samples: solution.samples,
        sectors: 0xFF,
        state: RunState::Solved,
        fit: FitStatus::Ok,
    }
    .encode()
}

/// The other end a run can come to, and for exactly the same reason.
///
/// A refusal used to be printed over the debug cable and nowhere else, so a
/// viewer saw collecting, collecting, then the 2 Hz idle heartbeat — and had to
/// read that silence as failure. It cannot. A solved packet that loses its
/// checksum produces the identical silence, so "back to idle" would report a
/// flat spin every time one frame got corrupted. Saying it outright costs 46
/// bytes, once.
///
/// The offset is the one still in use: a rejected fit is not adopted, and the
/// old correction stays. `radius` and `residual` are zero because nothing was
/// solved and a number there would imply otherwise — but `spread`, `samples`
/// and `sectors` are real, and they are what makes the refusal legible. A
/// spread of 0.002 against a floor of 0.05 *is* the explanation.
fn refused_packet(
    sample: Vec3,
    in_use: &Calibration,
    fit: &Fit,
    error: FitError,
) -> [u8; CALIBRATION_LEN] {
    packet::Calibration {
        sample,
        offset: in_use.offset(),
        radius: 0.0,
        residual: 0.0,
        spread: fit.spread(),
        samples: fit.samples(),
        sectors: fit.sectors(),
        state: RunState::Refused,
        fit: wire_status(error),
    }
    .encode()
}

/// Build the calibration packet for this tick.
///
/// `fit` is `None` when no run is going, and then the packet reports the offset
/// the read path is actually using rather than an estimate of a new one.
fn calibration_packet(
    sample: Vec3,
    in_use: &Calibration,
    fit: Option<&Fit>,
    solved: Option<&Solution>,
) -> [u8; CALIBRATION_LEN] {
    let mut out = packet::Calibration {
        sample,
        offset: in_use.offset(),
        radius: 0.0,
        residual: 0.0,
        spread: 0.0,
        samples: 0,
        sectors: 0,
        state: RunState::Idle,
        // Honest before anything has been solved: no samples have gone in, so
        // there is no radius and no residual to report, and the zeros above are
        // not being passed off as measurements.
        fit: FitStatus::TooFewSamples,
    };

    // Idle, but a run has happened — report what it found, since that is where
    // the offset in use came from.
    if let Some(solution) = solved {
        out.radius = solution.radius;
        out.residual = solution.residual;
        out.spread = solution.spread;
        out.samples = solution.samples;
        out.fit = FitStatus::Ok;
    }

    if let Some(fit) = fit {
        out.state = if fit.is_ready() {
            RunState::Ready
        } else {
            RunState::Collecting
        };
        out.samples = fit.samples();
        out.sectors = fit.sectors();
        out.spread = fit.spread();

        // Solved every packet rather than only at the end. That is what lets
        // the sphere on screen settle onto the cloud as the cloud fills in,
        // instead of appearing fully formed at the end — and it costs about
        // eighty microseconds, which at 50 Hz is nothing.
        //
        // `solve` and not `solve_for_use`: a mid-run estimate is allowed to be
        // poor, and refusing to show one would leave the screen empty for the
        // first several seconds of every run. The quality goes in the status
        // byte instead, so the far end gets both the settling sphere and an
        // honest word about whether it is worth anything yet.
        match fit.solve() {
            Ok(solution) => {
                out.offset = solution.calibration.offset();
                out.radius = solution.radius;
                out.residual = solution.residual;
                out.fit = if solution.is_believable() {
                    FitStatus::Ok
                } else {
                    FitStatus::Scattered
                };
            }
            Err(error) => {
                // The offset stays as the one in use, and the status says why
                // there is nothing better to show yet.
                out.radius = 0.0;
                out.residual = 0.0;
                out.fit = wire_status(error);
            }
        }
    }

    out.encode()
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let dp = hal::pac::Peripherals::take().unwrap();
    let mut cp = cortex_m::Peripherals::take().unwrap();

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
    let mut gpioc = dp.GPIOC.split(&mut rcc.ahb);
    let mut gpioe = dp.GPIOE.split(&mut rcc.ahb);

    gpioe.pe8.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    gpioe.pe9.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    gpioe.pe10.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    gpioe.pe11.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    gpioe.pe12.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    gpioe.pe13.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    gpioe.pe14.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    gpioe.pe15.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);

    // The blue USER button, which starts a calibration run. Pulled down,
    // because the board's button connects the pin to 3V3 — stage 3 has the
    // reasoning, and the fact that high means pressed here is a fact about this
    // schematic rather than about buttons.
    let button_pin = gpioa
        .pa0
        .into_pull_down_input(&mut gpioa.moder, &mut gpioa.pupdr);

    // --- the gyro, on SPI1 -------------------------------------------------

    let sck = gpioa
        .pa5
        .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
    let miso = gpioa
        .pa6
        .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
    let mosi = gpioa
        .pa7
        .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);

    let mut cs = gpioe
        .pe3
        .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    cs.set_high().ok();

    let spi: Spi<_, _, u8> = Spi::new(
        dp.SPI1,
        (sck, miso, mosi),
        Config::default().frequency(1_000_000.Hz()).mode(MODE_3),
        clocks,
        &mut rcc.apb2,
    );

    // --- the accelerometer and magnetometer, on I2C1 -----------------------

    let scl = gpiob
        .pb6
        .into_af_open_drain::<4>(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
    let sda = gpiob
        .pb7
        .into_af_open_drain::<4>(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);

    let i2c = I2c::new(dp.I2C1, (scl, sda), 100_000.Hz(), clocks, &mut rcc.apb1);
    let mut delay = Delay::new(cp.SYST, clocks);

    // --- the wire out ------------------------------------------------------

    let tx = gpioc
        .pc4
        .into_af_push_pull(&mut gpioc.moder, &mut gpioc.otyper, &mut gpioc.afrl);
    let rx = gpioc
        .pc5
        .into_af_push_pull(&mut gpioc.moder, &mut gpioc.otyper, &mut gpioc.afrl);

    let mut serial = Serial::new(dp.USART1, (tx, rx), BAUD.Bd(), clocks, &mut rcc.apb2);

    defmt::println!("stage 7 - all of it");
    defmt::println!("===================");

    // All three chips get asked who they are, for the same reason as before.
    let mut compass = Lsm303agr::new_with_i2c(i2c);
    let mag_id = compass.magnetometer_id().unwrap();
    let accel_id = compass.accelerometer_id().unwrap();

    let mut gyro = I3G4250D::new(spi, cs).unwrap();
    let gyro_id = gyro.who_am_i().unwrap();

    defmt::println!("gyroscope     = {:#04x} (expect {:#04x})", gyro_id, GYRO_ID);
    defmt::println!("magnetometer  = {:#04x} (expect {:#04x})", mag_id.raw(), EXPECTED_MAG_ID);
    defmt::println!("accelerometer = {:#04x} (expect {:#04x})", accel_id.raw(), EXPECTED_ACCEL_ID);
    if gyro_id != GYRO_ID || mag_id.raw() != EXPECTED_MAG_ID || accel_id.raw() != EXPECTED_ACCEL_ID {
        defmt::println!("!! a sensor did not answer correctly - fix that before reading on");
    }

    compass.init().unwrap();
    compass
        .set_accel_mode_and_odr(&mut delay, AccelMode::HighResolution, AccelOutputDataRate::Hz100)
        .unwrap();
    compass
        .set_mag_mode_and_odr(&mut delay, MagMode::HighResolution, MagOutputDataRate::Hz100)
        .unwrap();
    let mut compass = compass.into_mag_continuous().ok().unwrap();

    gyro.set_scale(Scale::Dps500).unwrap();
    gyro.set_odr(Odr::Hz200).unwrap();
    let scale = gyro.scale().unwrap();

    let mono = MonoTimer::new(cp.DWT, clocks, &mut cp.DCB);
    let cycles_per_second = mono.frequency().0 as f32;

    let mut tick = Timer::new(dp.TIM2, clocks, &mut rcc.apb1);
    tick.start((1_000 / SAMPLE_HZ).milliseconds());

    let mut estimate = Attitude::new();
    defmt::println!("");
    defmt::println!(
        "gains: gravity {=f32}, north {=f32}",
        estimate.gains().accel,
        estimate.gains().mag
    );
    let mut calibration = Calibration::from_offset(Vec3::from_array(MAG_OFFSET_NT));
    if calibration.offset() == Vec3::ZERO {
        defmt::println!("no correction compiled in - the heading will be wrong, see stage 3");
    } else {
        let o = calibration.offset();
        defmt::println!("hard iron {=f32} {=f32} {=f32} nT (one board's, see stage 3)", o.x, o.y, o.z);
    }
    defmt::println!("");
    defmt::println!("Tilt it, spin it, put it down and walk away.");
    defmt::println!("To recalibrate: press the blue USER button, then tip and roll it");
    defmt::println!("every way up - but keep it in one spot.");
    defmt::println!("");

    let mut last = mono.now();
    let mut ticks: u32 = 0;
    let mut mode = Mode::Fused;
    let mut button = Button::new(button_pin.is_high().unwrap_or(false));
    let mut solved: Option<Solution> = None;

    loop {
        let _ = block!(tick.wait());

        let dt = last.elapsed() as f32 / cycles_per_second;
        last = mono.now();

        let raw_gyro = gyro.gyro().unwrap();
        let rate = gyro_to_body(Vec3::new(
            scale.degrees(raw_gyro.x),
            scale.degrees(raw_gyro.y),
            scale.degrees(raw_gyro.z),
        )) * DEGREES_TO_RADIANS;

        // Only the direction of these two matters, so the units cancel — and
        // unlike the DLHC this chip's driver hands back real units (milli-g,
        // nanotesla) on every axis already, so there is no per-axis
        // sensitivity to divide out by hand.
        let raw_accel = compass.acceleration().unwrap();
        let gravity = accel_to_body(Vec3::new(
            raw_accel.x_mg() as f32,
            raw_accel.y_mg() as f32,
            raw_accel.z_mg() as f32,
        ));

        // In the sensor's own frame, because that is the frame the hard iron
        // was measured in and the frame it is fixed in. Correct first, map to
        // the body frame second — doing it the other way round would be wrong
        // the moment `mag_to_body` stops being the identity.
        //
        // **Ask first whether there is anything new.** This loop runs at 200 Hz
        // and the magnetometer produces data at 100, so a read every tick
        // returns each measurement about twice. That is harmless for the
        // filter, which is only being told the same thing twice — but it is not
        // harmless for the fit, where a repeated reading is not a second
        // opinion and counting it as one makes a run finish in half the real
        // time it needs. Measured, not reasoned: the first run on this board
        // hit its 500-sample threshold in 2.5 seconds, before the board had
        // been moved at all.
        let mag_is_new = compass
            .mag_status()
            .map(|status| status.xyz_new_data())
            .unwrap_or(false);

        let raw_mag = compass.magnetic_field().unwrap();
        let sensor_mag = Vec3::new(
            raw_mag.x_nt() as f32,
            raw_mag.y_nt() as f32,
            raw_mag.z_nt() as f32,
        );
        let north = mag_to_body(calibration.apply(sensor_mag));

        let press = button.pressed(button_pin.is_high().unwrap_or(false));
        ticks = ticks.wrapping_add(1);

        let calibrating = matches!(mode, Mode::Calibrating { .. });

        // One line, three sensors. Gyro for speed, gravity for which way is
        // down, north for which way is forward — except during a run, when the
        // magnetometer is the thing being measured and steps out.
        estimate.update(
            rate,
            Some(gravity),
            if calibrating { None } else { Some(north) },
            dt,
        );

        let orientation = estimate.orientation();
        serial.bwrite_all(&packet(orientation)).ok();

        let forward = orientation * Vec3::Y;
        let heading = wrap_degrees(libm::atan2f(forward.x, forward.y).to_degrees());

        match &mut mode {
            Mode::Fused => {
                show(ring_pattern(heading));

                if ticks % IDLE_CAL_EVERY_TICKS == 0 {
                    let bytes =
                        calibration_packet(sensor_mag, &calibration, None, solved.as_ref());
                    serial.bwrite_all(&bytes).ok();
                }

                if ticks % SAMPLE_HZ == 0 {
                    let up = orientation * Vec3::Z;
                    defmt::println!(
                        "heading {=f32} deg   tilt {=f32} deg   after {} s",
                        heading,
                        up.angle_between(Vec3::Z).to_degrees(),
                        ticks / SAMPLE_HZ
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
                //
                // And only when the sensor has actually produced something —
                // see the note where `mag_is_new` is read.
                if mag_is_new {
                    fit.push(sensor_mag);
                }

                let elapsed = ticks.wrapping_sub(*started);
                let long_enough = elapsed >= MIN_RUN_TICKS;
                let ready = long_enough && fit.is_ready();

                // Whichever requirement is furthest from being met is the one
                // worth showing, so the bar never promises more than the run
                // has actually got. The clock is one of those requirements —
                // leaving it out would fill the bar in seven seconds and then
                // leave it sitting full for another twenty-three, which reads
                // as a hang rather than as progress.
                let by_samples = fit.samples() as f32 / GOOD_SAMPLES as f32;
                let by_spread = fit.spread() / GOOD_SPREAD;
                let by_clock = elapsed as f32 / MIN_RUN_TICKS as f32;
                let lit = progress_pattern(by_samples.min(by_spread).min(by_clock));

                let blink_off = !ready && (ticks / BLINK_TICKS) % 2 == 0;
                show(if blink_off { 0 } else { lit });

                if ticks % CAL_EVERY_TICKS == 0 {
                    let bytes =
                        calibration_packet(sensor_mag, &calibration, Some(fit), solved.as_ref());
                    serial.bwrite_all(&bytes).ok();
                }

                if ready || press {
                    if press && !ready {
                        defmt::println!("stopped by hand");
                    }
                    // `solve_for_use`, because this is the line that replaces
                    // the correction the board reads with. Stopping by hand is
                    // allowed to skip the *sample count* and *spread* the
                    // automatic finish waits for — that is what a manual stop
                    // is for — but it is not allowed to skip the check that the
                    // answer describes the samples at all.
                    match fit.solve_for_use() {
                        Ok(solution) => {
                            let o = solution.calibration.offset();
                            calibration = solution.calibration;
                            serial.bwrite_all(&solved_packet(sensor_mag, &solution)).ok();
                            solved = Some(solution);
                            defmt::println!("");
                            defmt::println!("SOLVED from {=u32} samples", solution.samples);
                            defmt::println!("  offset    {=f32} {=f32} {=f32} nT", o.x, o.y, o.z);
                            defmt::println!("  field     {=f32} nT", solution.radius);
                            defmt::println!(
                                "  scatter   {=f32} nT ({=f32} of the field)",
                                solution.residual,
                                solution.residual / solution.radius
                            );
                            defmt::println!("");
                            defmt::println!("in use now. to make it the default, paste into BOTH");
                            defmt::println!("03_calibrate.rs and 07_fused.rs:");
                            defmt::println!(
                                "  const MAG_OFFSET_NT: [f32; 3] = [{=f32}, {=f32}, {=f32}];",
                                o.x,
                                o.y,
                                o.z
                            );
                        }
                        Err(error) => {
                            // On the wire first, then to the terminal. The two
                            // used to disagree about whether anything had
                            // happened — defmt was told and the viewer was not.
                            serial
                                .bwrite_all(&refused_packet(sensor_mag, &calibration, fit, error))
                                .ok();

                            match error {
                                FitError::Coplanar => {
                                    defmt::println!("");
                                    defmt::println!("NOT SOLVED - the samples are flat.");
                                    defmt::println!(
                                        "A circle does not say where the centre of a sphere is."
                                    );
                                    defmt::println!(
                                        "Tip and roll it as well as turning it. Old correction kept."
                                    );
                                }
                                FitError::Scattered => {
                                    defmt::println!("");
                                    defmt::println!(
                                        "NOT SOLVED - the samples are not on a sphere."
                                    );
                                    defmt::println!(
                                        "That usually means it barely moved. Sitting still, the"
                                    );
                                    defmt::println!(
                                        "sensor's own noise makes a tiny ball that passes every"
                                    );
                                    defmt::println!(
                                        "other check. Move it more. Old correction kept."
                                    );
                                }
                                FitError::TooFewSamples => {
                                    defmt::println!(
                                        "NOT SOLVED - too few samples. Old correction kept."
                                    );
                                }
                                FitError::Singular => {
                                    defmt::println!(
                                        "NOT SOLVED - the fit would not solve. Old correction kept."
                                    );
                                }
                            }
                        }
                    }
                    defmt::println!("");
                    mode = Mode::Fused;
                }
            }
        }
    }
}

//! Stage 6 — the first fix: gravity.
//!
//! Stage 5 drifts, and no amount of care with the gyro will stop it. The fix
//! is not a better gyro. The fix is something that **measures** rather than
//! accumulates, because a measurement can be wrong but cannot get
//! progressively wronger.
//!
//! There is exactly one such thing already on the board and already on a bus
//! this program knows how to drive. The accelerometer sits in the same package
//! as stage 2's magnetometer, on the same I2C pins, at its own address — stage
//! 2 already asked it who it was and then never read another byte from it.
//!
//! ```text
//! cargo run --bin 06_gravity
//! ```
//!
//! Then open `viewer/` in Chrome and press Connect, exactly as in stage 5.
//!
//! **PASS**, in four parts:
//!
//! 1. **Put the board down, hands off, leave it a minute.** It stops tipping
//!    over. Whatever tilt error it has settles at some small value and stays
//!    there — which is a completely different thing from stage 5, where it
//!    never settled anywhere.
//! 2. Tilt it — still fast, still smooth. Nothing was traded away.
//! 3. **Spin it flat on the desk and leave it.** The heading still slides.
//!    That is not a bug in this file and there is nothing here to fix; the
//!    next section says why, and stage 7 is where it gets fixed.
//! 4. **Wave a magnet at it. Nothing happens.** Stage 2's heading jumped when
//!    you did that. This program never reads the magnetometer, so a magnet is
//!    just a lump of metal — which is also the check that says any problem in
//!    tests 1 to 3 is a problem with *this* stage rather than with the compass.
//!
//! # Why gravity fixes tilt
//!
//! The accelerometer measures acceleration, and a board sitting still on a
//! desk feels exactly one: gravity, straight down, one g, never drifting. So
//! it always knows which way down is — but it is the mirror image of the gyro.
//! The moment the board actually moves, your hand's acceleration is mixed in
//! with gravity and the reading is rubbish. It is reliable exactly when
//! nothing is happening.
//!
//! So: trust the gyro for anything fast, and let the accelerometer pull it
//! slowly back toward reality. That is the whole of sensor fusion, and it is
//! one argument to `update` rather than a new algorithm.
//!
//! # Why gravity cannot fix heading
//!
//! When the board tips, the direction of down changes *relative to the board*
//! — there is something to correct against. Spin the board flat and gravity
//! does not move at all. The accelerometer reads precisely the same thing at
//! every heading, so it physically cannot see that rotation, let alone fix it.
//!
//! This is worth being clear about because it is the one failure in the PASS
//! list above, and it is a fact about the physics rather than about this code.
//! Turning a gain up would not touch it. `cargo test` in `math/` proves it on
//! your laptop in about a millisecond.
//!
//! # What this stage deliberately does not do
//!
//! It does not read the magnetometer, initialise it, or even ask it who it is.
//! The whole difference between this file and stage 7 is one sensor, and
//! leaving the plumbing out as well as the reading is what keeps that true.
//!
//! # The packet is unchanged
//!
//! Byte for byte identical to stage 5. Flash one, then the other, and the only
//! thing that changed is whether it stays level.

#![no_main]
#![no_std]

use attitude::{Attitude, Vec3};
use defmt_rtt as _;
use i3g4250d::{I3G4250D, Odr, Scale};
use lsm303agr::{AccelMode, AccelOutputDataRate, Lsm303agr};
use packet::ORIENTATION_LEN as PACKET_LEN;
use panic_probe as _;
use stm32f3xx_hal::delay::Delay;
use stm32f3xx_hal::hal::spi::MODE_3;
use stm32f3xx_hal::i2c::I2c;
use stm32f3xx_hal::serial::Serial;
use stm32f3xx_hal::spi::{config::Config, Spi};
use stm32f3xx_hal::timer::{MonoTimer, Timer};
use stm32f3xx_hal::{self as hal, block, prelude::*};

const GYRO_ID: u8 = 0xD3;

/// See `02_compass.rs` for why this is the AGR's value and not the DLHC's.
const EXPECTED_ACCEL_ID: u8 = 0x33;

const SAMPLE_HZ: u32 = 200;
const BAUD: u32 = 115_200;

const DEGREES_TO_RADIANS: f32 = core::f32::consts::PI / 180.0;

/// The two sensors do not have to agree with each other about which way is X,
/// and neither of them has to agree with the board.
///
/// The fusion works in one frame — **X right, Y forward, Z up** — where an
/// identity orientation means the board is lying level and facing north. These
/// two functions are where each chip's own axes get turned into that, and they
/// are the only place in this program that knows about it.
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
///
/// **Get this wrong and the failure does not look like a swapped axis.** It
/// looks like the fusion being broken: the model gets pulled toward a "down"
/// that is not down, slowly and confidently. Stages 2 and 4 print the raw
/// numbers you need for all three checks, so do them there before blaming
/// anything here.
fn gyro_to_body(v: Vec3) -> Vec3 {
    v
}

fn accel_to_body(v: Vec3) -> Vec3 {
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

    // --- the accelerometer, on I2C1 ----------------------------------------
    //
    // The same two pins and the same package stage 2 used. The magnetometer is
    // sitting right there on this bus and this program never speaks to it.

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

    defmt::println!("stage 6 - the first fix");
    defmt::println!("=======================");

    // Both chips get asked who they are, for the same reason as before.
    let mut sensors = Lsm303agr::new_with_i2c(i2c);
    let accel_id = sensors.accelerometer_id().unwrap();

    let mut gyro = I3G4250D::new(spi, cs).unwrap();
    let gyro_id = gyro.who_am_i().unwrap();

    defmt::println!("gyroscope     = {:#04x} (expect {:#04x})", gyro_id, GYRO_ID);
    defmt::println!("accelerometer = {:#04x} (expect {:#04x})", accel_id.raw(), EXPECTED_ACCEL_ID);
    if gyro_id != GYRO_ID || accel_id.raw() != EXPECTED_ACCEL_ID {
        defmt::println!("!! a sensor did not answer correctly - fix that before reading on");
    }

    sensors.init().unwrap();
    sensors
        .set_accel_mode_and_odr(&mut delay, AccelMode::HighResolution, AccelOutputDataRate::Hz100)
        .unwrap();

    gyro.set_scale(Scale::Dps500).unwrap();
    gyro.set_odr(Odr::Hz200).unwrap();
    let scale = gyro.scale().unwrap();

    let mono = MonoTimer::new(cp.DWT, clocks, &mut cp.DCB);
    let cycles_per_second = mono.frequency().0 as f32;

    let mut tick = Timer::new(dp.TIM2, clocks, &mut rcc.apb1);
    tick.start((1_000 / SAMPLE_HZ).milliseconds());

    let mut estimate = Attitude::new();
    defmt::println!("");
    defmt::println!("gain: gravity {=f32} (no north - that is stage 7)", estimate.gains().accel);
    defmt::println!("");
    defmt::println!("Put it down and it stops tipping over. Spin it flat and the heading still goes.");
    defmt::println!("");

    let mut last = mono.now();
    let mut ticks: u32 = 0;

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

        // Only the direction of this matters, so the units cancel — and unlike
        // the DLHC this chip's driver hands back real units (milli-g) on every
        // axis already, so there is no per-axis sensitivity to divide out by
        // hand.
        let raw_accel = sensors.acceleration().unwrap();
        let gravity = accel_to_body(Vec3::new(
            raw_accel.x_mg() as f32,
            raw_accel.y_mg() as f32,
            raw_accel.z_mg() as f32,
        ));

        // One line, two sensors. Gyro for speed, gravity for which way is down.
        // The `None` is the magnetometer's place, left empty on purpose.
        estimate.update(rate, Some(gravity), None, dt);

        let orientation = estimate.orientation();
        serial.bwrite_all(&packet(orientation)).ok();

        // The ring still shows heading, and heading is still the thing this
        // stage cannot fix — so expect it to walk. That is the honest display.
        let forward = orientation * Vec3::Y;
        let heading = wrap_degrees(libm::atan2f(forward.x, forward.y).to_degrees());
        show(ring_pattern(heading));

        ticks = ticks.wrapping_add(1);
        if ticks % SAMPLE_HZ == 0 {
            let up = orientation * Vec3::Z;
            defmt::println!(
                "heading {=f32} deg   tilt {=f32} deg   after {} s",
                heading,
                up.angle_between(Vec3::Z).to_degrees(),
                ticks / SAMPLE_HZ
            );
        }
    }
}

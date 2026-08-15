//! Stage 2 — a sensor that *measures*.
//!
//! The ring can display anything now, so give it something worth displaying.
//! The LSM303AGR on this board contains a magnetometer: it measures the
//! magnetic field it is sitting in, on three axes, and the earth's field is a
//! big part of that. Take the two horizontal components, do a bit of
//! trigonometry, and you have a heading.
//!
//! ```text
//! cargo run --bin 02_compass
//! ```
//!
//! **PASS** = turn the board slowly through a full circle, and the lit LED
//! **stays pointing the same way in the room**. The board moves under it. That
//! is the whole idea, and it is much more convincing than any number.
//!
//! Then bring a magnet near it and watch the heading go wherever the magnet
//! says. It measures *magnetic field*, not north.
//!
//! # Three new things
//!
//! **The bus.** I2C: two wires, shared, every device has an address. This one
//! package answers to two of them — `0x19` for the accelerometer and `0x1E` for
//! the magnetometer — because it is really two sensors that happen to share a
//! lid.
//!
//! **Asking who it is.** The first transaction is always a read of the
//! identification register, before anything else. If it comes back wrong then
//! the wiring, the address or the part is not what you think, and nothing you
//! do afterwards means anything.
//!
//! **This is not a hypothetical.** The first version of this file asked the
//! wrong part. Every STM32F3 Discovery tutorial says the e-compass is an
//! **LSM303DLHC**, and this board carries an **LSM303AGR**. They share both
//! addresses. Their accelerometers are compatible enough that DLHC code drives
//! an AGR's accelerometer correctly and you would never know. Their
//! magnetometers are unrelated parts, so the DLHC driver read registers that do
//! not exist here and got back a tidy, plausible, entirely fictional zero.
//!
//! What caught it was exactly the check in this file: an identification
//! register that is supposed to be a fixed constant, read back as `0x00`.
//!
//! # About the datasheet lookup
//!
//! The gyro in stage 4 has a full-scale range you choose, and choosing it is
//! what makes its raw counts mean something. **This magnetometer does not.**
//! The AGR has one fixed range, and one fixed sensitivity: 1.5 milligauss per
//! count, or 150 nanotesla, which is why the driver hands back nanotesla
//! directly instead of raw integers. There is nothing to select — but you still
//! have to go and find that number, and it is still the only thing standing
//! between an integer and a measurement.
//!
//! # The two constants at the bottom of the bench
//!
//! [`HEADING_SENSE`] and [`HEADING_OFFSET_DEG`] describe how the sensor's axes
//! are turned relative to the ring, and they are settled by looking at the
//! board rather than by reasoning. Turn the board slowly clockwise:
//!
//! - the lit LED holds still → both are right;
//! - it turns the *same* way as the board, at double speed → flip
//!   [`HEADING_SENSE`];
//! - it holds still relative to the room but points somewhere other than north
//!   → add the difference to [`HEADING_OFFSET_DEG`].

#![no_main]
#![no_std]

use defmt_rtt as _;
use lsm303agr::{Lsm303agr, MagMode, MagOutputDataRate};
use panic_probe as _;
use stm32f3xx_hal::delay::Delay;
use stm32f3xx_hal::hal::blocking::delay::DelayMs;
use stm32f3xx_hal::i2c::I2c;
use stm32f3xx_hal::{self as hal, prelude::*};

/// What the magnetometer's `WHO_AM_I` must read on an LSM303AGR.
const EXPECTED_MAG_ID: u8 = 0x40;

/// And the accelerometer's, which is a separate device on a separate address.
const EXPECTED_ACCEL_ID: u8 = 0x33;

/// Which way the heading runs. `-1.0` if the ring turns with the board
/// instead of against it — see the note at the top of this file.
const HEADING_SENSE: f32 = 1.0;

/// Rotation between the sensor's idea of zero and the ring's North.
const HEADING_OFFSET_DEG: f32 = 0.0;

/// Display update interval. The magnetometer is configured for 50 Hz below,
/// so this is not throwing readings away.
const STEP_MS: u16 = 20;

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

    defmt::println!("stage 2 - the compass");
    defmt::println!("=====================");

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

    defmt::println!("");
    defmt::println!("Turn the board slowly. The lit LED should stay put.");
    defmt::println!("");

    let mut ticks: u32 = 0;
    loop {
        let field = compass.magnetic_field().unwrap();

        // Nanotesla, straight from the driver — 150 of them per count, which is
        // the one number this sensor's datasheet exists to tell you. The earth
        // manages about 50,000 nT, so expect tens of thousands here.
        let x = field.x_nt() as f32;
        let y = field.y_nt() as f32;

        // atan2 rather than atan, because atan cannot tell which quadrant it is
        // in and would fold the circle in half.
        let angle = libm::atan2f(y, x).to_degrees();
        let heading = wrap_degrees(angle * HEADING_SENSE + HEADING_OFFSET_DEG);

        show(ring_pattern(heading));

        // Once a second, so the terminal stays readable while the ring runs at
        // fifty times that.
        ticks = ticks.wrapping_add(1);
        if ticks % 50 == 0 {
            defmt::println!(
                "field {=i32} {=i32} {=i32} nT   heading {=f32} deg",
                field.x_nt(),
                field.y_nt(),
                field.z_nt(),
                heading
            );
        }

        delay.delay_ms(STEP_MS);
    }
}

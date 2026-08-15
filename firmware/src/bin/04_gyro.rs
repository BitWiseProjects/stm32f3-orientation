//! Stage 4 — a sensor that *accumulates*.
//!
//! The second sensor is a gyroscope, and it works in a completely different
//! way from the magnetometer. The magnetometer told us where the board is
//! pointing. The gyro cannot answer that question at all. All it reports is
//! **how fast the board is turning right now**, in degrees per second. Hold it
//! still and it reads zero, whichever way it is facing.
//!
//! So to get an angle out of it, every rate has to be multiplied by how long
//! it lasted and added to a running total. Turning at ten degrees a second for
//! two seconds is twenty degrees. Do that continuously and you are
//! **integrating** — building up a position out of a stream of speeds.
//!
//! ```text
//! cargo run --bin 04_gyro
//! ```
//!
//! **PASS** = turn the board and the ring follows, smoothly and immediately.
//! Then bring the magnet back and watch **nothing happen** — this sensor is
//! not measuring magnetic anything.
//!
//! It feels better than the compass in every way you can see. That is the
//! setup, and stage 5 is the punchline: put it down, leave it alone, and come
//! back in a minute. The heading will have wandered off on its own, because
//! every error in every sample it ever took is still in that running total.
//!
//! # Two new things
//!
//! **A different bus.** SPI: four wires instead of two, faster, with a
//! dedicated chip-select line rather than an address. Same first move though —
//! ask the chip who it is. This one should answer `0xD3`.
//!
//! **Time has to be real.** If the code thinks a millisecond passed and two
//! actually did, the angle is wrong by double, and being wrong by double
//! *accumulates* too. So the loop is paced by a hardware timer, and the
//! interval fed to the integration is *measured* by a cycle counter rather
//! than assumed — because a timer that has already expired lets the loop run
//! late without ever saying so.
//!
//! # The reading this gives you is not a heading
//!
//! It starts at zero wherever the board happened to be lying, and counts turn
//! from there. That is all a gyro can offer.

#![no_main]
#![no_std]

use defmt_rtt as _;
use i3g4250d::{I3G4250D, Odr, Scale};
use panic_probe as _;
use stm32f3xx_hal::hal::spi::MODE_3;
use stm32f3xx_hal::spi::{config::Config, Spi};
use stm32f3xx_hal::timer::{MonoTimer, Timer};
use stm32f3xx_hal::{self as hal, block, prelude::*};

/// Identification value for the I3G4250D. The very similar L3GD20 answers
/// `0xD4`, and its driver would happily return numbers from this part with
/// every rate and scale label a few percent wrong.
const EXPECTED_ID: u8 = 0xD3;

/// Loop rate, matched to the data rate the sensor is configured for below.
/// Sampling faster would just read the same measurement twice.
const SAMPLE_HZ: u32 = 200;

/// Which way a positive rate turns the ring. Flip to `-1.0` if the display
/// runs backwards when you turn the board — see stage 2's note; this is the
/// same kind of constant, settled the same way.
const GYRO_SENSE: f32 = 1.0;

/// Ignore rates smaller than this before integrating them.
///
/// This is a **deliberately bad idea**, left switched off at `0.0`. Squashing
/// small rates does hide the drift, and it also means the board can be turned
/// slowly enough to register nothing at all. Stage 6 fixes drift properly.
const DEADBAND_DPS: f32 = 0.0;

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
fn wrap_degrees(degrees: f32) -> f32 {
    let wrapped = libm::fmodf(degrees, 360.0);
    if wrapped < 0.0 { wrapped + 360.0 } else { wrapped }
}

/// Ring position — 0 is North, counting clockwise — to its bit in the byte.
fn led_bit(position: u32) -> u8 {
    1u8 << ((position + 1) % 8)
}

/// The byte that points at an angle: one LED on it, two either side of it.
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
    let mut gpioe = dp.GPIOE.split(&mut rcc.ahb);

    gpioe.pe8.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    gpioe.pe9.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    gpioe.pe10.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    gpioe.pe11.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    gpioe.pe12.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    gpioe.pe13.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    gpioe.pe14.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    gpioe.pe15.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);

    let sck = gpioa
        .pa5
        .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
    let miso = gpioa
        .pa6
        .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
    let mosi = gpioa
        .pa7
        .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);

    // Chip select idles high — the gyro only listens while this is pulled low.
    // PE3 is not one of the eight ring LEDs.
    let mut cs = gpioe
        .pe3
        .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    cs.set_high().ok();

    // Mode 3: clock idles high, data sampled on the rising edge. This HAL
    // defaults to mode 0, and mode 0 against this part returns plausible
    // nonsense rather than an error.
    let spi: Spi<_, _, u8> = Spi::new(
        dp.SPI1,
        (sck, miso, mosi),
        Config::default().frequency(1_000_000.Hz()).mode(MODE_3),
        clocks,
        &mut rcc.apb2,
    );

    let mut gyro = I3G4250D::new(spi, cs).unwrap();

    defmt::println!("stage 4 - the gyro");
    defmt::println!("==================");

    let id = gyro.who_am_i().unwrap();
    defmt::println!("identification = {:#04x} (expect {:#04x})", id, EXPECTED_ID);
    if id != EXPECTED_ID {
        defmt::println!("!! wrong answer - stop here, nothing below this line means anything");
    }

    // ±500 degrees per second. Fast enough that turning the board by hand
    // never runs off the end of the scale, slow enough that the resolution is
    // still worth having.
    gyro.set_scale(Scale::Dps500).unwrap();
    gyro.set_odr(Odr::Hz200).unwrap();

    // Read the configuration back off the chip rather than assuming it took,
    // so the degrees-per-second below have a stated basis.
    let scale = gyro.scale().unwrap();

    // The cycle counter, for measuring how long a pass round the loop really
    // took. `MonoTimer` counts core clock cycles in hardware.
    let mono = MonoTimer::new(cp.DWT, clocks, &mut cp.DCB);
    let cycles_per_second = mono.frequency().0 as f32;

    // And a separate timer to pace the loop, so it runs at the sensor's rate
    // rather than as fast as the chip can ask.
    let mut tick = Timer::new(dp.TIM2, clocks, &mut rcc.apb1);
    tick.start((1_000 / SAMPLE_HZ).milliseconds());

    defmt::println!("scale {}", defmt::Debug2Format(&scale));
    defmt::println!("sampling at {} Hz, {} cycles per second", SAMPLE_HZ, cycles_per_second);
    defmt::println!("");
    defmt::println!("Turn the board - the ring follows. A magnet does nothing.");
    defmt::println!("Then put it down and watch where it goes on its own.");
    defmt::println!("");

    let mut heading = 0.0f32;
    let mut last = mono.now();
    let mut ticks: u32 = 0;

    loop {
        let _ = block!(tick.wait());

        // How long that actually took, in seconds. Measured, not assumed.
        let dt = last.elapsed() as f32 / cycles_per_second;
        last = mono.now();

        let rate_dps = scale.degrees(gyro.gyro().unwrap().z);
        let rate_dps = if libm::fabsf(rate_dps) < DEADBAND_DPS {
            0.0
        } else {
            rate_dps
        };

        // The whole of integration, on one line: rate times elapsed time,
        // added to what we had. Everything hard about it is in the fact that
        // this line also adds every error, forever.
        heading = wrap_degrees(heading + rate_dps * dt * GYRO_SENSE);

        show(ring_pattern(heading));

        ticks = ticks.wrapping_add(1);
        if ticks % SAMPLE_HZ == 0 {
            defmt::println!("rate {=f32} dps   accumulated {=f32} deg", rate_dps, heading);
        }
    }
}

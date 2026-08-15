//! Stage 5 — off the board, onto a screen.
//!
//! Eight LEDs can show one angle. The board turns in three dimensions, and to
//! see all of it the numbers have to leave the chip. So this stage does the
//! same integration as stage 4, in all three axes rather than one, and sends
//! the result out of the serial port continuously.
//!
//! ```text
//! cargo run --bin 05_serial
//! ```
//!
//! Then open `viewer/` in Chrome and press Connect.
//!
//! **PASS**, in two parts:
//!
//! 1. Turn the board and the model on screen turns with it, immediately and
//!    smoothly.
//! 2. **Put the board down, take your hands off, and leave it.** Come back in
//!    a minute. The board has not moved. The model has.
//!
//! That second part is not a bug and there is nothing to fix in this file. It
//! is what integration does, and seeing it clearly is the entire reason this
//! stage exists as its own program. Stage 6 is half the fix and stage 7 is the
//! rest, because the drift has two halves and one sensor can only reach one of
//! them.
//!
//! # Why it drifts
//!
//! The gyro does not read exactly zero when it is still. It is off by a tiny
//! amount — noise, temperature, how it happened to be manufactured — some
//! fraction of a degree per second. Nothing.
//!
//! But this loop adds it up, two hundred times a second, forever, and nothing
//! anywhere in it ever says "you are wrong, come back". Every error it makes,
//! it keeps. Half a degree per second is thirty degrees a minute.
//!
//! # The packet
//!
//! Nineteen bytes, sent as fast as the sensor produces measurements:
//!
//! ```text
//!   0xAA 0x55 | w:f32 | x:f32 | y:f32 | z:f32 | crc8
//!   \_______/   \_______________________/       \__/
//!    marker            orientation            checksum
//! ```
//!
//! Floats are little-endian. The marker is there so the receiver can find the
//! start of a packet in a stream it joined halfway through — which it always
//! does, because the board was already running before the browser connected.
//! The checksum is there so a packet that arrived mangled gets thrown away
//! instead of drawn.
//!
//! **Stages 6 and 7 send this exact same packet.** Nothing downstream can tell
//! the three apart, which is what makes the comparison in Act 5 honest: the
//! only thing that changes between the drifting model and the steady one is
//! which binary is flashed.
//!
//! Those nineteen bytes are laid out in the `packet` crate, not here — and the
//! browser at the far end compiles that same file. A wire format is an
//! agreement between two programs, and the only way to be sure they agree is
//! to give them one copy of it to share.

#![no_main]
#![no_std]

use attitude::{Attitude, Vec3};
use defmt_rtt as _;
use i3g4250d::{I3G4250D, Odr, Scale};
use packet::ORIENTATION_LEN as PACKET_LEN;
use panic_probe as _;
use stm32f3xx_hal::hal::spi::MODE_3;
use stm32f3xx_hal::serial::Serial;
use stm32f3xx_hal::spi::{config::Config, Spi};
use stm32f3xx_hal::timer::{MonoTimer, Timer};
use stm32f3xx_hal::{self as hal, block, prelude::*};

const EXPECTED_ID: u8 = 0xD3;
const SAMPLE_HZ: u32 = 200;
const BAUD: u32 = 115_200;


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

/// Pack an orientation into the nineteen bytes described at the top of this file.
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

    // USART1 on PC4 and PC5. These two reach the host through the ST-LINK's
    // virtual COM port, over solder bridges SB13 and SB15 — so the same USB
    // cable that flashes the board also carries this, and no extra hardware is
    // involved.
    let tx = gpioc
        .pc4
        .into_af_push_pull(&mut gpioc.moder, &mut gpioc.otyper, &mut gpioc.afrl);
    let rx = gpioc
        .pc5
        .into_af_push_pull(&mut gpioc.moder, &mut gpioc.otyper, &mut gpioc.afrl);

    let mut serial = Serial::new(dp.USART1, (tx, rx), BAUD.Bd(), clocks, &mut rcc.apb2);

    let mut gyro = I3G4250D::new(spi, cs).unwrap();

    defmt::println!("stage 5 - onto the PC");
    defmt::println!("=====================");

    let id = gyro.who_am_i().unwrap();
    defmt::println!("identification = {:#04x} (expect {:#04x})", id, EXPECTED_ID);
    if id != EXPECTED_ID {
        defmt::println!("!! wrong answer - stop here, nothing below this line means anything");
    }

    gyro.set_scale(Scale::Dps500).unwrap();
    gyro.set_odr(Odr::Hz200).unwrap();
    let scale = gyro.scale().unwrap();

    let mono = MonoTimer::new(cp.DWT, clocks, &mut cp.DCB);
    let cycles_per_second = mono.frequency().0 as f32;

    let mut tick = Timer::new(dp.TIM2, clocks, &mut rcc.apb1);
    tick.start((1_000 / SAMPLE_HZ).milliseconds());

    defmt::println!("sending {} byte packets at {} baud", PACKET_LEN, BAUD);
    defmt::println!("");
    defmt::println!("Gyro only. Put the board down and leave it - then look again.");
    defmt::println!("");

    // Gyro only: no accelerometer, no magnetometer, nothing that measures.
    let mut estimate = Attitude::new();
    let mut last = mono.now();
    let mut ticks: u32 = 0;

    loop {
        let _ = block!(tick.wait());

        let dt = last.elapsed() as f32 / cycles_per_second;
        last = mono.now();

        let raw = gyro.gyro().unwrap();
        let rate = Vec3::new(
            scale.degrees(raw.x),
            scale.degrees(raw.y),
            scale.degrees(raw.z),
        );

        // The filter works in radians per second; the datasheet talks degrees.
        estimate.integrate(rate * (core::f32::consts::PI / 180.0), dt);

        let orientation = estimate.orientation();

        // `bwrite_all` blocks until the last byte has left the shift register,
        // so there is no buffering hiding between this line and the wire.
        serial.bwrite_all(&packet(orientation)).ok();

        // Where the board's own +Y axis is pointing, expressed in the world —
        // which is a heading, and the one thing the ring can show.
        let forward = orientation * Vec3::Y;
        let heading = wrap_degrees(libm::atan2f(forward.x, forward.y).to_degrees());
        show(ring_pattern(heading));

        ticks = ticks.wrapping_add(1);
        if ticks % SAMPLE_HZ == 0 {
            defmt::println!("heading {=f32} deg after {} seconds", heading, ticks / SAMPLE_HZ);
        }
    }
}

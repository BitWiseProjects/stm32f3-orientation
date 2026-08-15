//! Stage 0 — does any of this work?
//!
//! Flash this first. It is the smallest program that proves the whole chain
//! is alive: the toolchain compiled for the right chip, `probe-rs` found the
//! debug interface, the binary is running, and `defmt` can get a message back
//! out to your terminal.
//!
//! ```text
//! cargo run --bin 00_ring
//! ```
//!
//! **PASS** = a light walks around the ring, and a line appears in the
//! terminal each time it goes round.
//!
//! If nothing happens, the problem is here — in the setup — and not in
//! anything the later stages do. That is the entire reason this file exists
//! separately from stage 1, which otherwise does the same thing much better.
//!
//! There is no clock configuration in this file. The chip wakes up running
//! from its internal 8 MHz oscillator, and for blinking a light that is
//! completely fine. Stage 1 is where the clock becomes worth setting up.

#![no_main]
#![no_std]

use cortex_m::asm::delay;
use defmt_rtt as _; // sends `defmt::println!` back up the debug cable
use panic_probe as _; // a panic stops the chip and prints why
use stm32f3xx_hal::gpio::{Gpioe, Output, Pin, PushPull, Ux};
use stm32f3xx_hal::{self as hal, prelude::*};

/// About a third of a second — and deliberately not more precise than that.
///
/// `asm::delay` counts loop *iterations*, not time, and its inner loop costs
/// three cycles per iteration on a Cortex-M4. So a million of these is about
/// three million cycles, which at the 8 MHz startup clock is roughly 0.37 s.
/// Measured on this board: eight steps to a lap, and a lap takes about three
/// seconds.
///
/// Do not read a duration out of the number itself. Stage 1 switches to a
/// hardware timer, which is the only way to get a delay you can put a figure on.
const STEP: u32 = 1_000_000;

#[cortex_m_rt::entry]
fn main() -> ! {
    let dp = hal::pac::Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();
    let mut gpioe = dp.GPIOE.split(&mut rcc.ahb);

    defmt::println!("stage 0 - if you can read this, the debug cable works");

    // The eight ring LEDs, in pin order: PE8 through PE15.
    //
    // `downgrade()` throws away the pin number from the type so they can sit
    // in an array together. Every pin is a different type until you do that,
    // which is the compiler making sure you cannot use PE8 where PE9 belongs.
    let mut ring: [Pin<Gpioe, Ux, Output<PushPull>>; 8] = [
        gpioe.pe8.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper).downgrade(),
        gpioe.pe9.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper).downgrade(),
        gpioe.pe10.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper).downgrade(),
        gpioe.pe11.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper).downgrade(),
        gpioe.pe12.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper).downgrade(),
        gpioe.pe13.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper).downgrade(),
        gpioe.pe14.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper).downgrade(),
        gpioe.pe15.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper).downgrade(),
    ];

    let mut laps: u32 = 0;
    loop {
        for led in &mut ring {
            led.set_high().ok();
            delay(STEP);
            led.set_low().ok();
        }

        defmt::println!("lap {}", laps);
        laps = laps.wrapping_add(1);
    }
}
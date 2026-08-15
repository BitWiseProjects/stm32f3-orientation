//! Stage 1 — the ring is one byte.
//!
//! Stage 0 walked a light around by talking to eight separate pins. That works,
//! and it is the wrong mental model. All eight of these LEDs live on the same
//! GPIO port, on consecutive pins PE8 through PE15, which means the state of
//! the whole ring is **one eight-bit number**. Writing that number lights a
//! pattern.
//!
//! ```text
//! cargo run --bin 01_ring
//! ```
//!
//! **PASS**, in two halves:
//!
//! 1. A single light walks clockwise from the top-left, all the way round,
//!    four times. Watch the step from West back to NW — the bit falls off one
//!    end of the byte and reappears at the other, and on the ring that is just
//!    the next position. No bump, no jump.
//! 2. Then **two** neighbouring lights walk round together, four times.
//!
//! That second half is the one that matters later. North almost never lands
//! exactly on an LED; most of the time it falls between two of them, and
//! lighting both is how the ring says "somewhere in here". Eight LEDs, sixteen
//! directions — and it costs nothing, because it is the same single write.
//!
//! # Two other things arrive in this file
//!
//! **The clock.** The chip wakes up on its 8 MHz internal oscillator. Here it
//! is switched to 72 MHz, sourced from the external clock signal the ST-LINK
//! feeds in on OSC_IN. Note `bypass_hse()` — there is no crystal wired to the
//! main processor on this board, only a driven clock, and asking for the wrong
//! one of those two hangs the chip silently inside `freeze()`.
//!
//! **`Delay` rather than a counting loop.** `cortex_m::asm::delay` counts loop
//! iterations, and at 72 MHz the flash needs wait states that stretch every
//! iteration — so a cycle-counted delay gets *slower per cycle* the faster you
//! clock the chip. Measured on this board: a delay sized for one second came
//! out at 2.86. `Delay` is driven by SysTick, a hardware counter, which flash
//! latency cannot reach.

#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;
use stm32f3xx_hal::delay::Delay;
use stm32f3xx_hal::hal::blocking::delay::DelayMs;
use stm32f3xx_hal::{self as hal, prelude::*};

/// How long each position stays lit.
const STEP_MS: u16 = 120;

/// Times round the ring before switching between the two patterns.
const LAPS: u32 = 4;

/// One light. Bit 0 of this byte is PE8, which is the NW position.
const SINGLE: u8 = 0b0000_0001;

/// Two neighbours, lit together.
const PAIR: u8 = 0b0000_0011;

/// Light exactly the LEDs whose bit is set, and darken the rest.
///
/// `BSRR` is the set/reset register: writing a 1 into the low half turns a pin
/// on, and writing a 1 into the high half turns it off. Both happen in a single
/// store, so the ring never passes through a half-updated state — which it
/// would if this were a read, a modify and a write of the output register.
///
/// The pins are shifted up by 8 because the ring starts at PE8.
fn show(pattern: u8) {
    let set = u32::from(pattern) << 8;
    let reset = u32::from(!pattern) << 24;

    // SAFETY: BSRR is write-only and its bits act independently, so this
    // cannot disturb a partially-completed write elsewhere. The eight pins it
    // touches were configured as outputs above and belong to nothing else.
    unsafe {
        (*hal::pac::GPIOE::ptr()).bsrr.write(|w| w.bits(set | reset));
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let dp = hal::pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    // 72 MHz, from the 8 MHz clock the ST-LINK drives into OSC_IN.
    let clocks = rcc
        .cfgr
        .use_hse(8.MHz())
        .bypass_hse()
        .sysclk(72.MHz())
        .freeze(&mut flash.acr);

    let mut gpioe = dp.GPIOE.split(&mut rcc.ahb);

    // Configure all eight as push-pull outputs, then drop the handles. From
    // here on the port is written as a whole through `show`, so the individual
    // pin types have done their job — they exist to make this configuration
    // step impossible to skip.
    gpioe.pe8.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    gpioe.pe9.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    gpioe.pe10.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    gpioe.pe11.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    gpioe.pe12.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    gpioe.pe13.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    gpioe.pe14.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    gpioe.pe15.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);

    let mut delay = Delay::new(cp.SYST, clocks);

    defmt::println!("stage 1 - the ring as one byte");
    defmt::println!("sysclk {} Hz", clocks.sysclk().0);
    defmt::println!("");

    loop {
        for (name, pattern) in [("one light", SINGLE), ("two neighbours", PAIR)] {
            defmt::println!("{}: {} laps", name, LAPS);

            for _ in 0..LAPS {
                for step in 0..8 {
                    // Rotating the byte walks the pattern round the ring. A
                    // rotation, not a shift: bits that fall off the top come
                    // back at the bottom, which on a circle is exactly right.
                    show(pattern.rotate_left(step));
                    delay.delay_ms(STEP_MS);
                }
            }
        }
    }
}

//! Diagnostic: prints the raw X/Y/Z of the accelerometer, gyro and
//! magnetometer twice a second, for checking which way each chip's axes point.

#![no_main]
#![no_std]

use defmt_rtt as _;
use i3g4250d::{I3G4250D, Odr, Scale};
use lsm303agr::{AccelMode, AccelOutputDataRate, Lsm303agr, MagMode, MagOutputDataRate};
use panic_probe as _;
use stm32f3xx_hal::delay::Delay;
use stm32f3xx_hal::hal::blocking::delay::DelayMs;
use stm32f3xx_hal::hal::spi::MODE_3;
use stm32f3xx_hal::i2c::I2c;
use stm32f3xx_hal::spi::{config::Config, Spi};
use stm32f3xx_hal::{self as hal, prelude::*};

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

    let scl = gpiob
        .pb6
        .into_af_open_drain::<4>(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
    let sda = gpiob
        .pb7
        .into_af_open_drain::<4>(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
    let i2c = I2c::new(dp.I2C1, (scl, sda), 100_000.Hz(), clocks, &mut rcc.apb1);
    let mut delay = Delay::new(cp.SYST, clocks);

    let mut compass = Lsm303agr::new_with_i2c(i2c);
    compass.init().unwrap();
    compass
        .set_accel_mode_and_odr(&mut delay, AccelMode::HighResolution, AccelOutputDataRate::Hz100)
        .unwrap();
    compass
        .set_mag_mode_and_odr(&mut delay, MagMode::HighResolution, MagOutputDataRate::Hz100)
        .unwrap();
    let mut compass = compass.into_mag_continuous().ok().unwrap();

    let mut gyro = I3G4250D::new(spi, cs).unwrap();
    gyro.set_scale(Scale::Dps500).unwrap();
    gyro.set_odr(Odr::Hz200).unwrap();
    let scale = gyro.scale().unwrap();

    defmt::println!("99 - axes");

    loop {
        let a = compass.acceleration().unwrap();
        let m = compass.magnetic_field().unwrap();
        let g = gyro.gyro().unwrap();
        defmt::println!(
            "accel {=i32} {=i32} {=i32} mg   gyro {=f32} {=f32} {=f32} dps   mag {=i32} {=i32} {=i32} nT",
            a.x_mg(),
            a.y_mg(),
            a.z_mg(),
            scale.degrees(g.x),
            scale.degrees(g.y),
            scale.degrees(g.z),
            m.x_nt(),
            m.y_nt(),
            m.z_nt()
        );
        delay.delay_ms(500u16);
    }
}

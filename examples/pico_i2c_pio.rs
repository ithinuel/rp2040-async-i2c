//! # Pico I2C PIO Example
//!
//! Reads the temperature from an LM75B
//!
//! This read over I2C the temerature from an LM75B temperature sensor wired on pins 20 and 21
//! using the PIO peripheral as an I2C bus controller.
//! The pins used for the I2C can be remapped to any other pin available to the PIO0 peripheral.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

// I2C HAL traits & Types.
use embedded_hal_async::i2c::{I2c, Operation};
use fugit::RateExtU32;

use defmt_rtt as _;
use panic_probe as _;

use rp_pico::hal::{self, pac, Clock};

/// Prints the temperature received from the sensor
fn print_temperature(temp: [u8; 2]) {
    let temp_i16 = i16::from_be_bytes(temp) >> 5;
    let temp_f32 = f32::from(temp_i16) * 0.125;

    // Write formatted output but ignore any error.
    defmt::info!("Temperature: {=f32}°C", temp_f32);
}

/// The function configures the RP2040 peripherals, reads the temperature from
/// the attached LM75B using PIO0.
async fn demo() {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let sio = hal::Sio::new(pac.SIO);
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    use rp2040_hal::pio::PIOExt;
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let mut i2c_pio = rp2040_async_i2c::pio::I2c::new(
        &mut pio,
        pins.gpio20.into_pull_up_disabled(),
        pins.gpio21.into_pull_up_disabled(),
        sm0,
        100.kHz(),
        clocks.system_clock.freq(),
    );

    let mut temp = [0; 2];
    i2c_pio
        .read(0x48u8, &mut temp)
        .await
        .expect("Failed to read from the peripheral");
    print_temperature(temp);

    i2c_pio
        .write(0x48u8, &[0])
        .await
        .expect("Failed to write to the peripheral");

    let mut temp = [0; 2];
    i2c_pio
        .read(0x48u8, &mut temp)
        .await
        .expect("Failed to read from the peripheral");
    print_temperature(temp);

    let mut config = [0];
    let mut thyst = [0; 2];
    let mut tos = [0; 2];
    let mut temp = [0; 2];
    let mut operations = [
        Operation::Write(&[1]),
        Operation::Read(&mut config),
        Operation::Write(&[2]),
        Operation::Read(&mut thyst),
        Operation::Write(&[3]),
        Operation::Read(&mut tos),
        Operation::Write(&[0]),
        Operation::Read(&mut temp),
    ];
    i2c_pio
        .transaction(0x48u8, &mut operations)
        .await
        .expect("Failed to run all operations");
    print_temperature(temp);

    core::future::pending().await
}

/// Entry point to our bare-metal application.
#[rp_pico::entry]
fn main() -> ! {
    let runtime = nostd_async::Runtime::new();
    let mut task = nostd_async::Task::new(demo());
    let handle = task.spawn(&runtime);
    handle.join();
    unreachable!()
}

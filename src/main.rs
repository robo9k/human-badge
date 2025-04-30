//! This example test the Pimoroni Pico Plus 2 on board LED.
//!
//! It does not work with the RP Pico 2 board. See `blinky.rs`.

#![no_std]
#![no_main]

use cyw43_pio::{PioSpi, RM2_CLOCK_DIVIDER};
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::{bind_interrupts, gpio};
use embassy_time::{Duration, Timer};
use gpio::{Level, Output};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};
use embassy_rp::i2c::{self, Config, InterruptHandler as I2cInterruptHandler};
use embassy_rp::peripherals::I2C0;

// Program metadata for `picotool info`.
// This isn't needed, but it's recomended to have these minimal entries.
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"Blinky Example"),
    embassy_rp::binary_info::rp_program_description!(
        c"This example tests the RP Pico on board LED, connected to gpio 25"
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    I2C0_IRQ => I2cInterruptHandler<I2C0>;
});

#[embassy_executor::task]
async fn cyw43_task(runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>) -> ! {
    runner.run().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let fw = include_bytes!("../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../cyw43-firmware/43439A0_clm.bin");

    // To make flashing faster for development, you may want to flash the firmwares independently
    // at hardcoded addresses, instead of baking them into the program with `include_bytes!`:
    //     probe-rs download ../../cyw43-firmware/43439A0.bin --binary-format bin --chip RP235x --base-address 0x10100000
    //     probe-rs download ../../cyw43-firmware/43439A0_clm.bin --binary-format bin --chip RP235x --base-address 0x10140000
    //let fw = unsafe { core::slice::from_raw_parts(0x10100000 as *const u8, 230321) };
    //let clm = unsafe { core::slice::from_raw_parts(0x10140000 as *const u8, 4752) };

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        RM2_CLOCK_DIVIDER,
        pio.irq0,
        cs,
        p.PIN_24,
        p.PIN_29,
        p.DMA_CH0,
    );

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (_net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    unwrap!(spawner.spawn(cyw43_task(runner)));

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    // https://cdn.shopify.com/s/files/1/0174/1800/files/ppico_plus_2_w_pinout_diagram.pdf?v=1727346378
    // https://cdn.shopify.com/s/files/1/0174/1800/files/Pimoroni_Pico_Plus_2_W_Schematic.pdf?v=1727350279
    let sda = p.PIN_4;
    let scl = p.PIN_5;

    info!("set up i2c ");
    let i2c = i2c::I2c::new_async(p.I2C0, scl, sda, Irqs, Config::default());

    const DEFAULT_ADDRESS: u8 = 0x76;

    use embedded_devices::devices::bosch::bme280::{BME280Async, Configuration, address::Address};
    use embedded_devices::devices::bosch::bme280::registers::{IIRFilter, Oversampling};
    use uom::si::thermodynamic_temperature::degree_celsius;
    use uom::num_traits::ToPrimitive;

    let mut delay = embassy_time::Delay;

    // Create and initialize the device
    let mut bme280 = BME280Async::new_i2c(i2c, Address::Custom(DEFAULT_ADDRESS));
    bme280.init(&mut delay).await.unwrap();
    bme280.configure(Configuration {
        temperature_oversampling: Oversampling::X_16,
        pressure_oversampling: Oversampling::X_16,
        humidity_oversampling: Oversampling::X_16,
        iir_filter: IIRFilter::Disabled,
    }).await.unwrap();

    // Read the current temperature in °C and convert it to a float
    let measurements = bme280.measure(&mut delay).await.unwrap();
    let temp = measurements.temperature.get::<degree_celsius>().to_f32();
    println!("Current temperature: {:?}°C", temp);

    let delay = Duration::from_secs(1);

    // RP2040 would be embassy_rp::flash::blocking_unique_id(), see https://github.com/embassy-rs/embassy/blob/572e788b2e878436bde527ad66cf561775cebc66/examples/rp/src/bin/flash.rs#L34
    let board_id = embassy_rp::otp::get_chipid().unwrap();
    info!("board id: {=u64:#X}", board_id);

    loop {
        info!("led on!");
        control.gpio_set(0, true).await;
        Timer::after(delay).await;

        info!("led off!");
        control.gpio_set(0, false).await;
        Timer::after(delay).await;
    }
}

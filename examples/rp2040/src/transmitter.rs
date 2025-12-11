#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::{
    gpio,
    peripherals::SPI0,
    spi::{self, Spi},
};
use embassy_time::{Delay, Duration, Timer};
use gpio::{Input, Level, Output};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;

use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use rfm69_rs::rfm69::Rfm69;

type Spi0Bus = Mutex<NoopRawMutex, Spi<'static, SPI0, spi::Async>>;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("init rp..");
    let p = embassy_rp::init(Default::default());
    info!("init pins...");
    let reset_pin = Output::new(p.PIN_21, Level::Low);
    let delay = Delay;

    let miso = p.PIN_16;
    let mosi = p.PIN_19;
    let clk = p.PIN_18;
    let cs_pin = p.PIN_17;
    let dio01 = Input::new(p.PIN_15, gpio::Pull::Up);

    let radio_cs = Output::new(cs_pin, Level::High);
    info!("config spi..");
    // create SPI
    let mut config = spi::Config::default();
    config.frequency = 1_000_000;
    config.phase = spi::Phase::CaptureOnFirstTransition;
    config.polarity = spi::Polarity::IdleLow;
    info!("setup spi ..");
    let spi = Spi::new(p.SPI0, clk, mosi, miso, p.DMA_CH0, p.DMA_CH1, config);

    info!("init spi ..");
    static SPI_BUS: StaticCell<Spi0Bus> = StaticCell::new();
    let spi_bus = SPI_BUS.init(Mutex::new(spi));

    let spi_device = SpiDevice::new(spi_bus, radio_cs);

    let mut rfm69 = Rfm69::new(spi_device, reset_pin, dio01, delay);
    info!("init rfm ..");
    rfm69.init().await.unwrap();
    rfm69.set_tx_power(13).await.unwrap();
    rfm69.set_frequency(433).await.unwrap();

    let registers = rfm69.read_all_registers().await.unwrap();
    registers.iter().for_each(|register| {
        info!("0x{:02X}: 0x{:02X}", register.0, register.1);
    });

    loop {
        rfm69.send("Hello, World!".as_bytes()).await.unwrap();
        info!("Sent message");
        let temperature = rfm69.read_temperature().await.unwrap();
        info!("Temperature: {}", temperature);
        Timer::after(Duration::from_secs(1)).await;
    }
}

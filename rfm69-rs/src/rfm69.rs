//! RFM69 Radio Driver
//!
//! An async driver for the RFM69 series of radio modules using `embedded-hal-async` traits.
//!
//! # Example
//!
//! ```ignore
//! let mut rfm69 = Rfm69::new(spi_device, reset_pin, dio0_pin, delay);
//! rfm69.init().await?;
//! rfm69.set_frequency(915).await?;
//!
//! // Send a message
//! rfm69.send(b"Hello, World!").await?;
//!
//! // Receive a message
//! rfm69.set_mode(Rfm69Mode::Rx).await?;
//! rfm69.wait_for_message().await?;
//! let mut buffer = [0u8; 65];
//! let len = rfm69.receive(&mut buffer).await?;
//! ```

use crate::read_write::ReadWrite;
use crate::registers::Register;
use crate::settings::{
    ContinuousDagc, ModemConfigChoice, SyncConfiguration, RF69_FSTEP, RF69_FXOSC,
    RF_DIOMAPPING1_DIO0_00, RF_PALEVEL_OUTPUTPOWER_11111, RF_PALEVEL_PA0_ON, RF_PALEVEL_PA1_ON,
    RF_PALEVEL_PA2_ON,
};
use defmt::{debug, Format};
use embedded_hal::{digital::InputPin, digital::OutputPin};
use embedded_hal_async::{delay::DelayNs, digital::Wait};

/// RFM69 radio driver instance.
///
/// Generic over SPI device, reset pin, interrupt pin, and delay provider.
pub struct Rfm69<SPI, RESET, INTR, D> {
    spi: SPI,
    reset_pin: RESET,
    intr_pin: INTR,
    delay: D,
    tx_power: i8,
    is_high_power: bool,
    current_mode: Rfm69Mode,
}

/// Errors that can occur when interacting with the RFM69 module.
#[derive(Debug, PartialEq, Format)]
pub enum Rfm69Error {
    /// Failed to reset the module.
    ResetError,
    /// SPI write operation failed.
    SpiWriteError,
    /// SPI read operation failed.
    SpiReadError,
    /// Invalid configuration provided.
    ConfigurationError,
    /// Message exceeds maximum size (60 bytes).
    MessageTooLarge,
    /// Operation not valid in current mode.
    InvalidMode,
    /// Waiting for interrupt failed.
    WaitError,
}

/// Operating mode of the RFM69 module.
#[derive(Clone, Debug, PartialEq, Format)]
pub enum Rfm69Mode {
    /// Sleep mode - lowest power consumption.
    Sleep = 0x00,
    /// Standby mode - ready to transmit or receive.
    Standby = 0x04,
    /// Frequency synthesizer mode.
    Fs = 0x08,
    /// Transmit mode.
    Tx = 0x0C,
    /// Receive mode.
    Rx = 0x10,
}

/// Configuration for initializing the RFM69 module.
#[derive(Clone)]
pub struct Rfm69Config {
    /// Sync word configuration.
    pub sync_configuration: SyncConfiguration,
    /// Sync words (up to 8 bytes).
    pub sync_words: [u8; 8],
    /// Number of sync words to use (1-8).
    pub sync_word_len: usize,
    /// Modem configuration preset.
    pub modem_config: ModemConfigChoice,
    /// Preamble length in bytes.
    pub preamble_length: u16,
    /// Frequency in MHz.
    pub frequency: u32,
    /// Transmit power in dBm.
    pub tx_power: i8,
    /// Whether this is a high power module (RFM69HW).
    pub is_high_power: bool,
}

impl Default for Rfm69Config {
    fn default() -> Self {
        Self {
            sync_configuration: SyncConfiguration::FifoFillAuto { sync_tolerance: 0 },
            sync_words: [0x2D, 0xD4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            sync_word_len: 2,
            modem_config: ModemConfigChoice::GfskRb250Fd250,
            preamble_length: 4,
            frequency: 915,
            tx_power: 13,
            is_high_power: true,
        }
    }
}

impl<SPI, RESET, INTR, D> Rfm69<SPI, RESET, INTR, D>
where
    SPI: ReadWrite,
    RESET: OutputPin,
    INTR: InputPin + Wait,
    D: DelayNs,
{
    async fn reset(&mut self) -> Result<(), Rfm69Error> {
        self.reset_pin
            .set_high()
            .map_err(|_| Rfm69Error::ResetError)?;
        self.delay.delay_us(100).await;
        self.reset_pin
            .set_low()
            .map_err(|_| Rfm69Error::ResetError)?;
        self.delay.delay_ms(5).await;
        Ok(())
    }

    /// Creates a new RFM69 driver instance.
    ///
    /// # Arguments
    ///
    /// * `spi` - SPI device for communication
    /// * `reset_pin` - GPIO pin connected to the module's reset
    /// * `intr_pin` - GPIO pin connected to DIO0 for interrupts
    /// * `delay` - Delay provider for timing operations
    #[must_use]
    pub fn new(spi: SPI, reset_pin: RESET, intr_pin: INTR, delay: D) -> Self {
        Rfm69 {
            spi,
            reset_pin,
            intr_pin,
            delay,
            tx_power: 13,
            is_high_power: true,
            current_mode: Rfm69Mode::Standby,
        }
    }

    /// Initializes the RFM69 module with default settings.
    ///
    /// This resets the module and configures it with sensible defaults:
    /// - GFSK modulation at 250kbps
    /// - 915 MHz frequency
    /// - 13 dBm transmit power
    /// - 2-byte sync word (0x2D, 0xD4)
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the module fails or
    /// if the module doesn't respond with the expected version.
    pub async fn init(&mut self) -> Result<(), Rfm69Error> {
        self.delay.delay_ms(10).await;
        self.reset().await?;

        let version = self.read_register(Register::Version).await?;

        debug!("RFM69 version: {:?}", version);

        // the RFM69 module should return 0x24
        if version != 0x24 {
            return Err(Rfm69Error::SpiReadError);
        }

        // self.spi.write_many(Register::OpMode, &[0x04]);

        self.set_default_fifo_threshold().await?;
        self.set_dagc(ContinuousDagc::ImprovedLowBeta1).await?;

        self.write_register(Register::Lna, 0x88).await?;
        let sync_word = [0x2D, 0xD4];
        self.set_sync_words(
            SyncConfiguration::FifoFillAuto { sync_tolerance: 0 },
            &sync_word,
        )
        .await?;

        // If high power boost set previously, disable it
        self.write_register(Register::TestPa1, 0x55).await?;
        self.write_register(Register::TestPa2, 0x70).await?;

        self.set_modem_config(ModemConfigChoice::GfskRb250Fd250)
            .await?;

        self.set_preamble_length(4).await?;

        self.set_tx_power(13).await?;

        self.set_frequency(915).await?;

        self.set_mode(Rfm69Mode::Standby).await?;

        Ok(())
    }

    /// Initializes the RFM69 module with the provided configuration.
    ///
    /// # Arguments
    ///
    /// * `config` - Configuration settings for the module
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the module fails or
    /// if the configuration is invalid.
    pub async fn init_with_config(&mut self, config: Rfm69Config) -> Result<(), Rfm69Error> {
        self.delay.delay_ms(10).await;
        self.reset().await?;

        let version = self.read_register(Register::Version).await?;

        debug!("RFM69 version: {:?}", version);

        if version != 0x24 {
            return Err(Rfm69Error::SpiReadError);
        }

        self.is_high_power = config.is_high_power;

        self.set_default_fifo_threshold().await?;
        self.set_dagc(ContinuousDagc::ImprovedLowBeta1).await?;

        self.write_register(Register::Lna, 0x88).await?;
        self.set_sync_words(
            config.sync_configuration,
            &config.sync_words[..config.sync_word_len],
        )
        .await?;

        self.write_register(Register::TestPa1, 0x55).await?;
        self.write_register(Register::TestPa2, 0x70).await?;

        self.set_modem_config(config.modem_config).await?;
        self.set_preamble_length(config.preamble_length).await?;
        self.set_tx_power(config.tx_power).await?;
        self.set_frequency(config.frequency).await?;

        self.set_mode(Rfm69Mode::Standby).await?;

        Ok(())
    }

    /// Reads all registers from the RFM69 module.
    ///
    /// Returns an array of (register address, value) tuples for debugging purposes.
    pub async fn read_all_registers(&mut self) -> Result<[(u8, u8); 84], Rfm69Error> {
        let mut registers = [0u8; 79];
        self.read_many(Register::OpMode, &mut registers).await?;

        let mut mapped: [(u8, u8); 84] = [(0, 0); 84]; // Initialize the mapped array

        for (index, &value) in registers.iter().enumerate() {
            mapped[index] = ((index + 1).try_into().unwrap(), value);
        }

        mapped[79] = (
            Register::TestLna.addr(),
            self.read_register(Register::TestLna).await?,
        );
        mapped[80] = (
            Register::TestPa1.addr(),
            self.read_register(Register::TestPa1).await?,
        );
        mapped[81] = (
            Register::TestPa2.addr(),
            self.read_register(Register::TestPa2).await?,
        );
        mapped[82] = (
            Register::TestDagc.addr(),
            self.read_register(Register::TestDagc).await?,
        );
        mapped[83] = (
            Register::TestAfc.addr(),
            self.read_register(Register::TestAfc).await?,
        );

        Ok(mapped)
    }

    /// Reads the chip revision/version register.
    ///
    /// Should return 0x24 for RFM69 modules.
    pub async fn read_revision(&mut self) -> Result<u8, Rfm69Error> {
        self.read_register(Register::Version).await
    }

    /// Reads the internal temperature sensor.
    ///
    /// Returns the temperature in degrees Celsius.
    /// Note: This is approximate and intended for relative measurements.
    pub async fn read_temperature(&mut self) -> Result<f32, Rfm69Error> {
        self.write_register(Register::Temp1, 0x08).await?;
        while self.read_register(Register::Temp1).await? & 0x04 != 0x00 {
            self.delay.delay_ms(10).await;
        }

        let temp = self.read_register(Register::Temp2).await?;
        Ok((166 as f32) - temp as f32)
    }

    async fn set_default_fifo_threshold(&mut self) -> Result<(), Rfm69Error> {
        self.write_register(Register::FifoThresh, 0x8F).await?;
        Ok(())
    }

    async fn set_dagc(&mut self, value: ContinuousDagc) -> Result<(), Rfm69Error> {
        self.write_register(Register::TestDagc, value as u8).await?;
        Ok(())
    }

    async fn set_sync_words(
        &mut self,
        config: SyncConfiguration,
        sync_words: &[u8],
    ) -> Result<(), Rfm69Error> {
        if sync_words.len() > 8 || sync_words.len() == 0 {
            return Err(Rfm69Error::ConfigurationError);
        }

        let mut buffer = [0u8; 9]; // 1 byte for config + up to 8 bytes for sync words

        // Add the config value to the first position
        // We need to know how many sync words we have to set the correct config value
        buffer[0] = config.value(sync_words.len() as u8);
        // Add the sync words to the buffer
        buffer[1..1 + sync_words.len()].copy_from_slice(sync_words);
        // Write the config value first, then the sync words.
        self.write_many(Register::SyncConfig, &buffer).await?;

        Ok(())
    }

    async fn set_modem_config(&mut self, config: ModemConfigChoice) -> Result<(), Rfm69Error> {
        let values = config.values();

        self.write_many(Register::DataModul, &values[0..5]).await?;
        self.write_many(Register::RxBw, &values[5..7]).await?;
        self.write_register(Register::PacketConfig1, values[7])
            .await?;

        Ok(())
    }

    async fn set_preamble_length(&mut self, preamble_length: u16) -> Result<(), Rfm69Error> {
        // split the preamble length into two bytes
        let msb = (preamble_length >> 8) as u8;
        let lsb = preamble_length as u8;

        // write the two bytes to the RFM69
        let buffer = [msb, lsb];

        self.write_many(Register::PreambleMsb, &buffer).await?;
        Ok(())
    }

    /// Sets the carrier frequency.
    ///
    /// # Arguments
    ///
    /// * `freq_mhz` - Frequency in MHz (e.g., 433, 868, 915)
    pub async fn set_frequency(&mut self, freq_mhz: u32) -> Result<(), Rfm69Error> {
        let mut frf = (freq_mhz * RF69_FSTEP) as u32;
        frf /= RF69_FXOSC as u32;

        // split the frequency into three bytes
        let msb = ((frf >> 16) & 0xFF) as u8;
        let mid = ((frf >> 8) & 0xFF) as u8;
        let lsb = (frf & 0xFF) as u8;

        let buffer = [msb, mid, lsb];
        self.write_many(Register::FrfMsb, &buffer).await?;
        Ok(())
    }

    /// Sets the transmit power level.
    ///
    /// # Arguments
    ///
    /// * `tx_power` - Power level in dBm
    ///   - For high power modules (RFM69HW): -2 to +20 dBm
    ///   - For standard modules: -18 to +13 dBm
    ///
    /// Values outside these ranges will be clamped.
    pub async fn set_tx_power(&mut self, tx_power: i8) -> Result<(), Rfm69Error> {
        let pa_level;

        if self.is_high_power {
            let clamped_power = tx_power.clamp(-2, 20);

            if clamped_power <= 13 {
                // -2dBm to +13dBm
                // Need PA1 exclusivelly on RFM69HW
                pa_level =
                    RF_PALEVEL_PA1_ON | ((tx_power + 18) as u8 & RF_PALEVEL_OUTPUTPOWER_11111);
            } else if clamped_power >= 18 {
                // +18dBm to +20dBm
                // Need PA1+PA2
                // Also need PA boost settings change when tx is turned on and off, see setModeTx()
                pa_level = RF_PALEVEL_PA1_ON
                    | RF_PALEVEL_PA2_ON
                    | ((tx_power + 11) as u8 & RF_PALEVEL_OUTPUTPOWER_11111);
            } else {
                // +14dBm to +17dBm
                // Need PA1+PA2
                pa_level = RF_PALEVEL_PA1_ON
                    | RF_PALEVEL_PA2_ON
                    | ((tx_power + 14) as u8 & RF_PALEVEL_OUTPUTPOWER_11111);
            }
        } else {
            let clamped_power = tx_power.clamp(-18, 13);
            pa_level =
                RF_PALEVEL_PA0_ON | ((clamped_power + 18) as u8 & RF_PALEVEL_OUTPUTPOWER_11111);
        }

        self.write_register(Register::PaLevel, pa_level).await?;
        self.tx_power = tx_power;
        Ok(())
    }

    /// Sets the operating mode of the RFM69 module.
    ///
    /// # Arguments
    ///
    /// * `mode` - The desired operating mode
    ///
    /// This method configures the DIO0 interrupt mapping appropriately:
    /// - In TX mode: DIO0 signals PacketSent
    /// - In RX mode: DIO0 signals PayloadReady
    pub async fn set_mode(&mut self, mode: Rfm69Mode) -> Result<(), Rfm69Error> {
        if self.current_mode == mode {
            return Ok(());
        }

        match mode {
            Rfm69Mode::Rx => {
                // If high power boost, return power amp to receive mode
                if self.tx_power >= 18 {
                    self.write_register(Register::TestPa1, 0x55).await?;
                    self.write_register(Register::TestPa2, 0x70).await?;
                }

                // Configure DIO0 for PayloadReady interrupt in RX mode
                self.write_register(Register::DioMapping1, RF_DIOMAPPING1_DIO0_00)
                    .await?;
            }

            Rfm69Mode::Tx => {
                // If high power boost, enable power amp
                if self.tx_power >= 18 {
                    self.write_register(Register::TestPa1, 0x5D).await?;
                    self.write_register(Register::TestPa2, 0x7C).await?;
                }

                // Configure DIO0 for PacketSent interrupt in TX mode
                self.write_register(Register::DioMapping1, RF_DIOMAPPING1_DIO0_00)
                    .await?;
            }

            _ => {}
        }

        // Read the current mode
        let mut current_mode = self.read_register(Register::OpMode).await?;
        current_mode &= !0x1C;
        current_mode |= mode.clone() as u8 & 0x1C;

        // // Set the new mode
        self.write_register(Register::OpMode, current_mode).await?;
        while (self.read_register(Register::IrqFlags1).await? & 0x80) == 0x00 {
            self.delay.delay_ms(10).await;
        }

        self.current_mode = mode;
        Ok(())
    }

    async fn wait_packet_sent(&mut self) -> Result<(), Rfm69Error> {
        // DIO0 is configured for PacketSent in TX mode, wait for IRQ
        self.intr_pin
            .wait_for_high()
            .await
            .map_err(|_| Rfm69Error::WaitError)?;
        Ok(())
    }

    /// Sends a data packet.
    ///
    /// # Arguments
    ///
    /// * `data` - Data to send (maximum 60 bytes)
    ///
    /// # Errors
    ///
    /// Returns `Rfm69Error::MessageTooLarge` if data exceeds 60 bytes.
    ///
    /// This method automatically switches to TX mode, waits for transmission
    /// to complete, then returns to Standby mode.
    pub async fn send(&mut self, data: &[u8]) -> Result<(), Rfm69Error> {
        const HEADER_LENGTH: usize = 5;

        if data.len() > 60 {
            return Err(Rfm69Error::MessageTooLarge);
        }

        let mut buffer: [u8; 65] = [0x00; 65];
        let header = [0xFF, 0xFF, 0x00, 0x00];
        buffer[0] = (data.len() + 4) as u8;
        buffer[1..5].copy_from_slice(&header);
        buffer[5..5 + data.len()].copy_from_slice(data);

        self.write_many(Register::Fifo, &buffer[0..data.len() + HEADER_LENGTH])
            .await?;

        self.set_mode(Rfm69Mode::Tx).await?;
        self.wait_packet_sent().await?;
        self.set_mode(Rfm69Mode::Standby).await?;

        Ok(())
    }

    /// Checks if a message is available in the FIFO.
    ///
    /// Must be in RX mode to call this method.
    ///
    /// # Errors
    ///
    /// Returns `Rfm69Error::InvalidMode` if not in RX mode.
    pub async fn is_message_available(&mut self) -> Result<bool, Rfm69Error> {
        if self.current_mode != Rfm69Mode::Rx {
            return Err(Rfm69Error::InvalidMode);
        }
        Ok((self.read_register(Register::IrqFlags2).await? & 0x04) == 0x04)
    }

    /// Waits for a message to be received.
    ///
    /// This method blocks until a message is available, using the DIO0
    /// interrupt to efficiently wait without polling.
    ///
    /// Must be in RX mode before calling this method.
    ///
    /// # Errors
    ///
    /// Returns `Rfm69Error::InvalidMode` if not in RX mode.
    /// Returns `Rfm69Error::WaitError` if the interrupt wait fails.
    pub async fn wait_for_message(&mut self) -> Result<(), Rfm69Error> {
        if self.current_mode != Rfm69Mode::Rx {
            return Err(Rfm69Error::InvalidMode);
        }

        // DIO0 is configured for PayloadReady in RX mode, wait for IRQ
        self.intr_pin
            .wait_for_high()
            .await
            .map_err(|_| Rfm69Error::WaitError)?;
        Ok(())
    }

    /// Waits for a message with a timeout.
    ///
    /// Similar to `wait_for_message()`, but returns after the specified
    /// timeout if no message is received.
    ///
    /// # Arguments
    ///
    /// * `timeout_ms` - Maximum time to wait in milliseconds
    ///
    /// # Returns
    ///
    /// Returns `Ok(true)` if a message was received, `Ok(false)` if the
    /// timeout expired.
    ///
    /// # Errors
    ///
    /// Returns `Rfm69Error::InvalidMode` if not in RX mode.
    pub async fn wait_for_message_with_timeout(
        &mut self,
        timeout_ms: u32,
    ) -> Result<bool, Rfm69Error> {
        if self.current_mode != Rfm69Mode::Rx {
            return Err(Rfm69Error::InvalidMode);
        }

        // Poll with delays up to timeout
        let mut elapsed: u32 = 0;
        const POLL_INTERVAL_MS: u32 = 10;

        while elapsed < timeout_ms {
            if self.is_message_available().await? {
                return Ok(true);
            }
            self.delay.delay_ms(POLL_INTERVAL_MS).await;
            elapsed += POLL_INTERVAL_MS;
        }

        Ok(false)
    }

    /// Receives a message from the FIFO.
    ///
    /// Call `wait_for_message()` or check `is_message_available()` before
    /// calling this method to ensure a message is ready.
    ///
    /// # Arguments
    ///
    /// * `buffer` - Buffer to store the received message
    ///
    /// # Returns
    ///
    /// Returns the number of bytes received (excluding the 4-byte header).
    pub async fn receive(&mut self, buffer: &mut [u8; 65]) -> Result<usize, Rfm69Error> {
        let message_len = self.read_register(Register::Fifo).await?;
        if buffer.len() < message_len as usize {
            return Err(Rfm69Error::MessageTooLarge);
        }

        let mut header = [0u8; 4];
        self.read_many(Register::Fifo, &mut header).await?;

        self.read_many(Register::Fifo, &mut buffer[0..(message_len - 4) as usize])
            .await?;
        Ok((message_len - 4) as usize)
    }

    /// Reads the current RSSI (Received Signal Strength Indicator).
    ///
    /// # Returns
    ///
    /// Returns the RSSI value in -dBm (e.g., 40 means -40 dBm).
    pub async fn rssi(&mut self) -> Result<u8, Rfm69Error> {
        let rssi = self.read_register(Register::RssiValue).await?;
        Ok(rssi / 2)
    }

    /// Returns the current operating mode.
    #[must_use]
    pub fn current_mode(&self) -> &Rfm69Mode {
        &self.current_mode
    }

    /// Returns the current transmit power setting in dBm.
    #[must_use]
    pub fn tx_power(&self) -> i8 {
        self.tx_power
    }

    /// Returns whether this is configured as a high power module.
    #[must_use]
    pub fn is_high_power(&self) -> bool {
        self.is_high_power
    }

    async fn write_register(&mut self, register: Register, value: u8) -> Result<(), Rfm69Error> {
        self.write_many(register, &[value]).await?;
        Ok(())
    }

    async fn read_register(&mut self, register: Register) -> Result<u8, Rfm69Error> {
        let mut buffer = [0u8; 1];
        self.spi
            .read_many(register, &mut buffer)
            .await
            .map_err(|_| Rfm69Error::SpiWriteError)?;
        Ok(buffer[0])
    }

    async fn write_many(&mut self, register: Register, values: &[u8]) -> Result<(), Rfm69Error> {
        self.spi
            .write_many(register, values)
            .await
            .map_err(|_| Rfm69Error::SpiWriteError)?;
        Ok(())
    }

    async fn read_many(&mut self, register: Register, buffer: &mut [u8]) -> Result<(), Rfm69Error> {
        self.spi
            .read_many(register, buffer)
            .await
            .map_err(|_| Rfm69Error::SpiReadError)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {

    use crate::settings::{ContinuousDagc, SyncConfiguration};

    use super::*;
    use embedded_hal_mock::eh1::delay::{CheckedDelay, Transaction as DelayTransaction};
    use embedded_hal_mock::eh1::digital::{
        Mock as DigitalMock, State, Transaction as GpioTransaction,
    };
    use embedded_hal_mock::eh1::spi::{Mock as SpiDevice, Transaction as SpiTransaction};

    fn setup_rfm() -> Rfm69<SpiDevice<u8>, DigitalMock, DigitalMock, CheckedDelay> {
        let spi_expectations = [];
        let spi_device = SpiDevice::new(spi_expectations);

        let reset_expectations = [];
        let reset_pin = DigitalMock::new(reset_expectations);

        let intr_expectations = [];
        let intr_pin = DigitalMock::new(intr_expectations);

        let delay_expectations = [];
        let delay = CheckedDelay::new(delay_expectations);

        Rfm69::new(spi_device, reset_pin, intr_pin, delay)
    }

    fn check_expectations(rfm: &mut Rfm69<SpiDevice<u8>, DigitalMock, DigitalMock, CheckedDelay>) {
        rfm.reset_pin.done();
        rfm.intr_pin.done();
        rfm.delay.done();
        rfm.spi.done();
    }

    #[tokio::test]
    async fn test_reset() {
        let mut rfm = setup_rfm();

        let reset_expectations = [
            GpioTransaction::set(State::High),
            GpioTransaction::set(State::Low),
        ];
        rfm.reset_pin.update_expectations(&reset_expectations);

        let delay_expectations = [
            DelayTransaction::delay_us(100),
            DelayTransaction::delay_ms(5),
        ];
        rfm.delay.update_expectations(&delay_expectations);

        rfm.reset().await.unwrap();

        check_expectations(&mut rfm);
    }

    #[tokio::test]
    async fn test_read_temperature() {
        let mut rfm = setup_rfm();

        let temperature_expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::Temp1.write()),
            SpiTransaction::write(0x08),
            SpiTransaction::transaction_end(),
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::Temp1.read()),
            SpiTransaction::transfer_in_place(vec![0x00], vec![0x04]),
            SpiTransaction::transaction_end(),
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::Temp1.read()),
            SpiTransaction::transfer_in_place(vec![0x00], vec![0x00]),
            SpiTransaction::transaction_end(),
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::Temp2.read()),
            SpiTransaction::transfer_in_place(vec![0x00], vec![0x8D]),
            SpiTransaction::transaction_end(),
        ];
        rfm.spi.update_expectations(&temperature_expectations);

        let delay_expectations = [DelayTransaction::delay_ms(10)];
        rfm.delay.update_expectations(&delay_expectations);

        let temperature = rfm.read_temperature().await.unwrap();

        assert_eq!(temperature, 25.0);

        check_expectations(&mut rfm);
    }

    #[tokio::test]
    async fn test_set_default_fifo_threshold() {
        let mut rfm = setup_rfm();

        let spi_expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::FifoThresh.write()),
            SpiTransaction::write(0x8F),
            SpiTransaction::transaction_end(),
        ];

        rfm.spi.update_expectations(&spi_expectations);

        rfm.set_default_fifo_threshold().await.unwrap();

        check_expectations(&mut rfm);
    }

    #[tokio::test]
    async fn test_set_dagc() {
        let mut rfm = setup_rfm();

        let spi_expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::TestDagc.write()),
            SpiTransaction::write(ContinuousDagc::ImprovedLowBeta1 as u8),
            SpiTransaction::transaction_end(),
        ];

        rfm.spi.update_expectations(&spi_expectations);

        rfm.set_dagc(ContinuousDagc::ImprovedLowBeta1)
            .await
            .unwrap();

        check_expectations(&mut rfm);
    }

    #[test]
    fn test_set_power() {
        let mut rfm = setup_rfm();

        let spi_expectations = [];

        rfm.spi.update_expectations(&spi_expectations);

        check_expectations(&mut rfm);
    }

    #[tokio::test]
    async fn test_set_sync_words() {
        let mut rfm = setup_rfm();

        let spi_expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::SyncConfig.write()),
            SpiTransaction::write_vec(vec![184, 1, 2, 3, 4, 5, 6, 7, 8]),
            SpiTransaction::transaction_end(),
        ];

        rfm.spi.update_expectations(&spi_expectations);

        let sync_words = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08];
        rfm.set_sync_words(
            SyncConfiguration::FifoFillAuto { sync_tolerance: 0 },
            &sync_words,
        )
        .await
        .unwrap();

        check_expectations(&mut rfm);
    }

    #[tokio::test]
    async fn test_set_sync_words_clamp() {
        let mut rfm = setup_rfm();

        let spi_expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::SyncConfig.write()),
            SpiTransaction::write_vec(vec![191, 1, 2, 3, 4, 5, 6, 7, 8]),
            SpiTransaction::transaction_end(),
        ];

        rfm.spi.update_expectations(&spi_expectations);

        let sync_words = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08];

        // sync_tolerance is clamped to 7
        rfm.set_sync_words(
            SyncConfiguration::FifoFillAuto { sync_tolerance: 14 },
            &sync_words,
        )
        .await
        .unwrap();

        check_expectations(&mut rfm);
    }

    #[tokio::test]
    async fn test_set_sync_words_too_long() {
        let mut rfm = setup_rfm();

        let spi_expectations = [];

        rfm.spi.update_expectations(&spi_expectations);

        let sync_words = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09];
        assert_eq!(
            rfm.set_sync_words(
                SyncConfiguration::FifoFillAuto { sync_tolerance: 0 },
                &sync_words
            )
            .await,
            Err(Rfm69Error::ConfigurationError)
        );

        check_expectations(&mut rfm);
    }

    #[tokio::test]
    async fn test_set_sync_words_empty() {
        let mut rfm = setup_rfm();

        let spi_expectations = [];

        rfm.spi.update_expectations(&spi_expectations);

        let sync_words = [];
        assert_eq!(
            rfm.set_sync_words(
                SyncConfiguration::FifoFillAuto { sync_tolerance: 0 },
                &sync_words
            )
            .await,
            Err(Rfm69Error::ConfigurationError)
        );

        check_expectations(&mut rfm);
    }

    #[tokio::test]
    async fn test_set_modem_config() {
        let mut rfm = setup_rfm();

        let spi_expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::DataModul.write()),
            SpiTransaction::write_vec(vec![0x00, 0x3e, 0x80, 0x00, 0x52]),
            SpiTransaction::transaction_end(),
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::RxBw.write()),
            SpiTransaction::write_vec(vec![0xf4, 0xf4]),
            SpiTransaction::transaction_end(),
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::PacketConfig1.write()),
            SpiTransaction::write(0xd0),
            SpiTransaction::transaction_end(),
        ];

        rfm.spi.update_expectations(&spi_expectations);

        rfm.set_modem_config(ModemConfigChoice::FskRb2Fd5)
            .await
            .unwrap();

        check_expectations(&mut rfm);
    }

    #[tokio::test]
    async fn test_set_preamble_length() {
        let mut rfm = setup_rfm();

        let spi_expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::PreambleMsb.write()),
            SpiTransaction::write_vec(vec![0x00, 0xFF]),
            SpiTransaction::transaction_end(),
        ];

        rfm.spi.update_expectations(&spi_expectations);

        rfm.set_preamble_length(255).await.unwrap();

        check_expectations(&mut rfm);
    }

    #[tokio::test]
    async fn test_get_revision() {
        let mut rfm = setup_rfm();

        let spi_expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::Version.read()),
            SpiTransaction::transfer_in_place(vec![0x00], vec![0x24]),
            SpiTransaction::transaction_end(),
        ];

        rfm.spi.update_expectations(&spi_expectations);

        let revision = rfm.read_revision().await.unwrap();
        assert_eq!(revision, 0x24);

        check_expectations(&mut rfm);
    }

    #[tokio::test]
    async fn test_set_frequency() {
        let mut rfm = setup_rfm();

        let spi_expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::FrfMsb.write()),
            SpiTransaction::write_vec(vec![0xE4, 0xC0, 0x00]),
            SpiTransaction::transaction_end(),
        ];

        rfm.spi.update_expectations(&spi_expectations);

        rfm.set_frequency(915).await.unwrap();

        check_expectations(&mut rfm);
    }

    #[tokio::test]
    async fn test_set_tx_power() {
        let mut rfm = setup_rfm();

        let spi_expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::PaLevel.write()),
            SpiTransaction::write(0x50),
            SpiTransaction::transaction_end(),
        ];

        rfm.spi.update_expectations(&spi_expectations);

        rfm.set_tx_power(-2).await.unwrap();
        assert_eq!(rfm.tx_power, -2);

        check_expectations(&mut rfm);
    }

    #[tokio::test]
    async fn test_set_mode_rx() {
        let mut rfm = setup_rfm();
        rfm.tx_power = 18;

        let spi_expectations = [
            // If high power boost, return power amp to receive mode
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::TestPa1.write()),
            SpiTransaction::write(0x55),
            SpiTransaction::transaction_end(),
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::TestPa2.write()),
            SpiTransaction::write(0x70),
            SpiTransaction::transaction_end(),
            // Configure DIO0 for PayloadReady interrupt in RX mode
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::DioMapping1.write()),
            SpiTransaction::write(RF_DIOMAPPING1_DIO0_00),
            SpiTransaction::transaction_end(),
            // Read the current value of OpMode
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::OpMode.read()),
            SpiTransaction::transfer_in_place(vec![0x00], vec![0xC4]),
            SpiTransaction::transaction_end(),
            // Set the new mode, leaving the other bits unchanged
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::OpMode.write()),
            SpiTransaction::write(0xD0),
            SpiTransaction::transaction_end(),
            // Wait for the mode to change
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::IrqFlags1.read()),
            SpiTransaction::transfer_in_place(vec![0x00], vec![0x00]),
            SpiTransaction::transaction_end(),
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::IrqFlags1.read()),
            SpiTransaction::transfer_in_place(vec![0x00], vec![0x80]),
            SpiTransaction::transaction_end(),
        ];

        let delay_expectations = [DelayTransaction::delay_ms(10)];

        rfm.spi.update_expectations(&spi_expectations);
        rfm.delay.update_expectations(&delay_expectations);

        rfm.set_mode(Rfm69Mode::Rx).await.unwrap();

        check_expectations(&mut rfm);
    }

    #[tokio::test]
    async fn test_set_mode_tx() {
        let mut rfm = setup_rfm();
        rfm.tx_power = 18;

        let spi_expectations = [
            // If high power boost, return power amp to receive mode
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::TestPa1.write()),
            SpiTransaction::write(0x5D),
            SpiTransaction::transaction_end(),
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::TestPa2.write()),
            SpiTransaction::write(0x7C),
            SpiTransaction::transaction_end(),
            // Configure DIO0 for PacketSent interrupt in TX mode
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::DioMapping1.write()),
            SpiTransaction::write(RF_DIOMAPPING1_DIO0_00),
            SpiTransaction::transaction_end(),
            // // Read the current value of OpMode
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::OpMode.read()),
            SpiTransaction::transfer_in_place(vec![0x00], vec![0xC4]),
            SpiTransaction::transaction_end(),
            // // Set the new mode, leaving the other bits unchanged
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::OpMode.write()),
            SpiTransaction::write(0xCC),
            SpiTransaction::transaction_end(),
            // // Wait for the mode to change
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::IrqFlags1.read()),
            SpiTransaction::transfer_in_place(vec![0x00], vec![0x00]),
            SpiTransaction::transaction_end(),
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::IrqFlags1.read()),
            SpiTransaction::transfer_in_place(vec![0x00], vec![0x80]),
            SpiTransaction::transaction_end(),
        ];

        let delay_expectations = [DelayTransaction::delay_ms(10)];

        rfm.spi.update_expectations(&spi_expectations);
        rfm.delay.update_expectations(&delay_expectations);

        rfm.set_mode(Rfm69Mode::Tx).await.unwrap();

        check_expectations(&mut rfm);
    }

    #[tokio::test]
    async fn test_send_too_large() {
        let mut rfm = setup_rfm();

        let message = ['a' as u8; 70];

        assert_eq!(rfm.send(&message).await, Err(Rfm69Error::MessageTooLarge));

        check_expectations(&mut rfm);
    }

    #[tokio::test]
    async fn test_send() {
        let mut rfm = setup_rfm();

        let mut header = vec![17, 0xFF, 0xFF, 0x00, 0x00];
        let mut message = "Hello, world!".as_bytes().to_vec();

        header.append(&mut message);

        let spi_expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::Fifo.write()),
            SpiTransaction::write_vec(header),
            SpiTransaction::transaction_end(),
            // Configure DIO0 for PacketSent interrupt in TX mode
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::DioMapping1.write()),
            SpiTransaction::write(RF_DIOMAPPING1_DIO0_00),
            SpiTransaction::transaction_end(),
            // // Read the current value of OpMode
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::OpMode.read()),
            SpiTransaction::transfer_in_place(vec![0x00], vec![0xC4]),
            SpiTransaction::transaction_end(),
            // // Set the new mode, leaving the other bits unchanged
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::OpMode.write()),
            SpiTransaction::write(0xCC),
            SpiTransaction::transaction_end(),
            // // Wait for the mode to change
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::IrqFlags1.read()),
            SpiTransaction::transfer_in_place(vec![0x00], vec![0x00]),
            SpiTransaction::transaction_end(),
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::IrqFlags1.read()),
            SpiTransaction::transfer_in_place(vec![0x00], vec![0x80]),
            SpiTransaction::transaction_end(),
            // wait_packet_sent now uses IRQ (intr_pin.wait_for_high()) - no SPI expectations
            // // // Read the current value of OpMode
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::OpMode.read()),
            SpiTransaction::transfer_in_place(vec![0x00], vec![0xC0]),
            SpiTransaction::transaction_end(),
            // // // Set the new mode, leaving the other bits unchanged
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::OpMode.write()),
            SpiTransaction::write(0xC4),
            SpiTransaction::transaction_end(),
            // // // // Wait for the mode to change
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::IrqFlags1.read()),
            SpiTransaction::transfer_in_place(vec![0x00], vec![0x80]),
            SpiTransaction::transaction_end(),
        ];

        let delay_expectations = [DelayTransaction::delay_ms(10)];

        let intr_expectations = [GpioTransaction::wait_for_state(State::High)];

        rfm.spi.update_expectations(&spi_expectations);
        rfm.delay.update_expectations(&delay_expectations);
        rfm.intr_pin.update_expectations(&intr_expectations);

        let message = "Hello, world!".as_bytes();

        rfm.send(message).await.unwrap();

        check_expectations(&mut rfm);
    }

    #[tokio::test]
    async fn test_receive() {
        let mut rfm = setup_rfm();

        let spi_expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::Fifo.read()),
            SpiTransaction::transfer_in_place(vec![0x00], vec![9]),
            SpiTransaction::transaction_end(),
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::Fifo.read()),
            SpiTransaction::transfer_in_place(
                vec![0x00, 0x00, 0x00, 0x00],
                vec![0x00, 0x00, 0x00, 0x00],
            ),
            SpiTransaction::transaction_end(),
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::Fifo.read()),
            SpiTransaction::transfer_in_place(
                vec![0x00, 0x00, 0x00, 0x00, 0x00],
                vec![0x00, 0x00, 0x00, 0x00, 0x00],
            ),
            SpiTransaction::transaction_end(),
        ];

        rfm.spi.update_expectations(&spi_expectations);

        let mut buffer = [0u8; 65];

        let message_len = rfm.receive(&mut buffer).await.unwrap();
        assert_eq!(message_len, 5);

        check_expectations(&mut rfm);
    }

    #[tokio::test]
    async fn test_is_message_available() {
        let mut rfm = setup_rfm();
        rfm.current_mode = Rfm69Mode::Rx;

        let spi_expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::IrqFlags2.read()),
            SpiTransaction::transfer_in_place(vec![0x00], vec![0x04]),
            SpiTransaction::transaction_end(),
        ];
        rfm.spi.update_expectations(&spi_expectations);

        assert_eq!(rfm.is_message_available().await.unwrap(), true);

        let spi_expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::IrqFlags2.read()),
            SpiTransaction::transfer_in_place(vec![0x00], vec![0x00]),
            SpiTransaction::transaction_end(),
        ];
        rfm.spi.update_expectations(&spi_expectations);

        assert_eq!(rfm.is_message_available().await.unwrap(), false);

        rfm.current_mode = Rfm69Mode::Tx;
        assert_eq!(
            rfm.is_message_available().await,
            Err(Rfm69Error::InvalidMode)
        );

        check_expectations(&mut rfm);
    }

    #[tokio::test]
    async fn test_rssi() {
        let mut rfm = setup_rfm();
        rfm.current_mode = Rfm69Mode::Rx;

        let spi_expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::RssiValue.read()),
            SpiTransaction::transfer_in_place(vec![0x00], vec![0x50]),
            SpiTransaction::transaction_end(),
        ];
        rfm.spi.update_expectations(&spi_expectations);

        assert_eq!(rfm.rssi().await.unwrap(), 40);

        check_expectations(&mut rfm);
    }
}

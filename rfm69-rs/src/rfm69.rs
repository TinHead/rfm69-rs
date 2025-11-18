use crate::read_write::ReadWrite;
use crate::registers::Register;
use crate::settings::{
    ContinuousDagc, ModemConfigChoice, SyncConfiguration, RF69_FSTEP, RF69_FXOSC,
    RF_DIOMAPPING1_DIO0_00, RF_DIOMAPPING1_DIO0_01, RF_PALEVEL_OUTPUTPOWER_11111,
    RF_PALEVEL_PA0_ON, RF_PALEVEL_PA1_ON, RF_PALEVEL_PA2_ON,
};
use defmt::{debug, info, Format};
use embedded_hal::{digital::InputPin, digital::OutputPin};
use embedded_hal_async::{delay::DelayNs, digital::Wait};

pub struct Rfm69<SPI, RESET, INTR, D> {
    pub spi: SPI,
    pub reset_pin: RESET,
    pub intr_pin: INTR,
    pub delay: D,
    tx_power: i8,
    is_high_power: bool,
    current_mode: Rfm69Mode,
}

#[derive(Debug, PartialEq, Format)]
pub enum Rfm69Error {
    ResetError,
    SpiWriteError,
    SpiReadError,
    ConfigurationError,
    MessageTooLarge,
    InvalidMode,
}

#[derive(Clone, Debug, PartialEq, Format)]
pub enum Rfm69Mode {
    Sleep = 0x00,
    Standby = 0x04,
    Fs = 0x08,
    Tx = 0x0C,
    Rx = 0x10,
}

pub struct Rfm69Config {
    pub sync_configuration: SyncConfiguration,
    pub sync_words: [u8; 8],
    pub modem_config: ModemConfigChoice,
    pub preamble_length: u16,
    pub frequency: u32,
    pub tx_power: i8,
    pub is_high_power: bool,
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

    pub async fn init(&mut self) -> Result<(), Rfm69Error> {
        self.delay.delay_ms(10).await;
        self.reset().await?;

        let version = self.read_register(Register::Version)?;

        debug!("RFM69 version: {:?}", version);

        // the RFM69 module should return 0x24
        if version != 0x24 {
            return Err(Rfm69Error::SpiReadError);
        }

        // self.spi.write_many(Register::OpMode, &[0x04]);

        self.set_default_fifo_threshold()?;
        self.set_dagc(ContinuousDagc::ImprovedLowBeta1)?;

        self.write_register(Register::Lna, 0x88)?;
        let sync_word = [0x2D, 0xD4];
        self.set_sync_words(
            SyncConfiguration::FifoFillAuto { sync_tolerance: 0 },
            &sync_word,
        )?;

        // If high power boost set previously, disable it
        self.write_register(Register::TestPa1, 0x55)?;
        self.write_register(Register::TestPa2, 0x70)?;

        self.set_modem_config(ModemConfigChoice::GfskRb250Fd250)?;

        self.set_preamble_length(4)?;

        self.set_tx_power(13)?;

        self.set_frequency(915)?;

        self.set_mode(Rfm69Mode::Standby).await?;

        Ok(())
    }

    pub fn read_all_registers(&mut self) -> Result<[(u8, u8); 84], Rfm69Error> {
        let mut registers = [0u8; 79];
        self.read_many(Register::OpMode, &mut registers)?;

        let mut mapped: [(u8, u8); 84] = [(0, 0); 84]; // Initialize the mapped array

        for (index, &value) in registers.iter().enumerate() {
            mapped[index] = ((index + 1).try_into().unwrap(), value);
        }

        mapped[79] = (
            Register::TestLna.addr(),
            self.read_register(Register::TestLna)?,
        );
        mapped[80] = (
            Register::TestPa1.addr(),
            self.read_register(Register::TestPa1)?,
        );
        mapped[81] = (
            Register::TestPa2.addr(),
            self.read_register(Register::TestPa2)?,
        );
        mapped[82] = (
            Register::TestDagc.addr(),
            self.read_register(Register::TestDagc)?,
        );
        mapped[83] = (
            Register::TestAfc.addr(),
            self.read_register(Register::TestAfc)?,
        );

        Ok(mapped)
    }

    pub fn read_revision(&mut self) -> Result<u8, Rfm69Error> {
        self.read_register(Register::Version)
    }

    pub async fn read_temperature(&mut self) -> Result<f32, Rfm69Error> {
        self.write_register(Register::Temp1, 0x08)?;
        while self.read_register(Register::Temp1)? & 0x04 != 0x00 {
            self.delay.delay_ms(10).await;
        }

        let temp = self.read_register(Register::Temp2)?;
        Ok((166 as f32) - temp as f32)
    }

    fn set_default_fifo_threshold(&mut self) -> Result<(), Rfm69Error> {
        self.write_register(Register::FifoThresh, 0x8F)?;
        Ok(())
    }

    fn set_dagc(&mut self, value: ContinuousDagc) -> Result<(), Rfm69Error> {
        self.write_register(Register::TestDagc, value as u8)?;
        Ok(())
    }

    fn set_sync_words(
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
        self.write_many(Register::SyncConfig, &buffer)?;

        Ok(())
    }

    fn set_modem_config(&mut self, config: ModemConfigChoice) -> Result<(), Rfm69Error> {
        let values = config.values();

        self.write_many(Register::DataModul, &values[0..5])?;
        self.write_many(Register::RxBw, &values[5..7])?;
        self.write_register(Register::PacketConfig1, values[7])?;

        Ok(())
    }

    fn set_preamble_length(&mut self, preamble_length: u16) -> Result<(), Rfm69Error> {
        // split the preamble length into two bytes
        let msb = (preamble_length >> 8) as u8;
        let lsb = preamble_length as u8;

        // write the two bytes to the RFM69
        let buffer = [msb, lsb];

        self.write_many(Register::PreambleMsb, &buffer)?;
        Ok(())
    }

    pub fn set_frequency(&mut self, freq_mhz: u32) -> Result<(), Rfm69Error> {
        let mut frf = (freq_mhz * RF69_FSTEP) as u32;
        frf /= RF69_FXOSC as u32;

        // split the frequency into three bytes
        let msb = ((frf >> 16) & 0xFF) as u8;
        let mid = ((frf >> 8) & 0xFF) as u8;
        let lsb = (frf & 0xFF) as u8;

        let buffer = [msb, mid, lsb];
        self.write_many(Register::FrfMsb, &buffer)?;
        Ok(())
    }

    pub fn set_tx_power(&mut self, tx_power: i8) -> Result<(), Rfm69Error> {
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

        self.write_register(Register::PaLevel, pa_level)?;
        self.tx_power = tx_power;
        Ok(())
    }

    pub async fn set_mode(&mut self, mode: Rfm69Mode) -> Result<(), Rfm69Error> {
        if self.current_mode == mode {
            return Ok(());
        }

        match mode {
            Rfm69Mode::Rx => {
                // If high power boost, return power amp to receive mode
                if self.tx_power >= 18 {
                    self.write_register(Register::TestPa1, 0x55)?;
                    self.write_register(Register::TestPa2, 0x70)?;
                }
            }

            Rfm69Mode::Tx => {
                // If high power boost, enable power amp
                if self.tx_power >= 18 {
                    self.write_register(Register::TestPa1, 0x5D)?;
                    self.write_register(Register::TestPa2, 0x7C)?;
                }

                // enable the packet sent interrupt
                self.write_register(Register::DioMapping1, RF_DIOMAPPING1_DIO0_00)?;
            }

            _ => {}
        }

        // Read the current mode
        let mut current_mode = self.read_register(Register::OpMode)?;
        current_mode &= !0x1C;
        current_mode |= mode.clone() as u8 & 0x1C;

        // // Set the new mode
        self.write_register(Register::OpMode, current_mode)?;
        while (self.read_register(Register::IrqFlags1)? & 0x80) == 0x00 {
            self.delay.delay_ms(10).await;
        }

        self.current_mode = mode;
        Ok(())
    }

    async fn wait_packet_sent(&mut self) -> Result<(), Rfm69Error> {
        self.intr_pin.wait_for_high().await.unwrap();
        while (self.read_register(Register::IrqFlags2)? & 0x08) == 0 {
            info!("Waiting for packet sent...");
            self.delay.delay_ms(10).await;
        }
        Ok(())
    }

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

        self.write_many(Register::Fifo, &buffer[0..data.len() + HEADER_LENGTH])?;

        self.set_mode(Rfm69Mode::Tx).await?;
        self.wait_packet_sent().await?;
        self.set_mode(Rfm69Mode::Standby).await?;

        Ok(())
    }

    pub fn is_message_available(&mut self) -> Result<bool, Rfm69Error> {
        if self.current_mode != Rfm69Mode::Rx {
            return Err(Rfm69Error::InvalidMode);
        }
        Ok((self.read_register(Register::IrqFlags2)? & 0x04) == 0x04)
    }

    pub async fn wait_for_message(&mut self) -> Result<(), Rfm69Error> {
        while !self.is_message_available()? {
            self.delay.delay_ms(1000).await;
        }
        Ok(())
    }

    pub async fn receive(&mut self, buffer: &mut [u8; 65]) -> Result<usize, Rfm69Error> {
        let message_len = self.read_register(Register::Fifo)?;
        if buffer.len() < message_len as usize {
            return Err(Rfm69Error::MessageTooLarge);
        }

        let mut header = [0u8; 4];
        self.read_many(Register::Fifo, &mut header).unwrap();

        self.read_many(Register::Fifo, &mut buffer[0..(message_len - 4) as usize])
            .unwrap();
        Ok((message_len - 4) as usize)
    }

    pub fn rssi(&mut self) -> Result<u8, Rfm69Error> {
        let rssi = self.read_register(Register::RssiValue)?;
        Ok(rssi / 2)
    }

    fn write_register(&mut self, register: Register, value: u8) -> Result<(), Rfm69Error> {
        self.write_many(register, &[value])?;
        Ok(())
    }

    fn read_register(&mut self, register: Register) -> Result<u8, Rfm69Error> {
        let mut buffer = [0u8; 1];
        self.spi
            .read_many(register, &mut buffer)
            .map_err(|_| Rfm69Error::SpiWriteError)?;
        Ok(buffer[0])
    }

    fn write_many(&mut self, register: Register, values: &[u8]) -> Result<(), Rfm69Error> {
        self.spi
            .write_many(register, values)
            .map_err(|_| Rfm69Error::SpiWriteError)?;
        Ok(())
    }

    fn read_many(&mut self, register: Register, buffer: &mut [u8]) -> Result<(), Rfm69Error> {
        self.spi
            .read_many(register, buffer)
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

    #[test]
    fn test_set_default_fifo_threshold() {
        let mut rfm = setup_rfm();

        let spi_expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::FifoThresh.write()),
            SpiTransaction::write(0x8F),
            SpiTransaction::transaction_end(),
        ];

        rfm.spi.update_expectations(&spi_expectations);

        rfm.set_default_fifo_threshold().unwrap();

        check_expectations(&mut rfm);
    }

    #[test]
    fn test_set_dagc() {
        let mut rfm = setup_rfm();

        let spi_expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::TestDagc.write()),
            SpiTransaction::write(ContinuousDagc::ImprovedLowBeta1 as u8),
            SpiTransaction::transaction_end(),
        ];

        rfm.spi.update_expectations(&spi_expectations);

        rfm.set_dagc(ContinuousDagc::ImprovedLowBeta1).unwrap();

        check_expectations(&mut rfm);
    }

    #[test]
    fn test_set_power() {
        let mut rfm = setup_rfm();

        let spi_expectations = [];

        rfm.spi.update_expectations(&spi_expectations);

        check_expectations(&mut rfm);
    }

    #[test]
    fn test_set_sync_words() {
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
        .unwrap();

        check_expectations(&mut rfm);
    }

    #[test]
    fn test_set_sync_words_clamp() {
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
        .unwrap();

        check_expectations(&mut rfm);
    }

    #[test]
    fn test_set_sync_words_too_long() {
        let mut rfm = setup_rfm();

        let spi_expectations = [];

        rfm.spi.update_expectations(&spi_expectations);

        let sync_words = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09];
        assert_eq!(
            rfm.set_sync_words(
                SyncConfiguration::FifoFillAuto { sync_tolerance: 0 },
                &sync_words
            ),
            Err(Rfm69Error::ConfigurationError)
        );

        check_expectations(&mut rfm);
    }

    #[test]
    fn test_set_sync_words_empty() {
        let mut rfm = setup_rfm();

        let spi_expectations = [];

        rfm.spi.update_expectations(&spi_expectations);

        let sync_words = [];
        assert_eq!(
            rfm.set_sync_words(
                SyncConfiguration::FifoFillAuto { sync_tolerance: 0 },
                &sync_words
            ),
            Err(Rfm69Error::ConfigurationError)
        );

        check_expectations(&mut rfm);
    }

    #[test]
    fn test_set_modem_config() {
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

        rfm.set_modem_config(ModemConfigChoice::FskRb2Fd5).unwrap();

        check_expectations(&mut rfm);
    }

    #[test]
    fn test_set_preamble_length() {
        let mut rfm = setup_rfm();

        let spi_expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::PreambleMsb.write()),
            SpiTransaction::write_vec(vec![0x00, 0xFF]),
            SpiTransaction::transaction_end(),
        ];

        rfm.spi.update_expectations(&spi_expectations);

        rfm.set_preamble_length(255).unwrap();

        check_expectations(&mut rfm);
    }

    #[test]
    fn test_get_revision() {
        let mut rfm = setup_rfm();

        let spi_expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::Version.read()),
            SpiTransaction::transfer_in_place(vec![0x00], vec![0x24]),
            SpiTransaction::transaction_end(),
        ];

        rfm.spi.update_expectations(&spi_expectations);

        let revision = rfm.read_revision().unwrap();
        assert_eq!(revision, 0x24);

        check_expectations(&mut rfm);
    }

    #[test]
    fn test_set_frequency() {
        let mut rfm = setup_rfm();

        let spi_expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::FrfMsb.write()),
            SpiTransaction::write_vec(vec![0xE4, 0xC0, 0x00]),
            SpiTransaction::transaction_end(),
        ];

        rfm.spi.update_expectations(&spi_expectations);

        rfm.set_frequency(915).unwrap();

        check_expectations(&mut rfm);
    }

    #[test]
    fn test_set_tx_power() {
        let mut rfm = setup_rfm();

        let spi_expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::PaLevel.write()),
            SpiTransaction::write(0x50),
            SpiTransaction::transaction_end(),
        ];

        rfm.spi.update_expectations(&spi_expectations);

        rfm.set_tx_power(-2).unwrap();
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
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::IrqFlags2.read()),
            SpiTransaction::transfer_in_place(vec![0x00], vec![0x08]),
            SpiTransaction::transaction_end(),
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

        rfm.spi.update_expectations(&spi_expectations);
        rfm.delay.update_expectations(&delay_expectations);

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

        assert_eq!(rfm.is_message_available().unwrap(), true);

        let spi_expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::IrqFlags2.read()),
            SpiTransaction::transfer_in_place(vec![0x00], vec![0x00]),
            SpiTransaction::transaction_end(),
        ];
        rfm.spi.update_expectations(&spi_expectations);

        assert_eq!(rfm.is_message_available().unwrap(), false);

        rfm.current_mode = Rfm69Mode::Tx;
        assert_eq!(rfm.is_message_available(), Err(Rfm69Error::InvalidMode));

        check_expectations(&mut rfm);
    }

    #[test]
    fn test_rssi() {
        let mut rfm = setup_rfm();
        rfm.current_mode = Rfm69Mode::Rx;

        let spi_expectations = [
            SpiTransaction::transaction_start(),
            SpiTransaction::write(Register::RssiValue.read()),
            SpiTransaction::transfer_in_place(vec![0x00], vec![0x50]),
            SpiTransaction::transaction_end(),
        ];
        rfm.spi.update_expectations(&spi_expectations);

        assert_eq!(rfm.rssi().unwrap(), 40);

        check_expectations(&mut rfm);
    }
}

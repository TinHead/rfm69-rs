use crate::read_write::AsyncReadWrite;
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
    SPI: AsyncReadWrite,
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

        let version = self.read_register(Register::Version).await?;

        debug!("RFM69 version: {:?}", version);

        if version != 0x24 {
            return Err(Rfm69Error::SpiReadError);
        }

        self.set_default_fifo_threshold().await?;
        self.set_dagc(ContinuousDagc::ImprovedLowBeta1).await?;

        self.write_register(Register::Lna, 0x88).await?;
        let sync_word = [0x2D, 0xD4];
        self.set_sync_words(
            SyncConfiguration::FifoFillAuto { sync_tolerance: 0 },
            &sync_word,
        ).await?;

        self.write_register(Register::TestPa1, 0x55).await?;
        self.write_register(Register::TestPa2, 0x70).await?;

        self.set_modem_config(ModemConfigChoice::GfskRb250Fd250).await?;
        self.set_preamble_length(4).await?;
        self.set_tx_power(13).await?;
        self.set_frequency(915).await?;

        self.set_mode(Rfm69Mode::Standby).await?;

        Ok(())
    }

    pub fn read_revision_blocking(&mut self) -> Result<u8, Rfm69Error> {
        // retained for diagnostic use; requires a blocking SPI fallback
        Err(Rfm69Error::SpiReadError)
    }

    async fn set_default_fifo_threshold(&mut self) -> Result<(), Rfm69Error> {
        self.write_register(Register::FifoThresh, 0x8F).await
    }

    async fn set_dagc(&mut self, value: ContinuousDagc) -> Result<(), Rfm69Error> {
        self.write_register(Register::TestDagc, value as u8).await
    }

    async fn set_sync_words(
        &mut self,
        config: SyncConfiguration,
        sync_words: &[u8],
    ) -> Result<(), Rfm69Error> {
        if sync_words.len() > 8 || sync_words.len() == 0 {
            return Err(Rfm69Error::ConfigurationError);
        }

        let mut buffer = [0u8; 9];
        buffer[0] = config.value(sync_words.len() as u8);
        buffer[1..1 + sync_words.len()].copy_from_slice(sync_words);
        self.write_many(Register::SyncConfig, &buffer).await
    }

    async fn set_modem_config(&mut self, config: ModemConfigChoice) -> Result<(), Rfm69Error> {
        let values = config.values();
        self.write_many(Register::DataModul, &values[0..5]).await?;
        self.write_many(Register::RxBw, &values[5..7]).await?;
        self.write_register(Register::PacketConfig1, values[7]).await
    }

    async fn set_preamble_length(&mut self, preamble_length: u16) -> Result<(), Rfm69Error> {
        let msb = (preamble_length >> 8) as u8;
        let lsb = preamble_length as u8;
        self.write_many(Register::PreambleMsb, &[msb, lsb]).await
    }

    pub async fn set_frequency(&mut self, freq_mhz: u32) -> Result<(), Rfm69Error> {
        let mut frf = (freq_mhz * RF69_FSTEP) as u32;
        frf /= RF69_FXOSC as u32;
        let msb = ((frf >> 16) & 0xFF) as u8;
        let mid = ((frf >> 8) & 0xFF) as u8;
        let lsb = (frf & 0xFF) as u8;
        self.write_many(Register::FrfMsb, &[msb, mid, lsb]).await
    }

    pub async fn set_tx_power(&mut self, tx_power: i8) -> Result<(), Rfm69Error> {
        let pa_level;

        if self.is_high_power {
            let clamped_power = tx_power.clamp(-2, 20);

            if clamped_power <= 13 {
                pa_level =
                    RF_PALEVEL_PA1_ON | ((tx_power + 18) as u8 & RF_PALEVEL_OUTPUTPOWER_11111);
            } else if clamped_power >= 18 {
                pa_level = RF_PALEVEL_PA1_ON
                    | RF_PALEVEL_PA2_ON
                    | ((tx_power + 11) as u8 & RF_PALEVEL_OUTPUTPOWER_11111);
            } else {
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

    pub async fn set_mode(&mut self, mode: Rfm69Mode) -> Result<(), Rfm69Error> {
        if self.current_mode == mode {
            return Ok(());
        }

        match mode {
            Rfm69Mode::Rx => {
                if self.tx_power >= 18 {
                    self.write_register(Register::TestPa1, 0x55).await?;
                    self.write_register(Register::TestPa2, 0x70).await?;
                }
                // Restore DIO0 = PayloadReady. set_mode(Tx) writes DIO0_00
                // (PacketSent); without this, re-entering Rx leaves DIO0
                // permanently mapped wrong and the chip never raises an IRQ
                // on a new packet.
                self.write_register(Register::DioMapping1, RF_DIOMAPPING1_DIO0_01).await?;
            }
            Rfm69Mode::Tx => {
                if self.tx_power >= 18 {
                    self.write_register(Register::TestPa1, 0x5D).await?;
                    self.write_register(Register::TestPa2, 0x7C).await?;
                }
                self.write_register(Register::DioMapping1, RF_DIOMAPPING1_DIO0_00).await?;
            }
            _ => {}
        }

        let mut current_mode = self.read_register(Register::OpMode).await?;
        current_mode &= !0x1C;
        current_mode |= mode.clone() as u8 & 0x1C;

        self.write_register(Register::OpMode, current_mode).await?;
        while (self.read_register(Register::IrqFlags1).await? & 0x80) == 0x00 {
            self.delay.delay_ms(10).await;
        }

        self.current_mode = mode;
        Ok(())
    }

    async fn wait_packet_sent(&mut self) -> Result<(), Rfm69Error> {
        self.intr_pin.wait_for_high().await.unwrap();
        while (self.read_register(Register::IrqFlags2).await? & 0x08) == 0 {
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

        self.write_many(Register::Fifo, &buffer[0..data.len() + HEADER_LENGTH]).await?;

        self.set_mode(Rfm69Mode::Tx).await?;
        self.wait_packet_sent().await?;
        self.set_mode(Rfm69Mode::Standby).await?;

        Ok(())
    }

    pub fn is_message_available_sync(&mut self) -> bool {
        // Polling check without async — not usable with async SPI
        // Use is_message_available() instead
        false
    }

    pub async fn is_message_available(&mut self) -> Result<bool, Rfm69Error> {
        if self.current_mode != Rfm69Mode::Rx {
            return Err(Rfm69Error::InvalidMode);
        }
        Ok((self.read_register(Register::IrqFlags2).await? & 0x04) == 0x04)
    }

    pub async fn wait_for_message(&mut self) -> Result<(), Rfm69Error> {
        while !self.is_message_available().await? {
            self.delay.delay_ms(1).await;
        }
        Ok(())
    }

    pub async fn receive(&mut self, buffer: &mut [u8; 65]) -> Result<usize, Rfm69Error> {
        let message_len = self.read_register(Register::Fifo).await?;
        // Bounds: header is 4 bytes, max payload is buffer.len() (65 - 4 = 61).
        // Without this guard a corrupt length byte underflows (message_len - 4)
        // and overruns the SPI burst, leaving FIFO in a permanently bad state.
        if message_len < 4 || message_len as usize > buffer.len() + 4 {
            self.restart_rx().await?;
            return Err(Rfm69Error::MessageTooLarge);
        }

        let mut header = [0u8; 4];
        self.read_many(Register::Fifo, &mut header).await?;

        let payload_len = (message_len - 4) as usize;
        self.read_many(Register::Fifo, &mut buffer[0..payload_len]).await?;

        // If a FIFO overrun is latched (bit 4 of IrqFlags2), the chip will stop
        // raising PayloadReady on subsequent packets until we pulse RestartRx.
        let irq2 = self.read_register(Register::IrqFlags2).await?;
        if irq2 & 0x10 != 0 {
            self.restart_rx().await?;
        }

        Ok(payload_len)
    }

    /// Pulse PacketConfig2.RestartRx (bit 2) to flush a stuck FIFO and restart
    /// receiver. The bit auto-clears in the chip per the datasheet.
    async fn restart_rx(&mut self) -> Result<(), Rfm69Error> {
        let pc2 = self.read_register(Register::PacketConfig2).await?;
        self.write_register(Register::PacketConfig2, pc2 | 0x04).await
    }

    pub async fn rssi(&mut self) -> Result<u8, Rfm69Error> {
        let rssi = self.read_register(Register::RssiValue).await?;
        Ok(rssi / 2)
    }

    async fn write_register(&mut self, register: Register, value: u8) -> Result<(), Rfm69Error> {
        self.write_many(register, &[value]).await
    }

    pub async fn read_register(&mut self, register: Register) -> Result<u8, Rfm69Error> {
        let mut buffer = [0u8; 1];
        self.spi
            .read_many_async(register, &mut buffer).await
            .map_err(|_| Rfm69Error::SpiReadError)?;
        Ok(buffer[0])
    }

    async fn write_many(&mut self, register: Register, values: &[u8]) -> Result<(), Rfm69Error> {
        self.spi
            .write_many_async(register, values).await
            .map_err(|_| Rfm69Error::SpiWriteError)
    }

    async fn read_many(&mut self, register: Register, buffer: &mut [u8]) -> Result<(), Rfm69Error> {
        self.spi
            .read_many_async(register, buffer).await
            .map_err(|_| Rfm69Error::SpiReadError)
    }
}

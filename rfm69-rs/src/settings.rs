//! RFM69 configuration settings and constants.
//!
//! This module provides configuration types for the RFM69 radio module,
//! including modem presets and sync word configuration.

#![allow(dead_code)]

/// The crystal oscillator frequency of the RF69 module in MHz.
pub const RF69_FXOSC: f32 = 32000000.0 / 1000000.0;

/// The Frequency Synthesizer step = RF69_FXOSC / 2^^19
pub const RF69_FSTEP: u32 = 524288;

/// DAGC (Digital Automatic Gain Control) configuration.
pub enum ContinuousDagc {
    NormalMode = 0x00,
    ImprovedLowBeta0 = 0x20,
    ImprovedLowBeta1 = 0x30,
}

/// Sync word configuration for packet detection.
///
/// The sync word is used to identify valid packets. When a matching
/// sync word is detected, the receiver knows a packet is starting.
#[derive(Clone, Copy)]
pub enum SyncConfiguration {
    /// Sync word detection disabled.
    SyncOff,
    /// Automatic FIFO fill after sync word detection.
    ///
    /// `sync_tolerance` specifies the number of bit errors allowed
    /// in the sync word (0-7).
    FifoFillAuto {
        /// Number of tolerated bit errors in sync word (0-7).
        sync_tolerance: u8,
    },
    /// Manual FIFO fill control after sync word detection.
    ///
    /// `sync_tolerance` specifies the number of bit errors allowed
    /// in the sync word (0-7).
    FifoFillManual {
        /// Number of tolerated bit errors in sync word (0-7).
        sync_tolerance: u8,
    },
}

impl SyncConfiguration {
    pub fn value(self, sync_size: u8) -> u8 {
        match self {
            Self::SyncOff => 0x00,
            Self::FifoFillAuto { sync_tolerance } => {
                0x80 | 0x00 | (sync_size.clamp(1, 8) - 1) << 3 | sync_tolerance.clamp(0, 7)
            }
            Self::FifoFillManual { sync_tolerance } => {
                0x80 | 0x40 | (sync_size.clamp(1, 8) - 1) << 3 | sync_tolerance.clamp(0, 7)
            }
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_sync_configuration() {
        let table_test: [(SyncConfiguration, u8, u8); 4] = [
            (SyncConfiguration::SyncOff, 8, 0x00),
            (
                SyncConfiguration::FifoFillAuto { sync_tolerance: 0 },
                8,
                184,
            ),
            (
                SyncConfiguration::FifoFillManual { sync_tolerance: 0 },
                8,
                248,
            ),
            (
                SyncConfiguration::FifoFillManual { sync_tolerance: 7 },
                8,
                255,
            ),
        ];

        table_test.into_iter().for_each(|test_case| {
            assert_eq!(test_case.0.value(test_case.1), test_case.2);
        });
    }
}

pub struct ModemConfig {
    reg_02: u8,
    reg_03: u8,
    reg_04: u8,
    reg_05: u8,
    reg_06: u8,
    reg_19: u8,
    reg_1a: u8,
    reg_37: u8,
}

const RF_DATAMODUL_DATAMODE_PACKET: u8 = 0x00;
const RF_DATAMODUL_MODULATIONTYPE_FSK: u8 = 0x00;
const RF_DATAMODUL_MODULATIONTYPE_OOK: u8 = 0x08;

const RF_DATAMODUL_MODULATIONSHAPING_00: u8 = 0x00;
const RF_DATAMODUL_MODULATIONSHAPING_01: u8 = 0x01;

const CONFIG_FSK: u8 = RF_DATAMODUL_DATAMODE_PACKET
    | RF_DATAMODUL_MODULATIONTYPE_FSK
    | RF_DATAMODUL_MODULATIONSHAPING_00;

const CONFIG_GFSK: u8 = RF_DATAMODUL_DATAMODE_PACKET
    | RF_DATAMODUL_MODULATIONTYPE_FSK
    | RF_DATAMODUL_MODULATIONSHAPING_01;

const CONFIG_OOK: u8 = RF_DATAMODUL_DATAMODE_PACKET
    | RF_DATAMODUL_MODULATIONTYPE_OOK
    | RF_DATAMODUL_MODULATIONSHAPING_00;

const RH_RF69_PACKETCONFIG1_PACKETFORMAT_VARIABLE: u8 = 0x80;

const RH_RF69_PACKETCONFIG1_DCFREE_NONE: u8 = 0x00;
const RH_RF69_PACKETCONFIG1_DCFREE_WHITENING: u8 = 0x40;
const RH_RF69_PACKETCONFIG1_DCFREE_MANCHESTER: u8 = 0x20;

const RH_RF69_PACKETCONFIG1_CRC_ON: u8 = 0x10;
const RH_RF69_PACKETCONFIG1_ADDRESSFILTERING_NONE: u8 = 0x00;

const CONFIG_NOWHITE: u8 = RH_RF69_PACKETCONFIG1_PACKETFORMAT_VARIABLE
    | RH_RF69_PACKETCONFIG1_DCFREE_NONE
    | RH_RF69_PACKETCONFIG1_CRC_ON
    | RH_RF69_PACKETCONFIG1_ADDRESSFILTERING_NONE;

const CONFIG_WHITE: u8 = RH_RF69_PACKETCONFIG1_PACKETFORMAT_VARIABLE
    | RH_RF69_PACKETCONFIG1_DCFREE_WHITENING
    | RH_RF69_PACKETCONFIG1_CRC_ON
    | RH_RF69_PACKETCONFIG1_ADDRESSFILTERING_NONE;

const CONFIG_MANCHESTER: u8 = RH_RF69_PACKETCONFIG1_PACKETFORMAT_VARIABLE
    | RH_RF69_PACKETCONFIG1_DCFREE_MANCHESTER
    | RH_RF69_PACKETCONFIG1_CRC_ON
    | RH_RF69_PACKETCONFIG1_ADDRESSFILTERING_NONE;

/// Predefined modem configuration presets.
///
/// Each preset configures the modulation type, bitrate, and frequency deviation
/// (or bandwidth for OOK). All presets use whitening for improved performance.
///
/// # Naming Convention
///
/// - `Fsk` / `Gfsk` / `Ook` - Modulation type
/// - `Rb` - Bitrate in kbps
/// - `Fd` - Frequency deviation in kHz (FSK/GFSK)
/// - `Bw` - Receiver bandwidth in kHz (OOK)
///
/// # Example
///
/// ```ignore
/// use rfm69_rs::settings::ModemConfigChoice;
///
/// // Use GFSK at 250kbps with 250kHz deviation
/// rfm69.set_modem_config(ModemConfigChoice::GfskRb250Fd250).await?;
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModemConfigChoice {
    /// FSK, Whitening, Rb = 2kbs, Fd = 5kHz
    FskRb2Fd5,
    /// FSK, Whitening, Rb = 2.4kbs, Fd = 4.8kHz
    FskRb2_4Fd4_8,
    /// FSK, Whitening, Rb = 4.8kbs, Fd = 9.6kHz
    FskRb4_8Fd9_6,
    /// FSK, Whitening, Rb = 9.6kbs, Fd = 19.2kHz
    FskRb9_6Fd19_2,
    /// FSK, Whitening, Rb = 19.2kbs, Fd = 38.4kHz
    FskRb19_2Fd38_4,
    /// FSK, Whitening, Rb = 38.4kbs, Fd = 76.8kHz
    FskRb38_4Fd76_8,
    /// FSK, Whitening, Rb = 57.6kbs, Fd = 120kHz
    FskRb57_6Fd120,
    /// FSK, Whitening, Rb = 125kbs, Fd = 125kHz
    FskRb125Fd125,
    /// FSK, Whitening, Rb = 250kbs, Fd = 250kHz
    FskRb250Fd250,
    /// FSK, Whitening, Rb = 55.555kbs, Fd = 50kHz
    FskRb55555Fd50,

    /// GFSK, Whitening, Rb = 2kbs, Fd = 5kHz
    GfskRb2Fd5,
    /// GFSK, Whitening, Rb = 2.4kbs, Fd = 4.8kHz
    GfskRb2_4Fd4_8,
    /// GFSK, Whitening, Rb = 4.8kbs, Fd = 9.6kHz
    GfskRb4_8Fd9_6,
    /// GFSK, Whitening, Rb = 9.6kbs, Fd = 19.2kHz
    GfskRb9_6Fd19_2,
    /// GFSK, Whitening, Rb = 19.2kbs, Fd = 38.4kHz
    GfskRb19_2Fd38_4,
    /// GFSK, Whitening, Rb = 38.4kbs, Fd = 76.8kHz
    GfskRb38_4Fd76_8,
    /// GFSK, Whitening, Rb = 57.6kbs, Fd = 120kHz
    GfskRb57_6Fd120,
    /// GFSK, Whitening, Rb = 125kbs, Fd = 125kHz
    GfskRb125Fd125,
    /// GFSK, Whitening, Rb = 250kbs, Fd = 250kHz
    GfskRb250Fd250,
    /// GFSK, Whitening, Rb = 55.555kbs, Fd = 50kHz
    GfskRb55555Fd50,

    /// OOK, Whitening, Rb = 1kbs, Rx Bandwidth = 1kHz
    OokRb1Bw1,
    /// OOK, Whitening, Rb = 1.2kbs, Rx Bandwidth = 75kHz
    OokRb1_2Bw75,
    /// OOK, Whitening, Rb = 2.4kbs, Rx Bandwidth = 4.8kHz
    OokRb2_4Bw4_8,
    /// OOK, Whitening, Rb = 4.8kbs, Rx Bandwidth = 9.6kHz
    OokRb4_8Bw9_6,
    /// OOK, Whitening, Rb = 9.6kbs, Rx Bandwidth = 19.2kHz
    OokRb9_6Bw19_2,
    /// OOK, Whitening, Rb = 19.2kbs, Rx Bandwidth = 38.4kHz
    OokRb19_2Bw38_4,
    /// OOK, Whitening, Rb = 32kbs, Rx Bandwidth = 64kHz
    OokRb32Bw64,
}

impl ModemConfigChoice {
    const FSK_RB2_FD5: [u8; 8] = [CONFIG_FSK, 0x3e, 0x80, 0x00, 0x52, 0xf4, 0xf4, CONFIG_WHITE];
    const FSK_RB2_4FD4_8: [u8; 8] = [CONFIG_FSK, 0x34, 0x15, 0x00, 0x4f, 0xf4, 0xf4, CONFIG_WHITE];
    const FSK_RB4_8FD9_6: [u8; 8] = [CONFIG_FSK, 0x1a, 0x0b, 0x00, 0x9d, 0xf4, 0xf4, CONFIG_WHITE];

    const FSK_RB9_6FD19_2: [u8; 8] = [CONFIG_FSK, 0x0d, 0x05, 0x01, 0x3b, 0xf4, 0xf4, CONFIG_WHITE];
    const FSK_RB19_2FD38_4: [u8; 8] =
        [CONFIG_FSK, 0x06, 0x83, 0x02, 0x75, 0xf3, 0xf3, CONFIG_WHITE];
    const FSK_RB38_4FD76_8: [u8; 8] =
        [CONFIG_FSK, 0x03, 0x41, 0x04, 0xea, 0xf2, 0xf2, CONFIG_WHITE];

    const FSK_RB57_6FD120: [u8; 8] = [CONFIG_FSK, 0x02, 0x2c, 0x07, 0xae, 0xe2, 0xe2, CONFIG_WHITE];
    const FSK_RB125FD125: [u8; 8] = [CONFIG_FSK, 0x01, 0x00, 0x08, 0x00, 0xe1, 0xe1, CONFIG_WHITE];
    const FSK_RB250FD250: [u8; 8] = [CONFIG_FSK, 0x00, 0x80, 0x10, 0x00, 0xe0, 0xe0, CONFIG_WHITE];
    const FSK_RB55555FD50: [u8; 8] = [CONFIG_FSK, 0x02, 0x40, 0x03, 0x33, 0x42, 0x42, CONFIG_WHITE];

    const GFSK_RB2_FD5: [u8; 8] = [
        CONFIG_GFSK,
        0x3e,
        0x80,
        0x00,
        0x52,
        0xf4,
        0xf5,
        CONFIG_WHITE,
    ];
    const GFSK_RB2_4FD4_8: [u8; 8] = [
        CONFIG_GFSK,
        0x34,
        0x15,
        0x00,
        0x4f,
        0xf4,
        0xf4,
        CONFIG_WHITE,
    ];
    const GFSK_RB4_8FD9_6: [u8; 8] = [
        CONFIG_GFSK,
        0x1a,
        0x0b,
        0x00,
        0x9d,
        0xf4,
        0xf4,
        CONFIG_WHITE,
    ];

    const GFSK_RB9_6FD19_2: [u8; 8] = [
        CONFIG_GFSK,
        0x0d,
        0x05,
        0x01,
        0x3b,
        0xf4,
        0xf4,
        CONFIG_WHITE,
    ];
    const GFSK_RB19_2FD38_4: [u8; 8] = [
        CONFIG_GFSK,
        0x06,
        0x83,
        0x02,
        0x75,
        0xf3,
        0xf3,
        CONFIG_WHITE,
    ];
    const GFSK_RB38_4FD76_8: [u8; 8] = [
        CONFIG_GFSK,
        0x03,
        0x41,
        0x04,
        0xea,
        0xf2,
        0xf2,
        CONFIG_WHITE,
    ];

    const GFSK_RB57_6FD120: [u8; 8] = [
        CONFIG_GFSK,
        0x02,
        0x2c,
        0x07,
        0xae,
        0xe2,
        0xe2,
        CONFIG_WHITE,
    ];
    const GFSK_RB125FD125: [u8; 8] = [
        CONFIG_GFSK,
        0x01,
        0x00,
        0x08,
        0x00,
        0xe1,
        0xe1,
        CONFIG_WHITE,
    ];
    const GFSK_RB250FD250: [u8; 8] = [
        CONFIG_GFSK,
        0x00,
        0x80,
        0x10,
        0x00,
        0xe0,
        0xe0,
        CONFIG_WHITE,
    ];
    const GFSK_RB55555FD50: [u8; 8] = [
        CONFIG_GFSK,
        0x02,
        0x40,
        0x03,
        0x33,
        0x42,
        0x42,
        CONFIG_WHITE,
    ];

    const OOK_RB1_BW1: [u8; 8] = [CONFIG_OOK, 0x7d, 0x00, 0x00, 0x10, 0x88, 0x88, CONFIG_WHITE];
    const OOK_RB1_2BW75: [u8; 8] = [CONFIG_OOK, 0x68, 0x2b, 0x00, 0x10, 0xf1, 0xf1, CONFIG_WHITE];
    const OOK_RB2_4BW4_8: [u8; 8] = [CONFIG_OOK, 0x34, 0x15, 0x00, 0x10, 0xf5, 0xf5, CONFIG_WHITE];

    const OOK_RB4_8BW9_6: [u8; 8] = [CONFIG_OOK, 0x1a, 0x0b, 0x00, 0x10, 0xf4, 0xf4, CONFIG_WHITE];
    const OOK_RB9_6BW19_2: [u8; 8] = [CONFIG_OOK, 0x0d, 0x05, 0x00, 0x10, 0xf3, 0xf3, CONFIG_WHITE];
    const OOK_RB19_2BW38_4: [u8; 8] =
        [CONFIG_OOK, 0x06, 0x83, 0x00, 0x10, 0xf2, 0xf2, CONFIG_WHITE];
    const OOK_RB32BW64: [u8; 8] = [CONFIG_OOK, 0x03, 0xe8, 0x00, 0x10, 0xe2, 0xe2, CONFIG_WHITE];

    pub fn values(&self) -> &[u8; 8] {
        match self {
            Self::FskRb2Fd5 => &Self::FSK_RB2_FD5,
            Self::FskRb2_4Fd4_8 => &Self::FSK_RB2_4FD4_8,
            Self::FskRb4_8Fd9_6 => &Self::FSK_RB4_8FD9_6,
            Self::FskRb9_6Fd19_2 => &Self::FSK_RB9_6FD19_2,
            Self::FskRb19_2Fd38_4 => &Self::FSK_RB19_2FD38_4,
            Self::FskRb38_4Fd76_8 => &Self::FSK_RB38_4FD76_8,
            Self::FskRb57_6Fd120 => &Self::FSK_RB57_6FD120,
            Self::FskRb125Fd125 => &Self::FSK_RB125FD125,
            Self::FskRb250Fd250 => &Self::FSK_RB250FD250,
            Self::FskRb55555Fd50 => &Self::FSK_RB55555FD50,

            Self::GfskRb2Fd5 => &Self::GFSK_RB2_FD5,
            Self::GfskRb2_4Fd4_8 => &Self::GFSK_RB2_4FD4_8,
            Self::GfskRb4_8Fd9_6 => &Self::GFSK_RB4_8FD9_6,
            Self::GfskRb9_6Fd19_2 => &Self::GFSK_RB9_6FD19_2,
            Self::GfskRb19_2Fd38_4 => &Self::GFSK_RB19_2FD38_4,
            Self::GfskRb38_4Fd76_8 => &Self::GFSK_RB38_4FD76_8,
            Self::GfskRb57_6Fd120 => &Self::GFSK_RB57_6FD120,
            Self::GfskRb125Fd125 => &Self::GFSK_RB125FD125,
            Self::GfskRb250Fd250 => &Self::GFSK_RB250FD250,
            Self::GfskRb55555Fd50 => &Self::GFSK_RB55555FD50,

            Self::OokRb1Bw1 => &Self::OOK_RB1_BW1,
            Self::OokRb1_2Bw75 => &Self::OOK_RB1_2BW75,
            Self::OokRb2_4Bw4_8 => &Self::OOK_RB2_4BW4_8,
            Self::OokRb4_8Bw9_6 => &Self::OOK_RB4_8BW9_6,
            Self::OokRb9_6Bw19_2 => &Self::OOK_RB9_6BW19_2,
            Self::OokRb19_2Bw38_4 => &Self::OOK_RB19_2BW38_4,
            Self::OokRb32Bw64 => &Self::OOK_RB32BW64,
        }
    }
}

pub const RF_PALEVEL_PA0_ON: u8 = 0x80;
pub const RF_PALEVEL_PA1_ON: u8 = 0x40;
pub const RF_PALEVEL_PA2_ON: u8 = 0x20;

pub const RF_PALEVEL_OUTPUTPOWER_11111: u8 = 0x1f;

pub const RF_DIOMAPPING1_DIO0_00: u8 = 0x00;
pub const RF_DIOMAPPING1_DIO0_01: u8 = 0x40;

//! RFM69 register definitions.
//!
//! This module defines all the register addresses for the RFM69 radio module.
//! Each register is documented with its purpose.

#[allow(dead_code)]
#[derive(Copy, Clone)]
pub enum Register {
    Fifo = 0x00,           // FIFO register: used for read/write access to the FIFO buffer.
    OpMode = 0x01,         // Operating modes of the transceiver.
    DataModul = 0x02,      // Data operation mode and modulation settings.
    BitrateMsb = 0x03,     // Bitrate most significant byte.
    BitrateLsb = 0x04,     // Bitrate least significant byte.
    FdevMsb = 0x05,        // Frequency deviation most significant byte.
    FdevLsb = 0x06,        // Frequency deviation least significant byte.
    FrfMsb = 0x07,         // Frequency most significant byte.
    FrfMid = 0x08,         // Frequency middle byte.
    FrfLsb = 0x09,         // Frequency least significant byte.
    Osc1 = 0x0A,           // Oscillator settings.
    AfcCtrl = 0x0B,        // AFC control settings.
    LowBat = 0x0C,         // Low battery detector threshold.
    Listen1 = 0x0D,        // Listen mode settings.
    Listen2 = 0x0E,        // Listen mode duration.
    Listen3 = 0x0F,        // Listen mode frequency.
    Version = 0x10,        // Chip version.
    PaLevel = 0x11,        // Output power control.
    PaRamp = 0x12,         // Power amplifier ramping.
    Ocp = 0x13,            // Overcurrent protection.
    AgcRef = 0x14,         // AGC reference level.
    AgcThresh1 = 0x15,     // AGC threshold 1.
    AgcThresh2 = 0x16,     // AGC threshold 2.
    AgcThresh3 = 0x17,     // AGC threshold 3.
    Lna = 0x18,            // Low-noise amplifier settings.
    RxBw = 0x19,           // Receiver bandwidth.
    AfcBw = 0x1A,          // AFC bandwidth.
    OokPeak = 0x1B,        // OOK demodulator peak settings.
    OokAvg = 0x1C,         // OOK demodulator average settings.
    OokFix = 0x1D,         // OOK demodulator fixed threshold.
    AfcFei = 0x1E,         // AFC and frequency error indicator.
    AfcMsb = 0x1F,         // AFC most significant byte.
    AfcLsb = 0x20,         // AFC least significant byte.
    FeiMsb = 0x21,         // Frequency error most significant byte.
    FeiLsb = 0x22,         // Frequency error least significant byte.
    RssiConfig = 0x23,     // RSSI configuration.
    RssiValue = 0x24,      // RSSI value.
    DioMapping1 = 0x25,    // Mapping of pins DIO0 to DIO3.
    DioMapping2 = 0x26,    // Mapping of pins DIO4 and DIO5.
    IrqFlags1 = 0x27,      // Interrupt flags 1.
    IrqFlags2 = 0x28,      // Interrupt flags 2.
    RssiThresh = 0x29,     // RSSI threshold.
    RxTimeout1 = 0x2A,     // Timeout duration for RxStart.
    RxTimeout2 = 0x2B,     // Timeout duration for RSSI detection.
    PreambleMsb = 0x2C,    // Preamble length most significant byte.
    PreambleLsb = 0x2D,    // Preamble length least significant byte.
    SyncConfig = 0x2E,     // Sync word configuration.
    SyncValue1 = 0x2F,     // Sync word value byte 1.
    SyncValue2 = 0x30,     // Sync word value byte 2.
    SyncValue3 = 0x31,     // Sync word value byte 3.
    SyncValue4 = 0x32,     // Sync word value byte 4.
    SyncValue5 = 0x33,     // Sync word value byte 5.
    SyncValue6 = 0x34,     // Sync word value byte 6.
    SyncValue7 = 0x35,     // Sync word value byte 7.
    SyncValue8 = 0x36,     // Sync word value byte 8.
    PacketConfig1 = 0x37,  // Packet configuration 1.
    PayloadLength = 0x38,  // Payload length.
    NodeAddrs = 0x39,      // Node address.
    BroadcastAddrs = 0x3A, // Broadcast address.
    AutoModes = 0x3B,      // Auto modes configuration.
    FifoThresh = 0x3C,     // FIFO threshold.
    PacketConfig2 = 0x3D,  // Packet configuration 2.
    AesKey1 = 0x3E,        // AES key byte 1.
    AesKey2 = 0x3F,        // AES key byte 2.
    AesKey3 = 0x40,        // AES key byte 3.
    AesKey4 = 0x41,        // AES key byte 4.
    AesKey5 = 0x42,        // AES key byte 5.
    AesKey6 = 0x43,        // AES key byte 6.
    AesKey7 = 0x44,        // AES key byte 7.
    AesKey8 = 0x45,        // AES key byte 8.
    AesKey9 = 0x46,        // AES key byte 9.
    AesKey10 = 0x47,       // AES key byte 10.
    AesKey11 = 0x48,       // AES key byte 11.
    AesKey12 = 0x49,       // AES key byte 12.
    AesKey13 = 0x4A,       // AES key byte 13.
    AesKey14 = 0x4B,       // AES key byte 14.
    AesKey15 = 0x4C,       // AES key byte 15.
    AesKey16 = 0x4D,       // AES key byte 16.
    Temp1 = 0x4E,          // Temperature sensor control.
    Temp2 = 0x4F,          // Temperature sensor value.
    TestLna = 0x58,        // Test LNA settings.
    TestPa1 = 0x5A,        // Test PA1 control.
    TestPa2 = 0x5C,        // Test PA2 control.
    TestDagc = 0x6F,       // Test DAGC settings.
    TestAfc = 0x71,        // Test AFC settings.
}

const READ_MASK: u8 = 0x7F;
const WRITE_MASK: u8 = 0x80;

impl Register {
    #[inline]
    pub fn read(self) -> u8 {
        (self as u8) & READ_MASK
    }

    #[inline]
    pub fn write(self) -> u8 {
        (self as u8) | WRITE_MASK
    }

    #[inline]
    pub fn addr(self) -> u8 {
        self as u8
    }
}

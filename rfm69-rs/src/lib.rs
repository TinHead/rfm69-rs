//! # RFM69 Radio Driver
//!
//! An async, `no_std` compatible driver for the RFM69 series of radio modules.
//!
//! This crate provides a high-level interface for the RFM69 radio transceiver
//! using the `embedded-hal-async` traits for hardware abstraction.
//!
//! ## Features
//!
//! - Async SPI communication
//! - IRQ-based message waiting (no polling)
//! - Configurable modulation, bitrate, and frequency
//! - Support for both standard and high-power (RFM69HW) modules
//!
//! ## Example
//!
//! ```ignore
//! use rfm69_rs::rfm69::{Rfm69, Rfm69Mode};
//!
//! let mut radio = Rfm69::new(spi_device, reset_pin, dio0_pin, delay);
//! radio.init().await?;
//! radio.set_frequency(915).await?;
//!
//! // Send a message
//! radio.send(b"Hello!").await?;
//!
//! // Receive a message
//! radio.set_mode(Rfm69Mode::Rx).await?;
//! radio.wait_for_message().await?;
//! let mut buf = [0u8; 65];
//! let len = radio.receive(&mut buf).await?;
//! ```

#![cfg_attr(not(test), no_std)]

pub mod registers;
pub mod rfm69;
pub mod settings;

mod read_write;

// Re-export commonly used types for convenience
pub use rfm69::{Rfm69, Rfm69Config, Rfm69Error, Rfm69Mode};
pub use settings::{ModemConfigChoice, SyncConfiguration};

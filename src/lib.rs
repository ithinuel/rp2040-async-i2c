//! Implementation of the embedded-hal-async i2c traits for the Raspberry Pi's RP2040 chip.
#![no_std]
#![forbid(missing_docs)]
#![allow(incomplete_features)]
#![feature(async_fn_in_trait)]

/// Implementation for the Synopsys I2C peripheral as implemented on the RP2040 chip.
pub mod i2c;
/// Implementation for a PIO backed I2C peripheral.
#[cfg(feature = "pio")]
pub mod pio;

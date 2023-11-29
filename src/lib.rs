//! Implementation of the embedded-hal-async i2c traits for the Raspberry Pi's RP2040 chip.
#![no_std]
#![forbid(missing_docs)]

macro_rules! block_on {
    ($me:ident, $is_ready:ident ($($arg:expr),*), $setup_flags:ident ($($arg2:expr),*)) => {
        future::poll_fn(|cx| {
            let r = $me.$is_ready($($arg),*);

            if r.is_pending() {
                if let Some(waker_setter) = $me.waker_setter {
                    waker_setter(cx.waker().clone());
                    $me.$setup_flags($($arg2),*);
                } else {
                    // always ready to scan
                    cx.waker().wake_by_ref();
                }
            }
            r
        })
    };
}

/// Implementation for the Synopsys I2C peripheral as implemented on the RP2040 chip.
pub mod i2c;
/// Implementation for a PIO backed I2C peripheral.
#[cfg(feature = "pio")]
pub mod pio;

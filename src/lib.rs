#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(generic_associated_types)]

use core::{future::Future, ops::Deref, task::Poll};

use embedded_hal_async::i2c::Operation;
use fugit::HertzU32;
use rp2040_hal::{
    gpio::{bank0::BankPinId, FunctionI2C, Pin, PinId},
    i2c::{Error, SclPin, SdaPin},
    pac::{self, i2c0::RegisterBlock as Block, RESETS},
};

mod sealed {
    use rp2040_hal::pac;
    pub trait SubSystemReset {
        fn reset_bring_up(&self, resets: &mut pac::RESETS);
        fn reset_bring_down(&self, resets: &mut pac::RESETS);
    }
}

macro_rules! generate_reset {
    ($MODULE:ident, $module:ident) => {
        impl sealed::SubSystemReset for pac::$MODULE {
            fn reset_bring_up(&self, resets: &mut pac::RESETS) {
                resets.reset.modify(|_, w| w.$module().clear_bit());
                while resets.reset_done.read().$module().bit_is_clear() {}
            }
            fn reset_bring_down(&self, resets: &mut pac::RESETS) {
                resets.reset.modify(|_, w| w.$module().set_bit());
            }
        }
    };
}
generate_reset!(I2C0, i2c0);
generate_reset!(I2C1, i2c1);

pub(crate) use sealed::SubSystemReset;

const TX_FIFO_SIZE: u8 = 16;
const RX_FIFO_SIZE: u8 = 16;

pub struct AsyncI2C<'a, I2C, Pins> {
    i2c: I2C,
    pins: Pins,
    waker_setter: Option<fn(core::task::Waker)>,
    _clock: &'a rp2040_hal::clocks::SystemClock,
}

impl<T: Deref<Target = Block>, PINS> embedded_hal_async::i2c::ErrorType for AsyncI2C<'_, T, PINS> {
    type Error = Error;
}
impl<
        'a,
        T: SubSystemReset + Deref<Target = Block>,
        Sda: PinId + BankPinId,
        Scl: PinId + BankPinId,
    > AsyncI2C<'a, T, (Pin<Sda, FunctionI2C>, Pin<Scl, FunctionI2C>)>
{
    /// Configures the I2C peripheral to work in controller mode
    pub fn new(
        i2c: T,
        sda_pin: Pin<Sda, FunctionI2C>,
        scl_pin: Pin<Scl, FunctionI2C>,
        freq: HertzU32,
        resets: &mut RESETS,
        system_clock: &'a rp2040_hal::clocks::SystemClock,
    ) -> Self
    where
        Sda: SdaPin<T>,
        Scl: SclPin<T>,
    {
        let freq = freq.to_Hz();
        assert!(freq <= 1_000_000);
        assert!(freq > 0);

        i2c.reset_bring_down(resets);
        i2c.reset_bring_up(resets);

        i2c.ic_enable.write(|w| w.enable().disabled());

        // select controller mode & speed
        i2c.ic_con.modify(|_, w| {
            w.speed().fast();
            w.master_mode().enabled();
            w.ic_slave_disable().slave_disabled();
            w.ic_restart_en().enabled();
            w.tx_empty_ctrl().enabled()
        });

        // Clear FIFO threshold
        i2c.ic_tx_tl.write(|w| unsafe { w.tx_tl().bits(0) });
        i2c.ic_rx_tl.write(|w| unsafe { w.rx_tl().bits(0) });

        use rp2040_hal::Clock;
        let freq_in = system_clock.freq().to_Hz();

        // There are some subtleties to I2C timing which we are completely ignoring here
        // See: https://github.com/raspberrypi/pico-sdk/blob/bfcbefafc5d2a210551a4d9d80b4303d4ae0adf7/src/rp2_common/hardware_i2c/i2c.c#L69
        let period = (freq_in + freq / 2) / freq;
        let lcnt = period * 3 / 5; // spend 3/5 (60%) of the period low
        let hcnt = period - lcnt; // and 2/5 (40%) of the period high

        // Check for out-of-range divisors:
        assert!(hcnt <= 0xffff);
        assert!(lcnt <= 0xffff);
        assert!(hcnt >= 8);
        assert!(lcnt >= 8);

        // Per I2C-bus specification a device in standard or fast mode must
        // internally provide a hold time of at least 300ns for the SDA signal to
        // bridge the undefined region of the falling edge of SCL. A smaller hold
        // time of 120ns is used for fast mode plus.
        let sda_tx_hold_count = if freq < 1000000 {
            // sda_tx_hold_count = freq_in [cycles/s] * 300ns * (1s / 1e9ns)
            // Reduce 300/1e9 to 3/1e7 to avoid numbers that don't fit in uint.
            // Add 1 to avoid division truncation.
            ((freq_in * 3) / 10000000) + 1
        } else {
            // fast mode plus requires a clk_in > 32MHz
            assert!(freq_in >= 32_000_000);

            // sda_tx_hold_count = freq_in [cycles/s] * 120ns * (1s / 1e9ns)
            // Reduce 120/1e9 to 3/25e6 to avoid numbers that don't fit in uint.
            // Add 1 to avoid division truncation.
            ((freq_in * 3) / 25000000) + 1
        };
        assert!(sda_tx_hold_count <= lcnt - 2);

        unsafe {
            i2c.ic_fs_scl_hcnt
                .write(|w| w.ic_fs_scl_hcnt().bits(hcnt as u16));
            i2c.ic_fs_scl_lcnt
                .write(|w| w.ic_fs_scl_lcnt().bits(lcnt as u16));
            i2c.ic_fs_spklen.write(|w| {
                w.ic_fs_spklen()
                    .bits(if lcnt < 16 { 1 } else { (lcnt / 16) as u8 })
            });
            i2c.ic_sda_hold
                .modify(|_r, w| w.ic_sda_tx_hold().bits(sda_tx_hold_count as u16));
        }

        // Enable I2C block
        i2c.ic_enable.write(|w| w.enable().enabled());

        Self {
            i2c,
            pins: (sda_pin, scl_pin),
            waker_setter: None,
            _clock: system_clock,
        }
    }
}
impl<T, Sda, Scl> AsyncI2C<'_, T, (Pin<Sda, FunctionI2C>, Pin<Scl, FunctionI2C>)>
where
    T: SubSystemReset + Deref<Target = Block>,
    Sda: PinId + BankPinId,
    Scl: PinId + BankPinId,
{
    /// Releases the I2C peripheral and associated pins
    #[allow(clippy::type_complexity)]
    pub fn free(self, resets: &mut RESETS) -> (T, (Pin<Sda, FunctionI2C>, Pin<Scl, FunctionI2C>)) {
        self.i2c.reset_bring_down(resets);

        (self.i2c, self.pins)
    }
}

impl<'clk, T: Deref<Target = Block>, PINS> AsyncI2C<'clk, T, PINS> {
    async fn block_on<F: FnMut(&mut Self) -> Poll<U>, U>(&mut self, mut f: F) -> U {
        // if a waker is set enbale interrupt
        futures::future::poll_fn(|cx| {
            let r = f(self);

            if r.is_pending() {
                if let Some(waker_setter) = self.waker_setter {
                    waker_setter(cx.waker().clone());
                } else {
                    // always ready to scan
                    cx.waker().wake_by_ref();
                }
            }
            r
        })
        .await
    }

    pub fn set_waker_setter(&mut self, waker_setter: fn(core::task::Waker)) {
        self.waker_setter = Some(waker_setter);
    }

    /// Number of bytes currently in the RX FIFO
    #[inline]
    pub fn rx_fifo_used(&self) -> u8 {
        self.i2c.ic_rxflr.read().rxflr().bits()
    }

    /// Remaining capacity in the RX FIFO
    #[inline]
    pub fn rx_fifo_free(&self) -> u8 {
        RX_FIFO_SIZE - self.rx_fifo_used()
    }

    /// RX FIFO is empty
    #[inline]
    pub fn rx_fifo_empty(&self) -> bool {
        self.rx_fifo_used() == 0
    }

    /// Number of bytes currently in the TX FIFO
    #[inline]
    pub fn tx_fifo_used(&self) -> u8 {
        self.i2c.ic_txflr.read().txflr().bits()
    }

    /// Remaining capacity in the TX FIFO
    #[inline]
    pub fn tx_fifo_free(&self) -> u8 {
        TX_FIFO_SIZE - self.tx_fifo_used()
    }

    /// TX FIFO is at capacity
    #[inline]
    pub fn tx_fifo_full(&self) -> bool {
        self.tx_fifo_free() == 0
    }

    fn validate(
        addr: u16,
        opt_tx_empty: Option<bool>,
        opt_rx_empty: Option<bool>,
    ) -> Result<(), Error> {
        // validate tx parameters if present
        if opt_tx_empty.unwrap_or(false) {
            return Err(Error::InvalidWriteBufferLength);
        }

        // validate rx parameters if present
        if opt_rx_empty.unwrap_or(false) {
            return Err(Error::InvalidReadBufferLength);
        }

        // validate address
        if addr >= 0x80 {
            Err(Error::AddressOutOfRange(addr))
        } else if (addr & 0x78) == 0 || (addr & 0x78) == 0x78 {
            Err(Error::AddressReserved(addr))
        } else {
            Ok(())
        }
    }

    fn setup(&mut self, addr: u16) {
        self.i2c.ic_enable.write(|w| w.enable().disabled());
        self.i2c.ic_tar.write(|w| unsafe { w.ic_tar().bits(addr) });
        self.i2c.ic_enable.write(|w| w.enable().enabled());
    }

    fn read_and_clear_abort_reason(&mut self) -> Option<u32> {
        let abort_reason = self.i2c.ic_tx_abrt_source.read().bits();
        if abort_reason != 0 {
            // Note clearing the abort flag also clears the reason, and
            // this instance of flag is clear-on-read! Note also the
            // IC_CLR_TX_ABRT register always reads as 0.
            self.i2c.ic_clr_tx_abrt.read();
            Some(abort_reason)
        } else {
            None
        }
    }

    async fn non_blocking_read_internal(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        let mut received = 0;
        let mut queued = 0;
        let mut abort_reason = None;
        while received < buffer.len() {
            while queued < buffer.len() && !self.tx_fifo_full() {
                self.i2c.ic_data_cmd.write(|w| {
                    if queued == 0 {
                        w.restart().enable();
                    } else {
                        w.restart().disable();
                    }

                    if (queued + 1) == buffer.len() {
                        w.stop().enable();
                    } else {
                        w.stop().disable();
                    }

                    w.cmd().read()
                });
                queued += 1;
            }

            match self
                .block_on(|me| {
                    if let Some(abort_reason) = me.read_and_clear_abort_reason() {
                        Poll::Ready(Err(abort_reason))
                    } else if me.i2c.ic_rxflr.read().bits() != 0 {
                        Poll::Ready(Ok(()))
                    } else {
                        me.i2c
                            .ic_intr_mask
                            .modify(|_, w| w.m_rx_full().disabled().m_tx_abrt().disabled());
                        Poll::Pending
                    }
                })
                .await
            {
                Ok(_) => {}
                Err(reason) => {
                    abort_reason = Some(reason);
                    break;
                }
            }

            buffer[received] = self.i2c.ic_data_cmd.read().dat().bits();
            received += 1;
        }

        // wait for stop condition to be emitted.
        self.block_on(|me| {
            if me.i2c.ic_raw_intr_stat.read().stop_det().is_inactive() {
                me.i2c
                    .ic_intr_mask
                    .modify(|_, w| w.m_rx_full().disabled().m_tx_abrt().disabled());
                Poll::Pending
            } else {
                Poll::Ready(())
            }
        })
        .await;
        self.i2c.ic_clr_stop_det.read().clr_stop_det();

        if let Some(abort_reason) = abort_reason {
            return Err(Error::Abort(abort_reason));
        }
        Ok(())
    }

    async fn non_blocking_write_internal(
        &mut self,
        bytes: impl IntoIterator<Item = u8>,
        do_stop: bool,
    ) -> Result<(), Error> {
        let mut bytes = bytes.into_iter().peekable();
        let mut abort_reason = None;
        while let Some(byte) = bytes.next() {
            if self.tx_fifo_full() {
                // wait a bit
                match self
                    .block_on(|me| {
                        if let Some(abort_reason) = me.read_and_clear_abort_reason() {
                            Poll::Ready(Err(abort_reason))
                        } else if !me.tx_fifo_full() {
                            Poll::Ready(Ok(()))
                        } else {
                            me.i2c
                                .ic_intr_mask
                                .modify(|_, w| w.m_tx_empty().disabled().m_tx_abrt().disabled());
                            Poll::Pending
                        }
                    })
                    .await
                {
                    Ok(_) => {}
                    Err(reason) => {
                        abort_reason = Some(reason);
                        break;
                    }
                };
            }

            // else enqueue
            let last = bytes.peek().is_none();
            self.i2c.ic_data_cmd.write(|w| {
                if do_stop && last {
                    w.stop().enable();
                } else {
                    w.stop().disable();
                }
                unsafe { w.dat().bits(byte) }
            });
        }
        if abort_reason.is_none() {
            // wait for the tx_fifo to be emptied
            self.block_on(|me| {
                if me.i2c.ic_raw_intr_stat.read().tx_empty().is_inactive() {
                    me.i2c
                        .ic_intr_mask
                        .modify(|_, w| w.m_tx_empty().disabled().m_tx_abrt().disabled());
                    Poll::Pending
                } else {
                    Poll::Ready(())
                }
            })
            .await;

            abort_reason = self.read_and_clear_abort_reason();
        }

        if abort_reason.is_some() || do_stop {
            // If the transaction was aborted or if it completed
            // successfully wait until the STOP condition has occured.
            self.block_on(|me| {
                if me.i2c.ic_raw_intr_stat.read().stop_det().is_inactive() {
                    me.i2c
                        .ic_intr_mask
                        .modify(|_, w| w.m_tx_empty().disabled().m_tx_abrt().disabled());
                    Poll::Pending
                } else {
                    Poll::Ready(())
                }
            })
            .await;

            self.i2c.ic_clr_stop_det.read().clr_stop_det();
        }

        // Note the hardware issues a STOP automatically on an abort condition.
        // Note also the hardware clears RX FIFO as well as TX on abort,
        // because we set hwparam IC_AVOID_RX_FIFO_FLUSH_ON_TX_ABRT to 0.
        if let Some(abort_reason) = abort_reason {
            return Err(Error::Abort(abort_reason));
        }

        Ok(())
    }

    pub fn write_iter<'a: 'clk, A, U>(
        &'a mut self,
        address: A,
        bytes: U,
    ) -> impl Future<Output = Result<(), Error>> + 'a
    where
        U: IntoIterator<Item = u8> + 'a,
        A: embedded_hal_async::i2c::AddressMode + 'static + Into<u16>,
    {
        let addr: u16 = address.into();
        let mut bytes = bytes.into_iter().peekable();

        async move {
            Self::validate(addr, Some(bytes.peek().is_none()), None)?;
            self.setup(addr);
            self.non_blocking_write_internal(bytes, true).await
        }
    }
}
impl<T, PINS, A> embedded_hal_async::i2c::I2c<A> for AsyncI2C<'_, T, PINS>
where
    T: Deref<Target = Block>,
    A: embedded_hal_async::i2c::AddressMode + 'static + Into<u16>,
{
    type WriteFuture<'a>
    = impl Future<Output = Result<(), Error>> + 'a where
        Self: 'a ;

    type ReadFuture<'a> = impl Future<Output = Result<(), Error>> + 'a
    where
        Self: 'a;

    type WriteReadFuture<'a> = impl Future<Output = Result<(), Error>> + 'a
    where
        Self: 'a;

    type TransactionFuture<'a, 'b> = impl Future<Output = Result<(), Error>> + 'a
        where Self: 'a, 'b: 'a;

    fn read<'a>(&'a mut self, address: A, buffer: &'a mut [u8]) -> Self::ReadFuture<'a> {
        let addr: u16 = address.into();

        async move {
            Self::validate(addr, None, Some(buffer.is_empty()))?;
            self.setup(addr);
            self.non_blocking_read_internal(buffer).await
        }
    }

    fn write<'a>(&'a mut self, address: A, bytes: &'a [u8]) -> Self::WriteFuture<'a> {
        let addr: u16 = address.into();

        async move {
            Self::validate(addr, Some(bytes.is_empty()), None)?;
            self.setup(addr);
            self.non_blocking_write_internal(bytes.iter().cloned(), true)
                .await
        }
    }

    fn write_read<'a>(
        &'a mut self,
        address: A,
        bytes: &'a [u8],
        buffer: &'a mut [u8],
    ) -> Self::WriteReadFuture<'a> {
        let addr: u16 = address.into();

        async move {
            Self::validate(addr, Some(bytes.is_empty()), Some(buffer.is_empty()))?;
            self.setup(addr);
            self.non_blocking_write_internal(bytes.iter().cloned(), false)
                .await?;
            self.non_blocking_read_internal(buffer).await
        }
    }

    fn transaction<'a, 'b>(
        &'a mut self,
        _: A,
        _: &'a mut [Operation<'b>],
    ) -> Self::TransactionFuture<'a, 'b> {
        async move { todo!() }
    }
}

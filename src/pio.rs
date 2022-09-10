use core::{future, future::Future, task::Poll};

use embedded_hal::i2c::{
    blocking::Operation, AddressMode, ErrorKind, NoAcknowledgeSource, SevenBitAddress,
    TenBitAddress,
};
use fugit::HertzU32;
use pio::{Instruction, InstructionOperands, SideSet};
use rp2040_hal::{
    gpio::{Disabled, DisabledConfig, Function, FunctionConfig, Pin, PinId, ValidPinMode},
    pio::{
        PIOExt, PinDir, PinState, PioIRQ, Rx, ShiftDirection, StateMachine, StateMachineIndex, Tx,
        UninitStateMachine, PIO,
    },
};

const SC0SD0: Instruction = Instruction {
    operands: pio::InstructionOperands::SET {
        destination: pio::SetDestination::PINDIRS,
        data: 0,
    },
    delay: 7,
    side_set: Some(0),
};
const SC0SD1: Instruction = Instruction {
    operands: pio::InstructionOperands::SET {
        destination: pio::SetDestination::PINDIRS,
        data: 1,
    },
    delay: 7,
    side_set: Some(0),
};
const SC1SD0: Instruction = Instruction {
    operands: pio::InstructionOperands::SET {
        destination: pio::SetDestination::PINDIRS,
        data: 0,
    },
    delay: 7,
    side_set: Some(1),
};
const SC1SD1: Instruction = Instruction {
    operands: pio::InstructionOperands::SET {
        destination: pio::SetDestination::PINDIRS,
        data: 1,
    },
    delay: 7,
    side_set: Some(1),
};

const SIDESET: SideSet = SideSet::new(true, 1, true);

const NAK_BIT: u16 = 0b0000_0000_0000_0001;
const FINAL_BIT: u16 = 0b0000_0010_0000_0000;
const INSTR_OFFSET: usize = 10;
const DATA_OFFSET: usize = 1;

/// Instance of I2C Controller.
pub struct I2C<'pio, P, SMI, SDA, SCL>
where
    P: PIOExt + FunctionConfig,
    SMI: StateMachineIndex,
    SDA: PinId,
    SCL: PinId,
    Function<P>: ValidPinMode<SDA> + ValidPinMode<SCL>,
{
    pio: &'pio mut PIO<P>,
    tx: Tx<(P, SMI)>,
    rx: Rx<(P, SMI)>,
    _sm: StateMachine<(P, SMI), rp2040_hal::pio::Running>,
    _sda: Pin<SDA, Function<P>>,
    _scl: Pin<SCL, Function<P>>,
    waker_setter: Option<fn(core::task::Waker)>,
}

impl<'pio, P, SMI, SDA, SCL> I2C<'pio, P, SMI, SDA, SCL>
where
    P: PIOExt + FunctionConfig,
    SMI: StateMachineIndex,
    SDA: PinId,
    SCL: PinId,
    Function<P>: ValidPinMode<SDA> + ValidPinMode<SCL>,
{
    /// Creates a new instance of this driver.
    ///
    /// Note: the PIO must have been reset before using this driver.
    pub fn new<SdaDisabledConfig: DisabledConfig, SclDisabledConfig: DisabledConfig>(
        pio: &'pio mut PIO<P>,
        sda: rp2040_hal::gpio::Pin<SDA, Disabled<SdaDisabledConfig>>,
        scl: rp2040_hal::gpio::Pin<SCL, Disabled<SclDisabledConfig>>,
        sm: UninitStateMachine<(P, SMI)>,
        bus_freq: HertzU32,
        clock_freq: HertzU32,
    ) -> Self
    where
        Function<P>: ValidPinMode<SDA> + ValidPinMode<SCL>,
    {
        // prepare the PIO program
        let mut a = pio::Assembler::<32>::new_with_side_set(SIDESET);

        let mut byte_send = a.label();
        let mut byte_nack = a.label();
        let mut byte_end = a.label();
        let mut bitloop = a.label();
        let mut wrap_target = a.label();
        let mut wrap_source = a.label();
        let mut do_exec = a.label();

        a.bind(&mut byte_nack);
        // continue if NAK was expected
        a.jmp(pio::JmpCondition::YDecNonZero, &mut byte_end);
        // In nack was not expected, park on IRQ (0+SM:id())%4 until the software handles it.
        a.irq(false, true, 0, true);
        a.jmp(pio::JmpCondition::Always, &mut byte_end);

        a.bind(&mut byte_send);
        // Unpack Final
        a.out(pio::OutDestination::Y, 1);
        // loop 8 times
        a.set(pio::SetDestination::X, 7);

        // Send 1 byte
        a.bind(&mut bitloop);
        // Serialize write data (all-ones is reading)
        a.out_with_delay(pio::OutDestination::PINDIRS, 1, 7);
        // SCL rising edge
        a.nop_with_delay_and_side_set(2, 1);
        // Allow clock to be stretched
        a.wait_with_delay(1, pio::WaitSource::GPIO, SCL::DYN.num, false, 4);
        // Sample read data in middle of SCL pulse
        a.in_with_delay(pio::InSource::PINS, 1, 7);
        // SCL falling edge
        a.jmp_with_delay_and_side_set(pio::JmpCondition::XDecNonZero, &mut bitloop, 7, 0);

        // handle ACK pulse
        // On reads, we provide the ACK, on writes, the pin is passively pulled up
        a.out_with_delay(pio::OutDestination::PINDIRS, 1, 7);
        // SCL risin edge
        a.nop_with_delay_and_side_set(7, 1);
        // Allow clock to be stretched
        a.wait_with_delay(1, pio::WaitSource::GPIO, SCL::DYN.num, false, 7);
        // Test SDA for ACK/NACK, fall through if ACK
        a.jmp_with_delay_and_side_set(pio::JmpCondition::PinHigh, &mut byte_nack, 2, 0);

        a.bind(&mut byte_end);
        // complete the operation by pushing the shift register to the fifo
        a.push(false, true);

        a.bind(&mut wrap_target);
        // Unpack Instr count
        a.out(pio::OutDestination::X, 6);
        // Instr == 0, this is a data record
        a.jmp(pio::JmpCondition::XIsZero, &mut byte_send);
        // Instr > 0, remainder of this OSR is invalid
        a.out(pio::OutDestination::NULL, 10);

        // special action sub function (Start/Restart/Stop sequences)
        a.bind(&mut do_exec);
        // Execute one instruction per FIFO word
        a.out(pio::OutDestination::EXEC, 16);
        a.jmp(pio::JmpCondition::XDecNonZero, &mut do_exec);
        a.bind(&mut wrap_source);

        let program = a.assemble_with_wrap(wrap_source, wrap_target);

        // Install the program into PIO instruction memory.
        let installed = pio.install(&program).unwrap();
        let wrap_target = installed.wrap_target();

        // Configure the PIO state machine.
        let bit_freq = 32 * bus_freq;
        let mut int = clock_freq / bit_freq;
        let rem = clock_freq - (int * bit_freq);
        let frac = (rem * 256) / bit_freq;

        assert!(
            (1..=65536).contains(&int) && (int != 65536 || frac == 0),
            "The ratio between the bus frequency and the system clock must be within [1.0, 65536.0]."
        );

        // 65536.0 is represented as 0 in the pio's clock divider
        if int == 65536 {
            int = 0;
        }
        // Using lossy conversion because range have been checked
        let int: u16 = int as u16;
        let frac: u8 = frac as u8;

        // init
        let (mut sm, rx, tx) = rp2040_hal::pio::PIOBuilder::from_program(installed)
            // use both RX & TX FIFO
            .buffers(rp2040_hal::pio::Buffers::RxTx)
            // Pin configuration
            .set_pins(SDA::DYN.num, 1)
            .out_pins(SDA::DYN.num, 1)
            .in_pin_base(SDA::DYN.num)
            .side_set_pin_base(SCL::DYN.num)
            .jmp_pin(SDA::DYN.num)
            // OSR config
            .out_shift_direction(rp2040_hal::pio::ShiftDirection::Left)
            .autopull(true)
            .pull_threshold(16)
            // ISR config
            .in_shift_direction(ShiftDirection::Left)
            .push_threshold(8)
            // clock config
            .clock_divisor_fixed_point(int, frac)
            .build(sm);

        // enable pull up on SDA & SCL: idle bus
        let sda = sda.into_pull_up_input();
        let scl = scl.into_pull_up_input();

        // This will pull the bus high for a little bit of time
        sm.set_pins([
            (SCL::DYN.num, PinState::High),
            (SDA::DYN.num, PinState::High),
        ]);
        sm.set_pindirs([
            (SCL::DYN.num, PinDir::Output),
            (SDA::DYN.num, PinDir::Output),
        ]);

        // attach SDA pin to pio
        let mut sda: Pin<SDA, Function<P>> = sda.into_mode();
        // configure SDA pin as inverted
        sda.set_output_enable_override(rp2040_hal::gpio::OutputEnableOverride::Invert);

        // attach SCL pin to pio
        let mut scl: Pin<SCL, Function<P>> = scl.into_mode();
        // configure SCL pin as inverted
        scl.set_output_enable_override(rp2040_hal::gpio::OutputEnableOverride::Invert);

        // the PIO now keeps the pin as Input, we can set the pin state to Low.
        sm.set_pins([(SDA::DYN.num, PinState::Low), (SCL::DYN.num, PinState::Low)]);

        // Set the state machine on the entry point.
        sm.exec_instruction(
            InstructionOperands::JMP {
                condition: pio::JmpCondition::Always,
                address: wrap_target,
            }
            .encode(),
        );

        // enable
        let sm = sm.start();

        Self {
            pio,
            tx,
            rx,
            _sm: sm,
            _sda: sda,
            _scl: scl,
            waker_setter: None,
        }
    }

    /// Sets the function pointer to be called to setup the waker context.
    ///
    /// Once set, the instance will rely on interrupts to wake. The interrup used is IRQ0 and the
    /// flags are:
    /// - `sm<SMI::id()>`
    /// - rx FIFO not empty
    /// - tx FIFO not full
    pub fn set_waker_setter(&mut self, waker_setter: fn(core::task::Waker)) {
        self.waker_setter = Some(waker_setter);
    }

    /// Calls `f` to check if we are ready or not.
    /// If not, `g` is called once the waker is set (to eg enable the required interrupts).
    async fn block_on<F, U, G>(&mut self, mut f: F, mut g: G) -> U
    where
        F: FnMut(&mut Self) -> Poll<U>,
        G: FnMut(&mut Self),
    {
        future::poll_fn(|cx| {
            let r = f(self);

            if r.is_pending() {
                if let Some(waker_setter) = self.waker_setter {
                    waker_setter(cx.waker().clone());
                    g(self);
                } else {
                    // always ready to scan
                    cx.waker().wake_by_ref();
                }
            }
            r
        })
        .await
    }

    fn has_errored(&self) -> bool {
        self.pio.get_irq_raw() & (1 << SMI::id()) != 0
    }

    async fn resume_after_error(&mut self) {
        self.tx.drain_fifo();
        self.pio.clear_irq(1 << SMI::id());

        self.block_on(
            |me| {
                if me.rx.read().is_some() {
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            },
            |me| {
                me.rx.enable_rx_not_empty_interrupt(PioIRQ::Irq0);
            },
        )
        .await;
    }

    async fn put(&mut self, data: u16) {
        self.block_on(
            |me| {
                if me.tx.write_u16_replicated(data) {
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            },
            |me| {
                me.tx.enable_tx_not_full_interrupt(PioIRQ::Irq0);
            },
        )
        .await;
    }

    async fn put_data(&mut self, data: u8, read_ack: bool, last: bool) {
        let final_field = if last { FINAL_BIT } else { 0 };
        let nak_field = if read_ack { NAK_BIT } else { 0 };
        let data_field = u16::from(data) << DATA_OFFSET;

        // instr (6bits) = 0 | final (1bit) | data (8bits) | read ack (1bit)
        let word = final_field | data_field | nak_field;
        self.put(word).await
    }

    async fn put_instr_sequence<T, U>(&mut self, seq: T)
    where
        T: IntoIterator<IntoIter = U>,
        U: Iterator<Item = Instruction> + ExactSizeIterator,
    {
        let seq = seq.into_iter();
        assert!(seq.len() < 64);
        let n = seq.len() as u16;

        self.put((n - 1) << INSTR_OFFSET).await;
        for instr in seq {
            self.put(instr.encode(SIDESET)).await;
        }
    }

    async fn start(&mut self) {
        self.put_instr_sequence([SC1SD0, SC0SD0]).await
    }

    async fn stop(&mut self) {
        if self.has_errored() {
            self.resume_after_error().await;
        }
        self.put_instr_sequence([SC0SD0, SC1SD0, SC1SD1]).await
    }

    async fn restart(&mut self) {
        self.put_instr_sequence([SC0SD1, SC1SD1, SC1SD0, SC0SD0])
            .await
    }

    async fn setup<A>(&mut self, address: A, read: bool, do_restart: bool) -> Result<(), ErrorKind>
    where
        A: Into<u16> + 'static,
    {
        // TODO: validate addr
        let address: u16 = address.into();

        // At this stage, the all fifos should be empty and in counter
        assert!(self.rx.read().is_none(), "rx FIFO shall be empty");

        // send start condition
        if !do_restart {
            self.start().await;
        } else {
            self.restart().await;
        }

        let read_flag = if read { 1 } else { 0 };

        // send address
        use core::any::TypeId;
        let a_tid = TypeId::of::<A>();
        let mut address_len: u32 = if TypeId::of::<SevenBitAddress>() == a_tid {
            let addr = (address << 1) | read_flag;
            self.put_data(addr as u8, true, false).await;
            1
        } else if TypeId::of::<TenBitAddress>() == a_tid {
            let addr_hi = 0xF0 | ((address >> 7) & 0x6) | read_flag;
            let addr_lo = address & 0xFF;
            self.put_data(addr_hi as u8, true, false).await;
            self.put_data(addr_lo as u8, true, false).await;
            2
        } else {
            panic!("Unsupported address type.");
        };

        self.block_on(
            |me| {
                while address_len > 0 && me.rx.read().is_some() {
                    address_len -= 1;
                }

                if me.has_errored() || address_len == 0 {
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            },
            |me| {
                me.rx.enable_rx_not_empty_interrupt(PioIRQ::Irq0);
                me.pio.irq0().enable_sm_interrupt(SMI::id() as u8);
            },
        )
        .await;

        if self.has_errored() {
            Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address))
        } else {
            Ok(())
        }
    }

    async fn read(&mut self, buffer: &mut [u8]) -> Result<(), ErrorKind> {
        assert!(
            !self.has_errored() && self.rx.is_empty(),
            "Invalid state in entering read: has_errored:{} rx.is_empty:{}",
            self.has_errored(),
            self.rx.is_empty()
        );

        let mut queued = 0;
        let mut iter = buffer.iter_mut();

        // while there are still bytes to queue
        while iter.len() != 0 && !self.has_errored() {
            if queued < iter.len() && !self.tx.is_full() {
                queued += 1;
                let last = queued == iter.len();
                self.put_data(0xFF, last, last).await;
            }

            if let Some(byte) = self.rx.read() {
                queued -= 1;
                if let Some(data) = iter.next() {
                    *data = (byte & 0xFF) as u8;
                }
            } else {
                self.block_on(
                    |me| {
                        if me.has_errored()
                            || (iter.len() > 0 && !me.tx.is_full())
                            || !me.rx.is_empty()
                        {
                            Poll::Ready(())
                        } else {
                            Poll::Pending
                        }
                    },
                    |me| {
                        if iter.len() > 0 {
                            me.tx.enable_tx_not_full_interrupt(PioIRQ::Irq0);
                        }
                        me.pio.irq0().enable_sm_interrupt(SMI::id() as u8);
                        me.rx.enable_rx_not_empty_interrupt(PioIRQ::Irq0);
                    },
                )
                .await;
            }
        }

        if self.has_errored() {
            Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data))
        } else {
            Ok(())
        }
    }

    async fn write<B>(&mut self, buffer: B) -> Result<(), ErrorKind>
    where
        B: IntoIterator<Item = u8>,
    {
        assert!(
            !self.has_errored() && self.rx.is_empty(),
            "Invalid state in entering write: has_errored:{} rx.is_empty:{}",
            self.has_errored(),
            self.rx.is_empty()
        );

        let mut queued = 0;
        let mut iter = buffer.into_iter().peekable();
        while let (Some(byte), false) = (iter.next(), self.has_errored()) {
            // ignore any received bytes
            if self.rx.read().is_some() {
                queued -= 1;
            }
            self.put_data(byte, true, iter.peek().is_none()).await;
            queued += 1;
        }

        self.block_on(
            |me| {
                if me.rx.read().is_some() {
                    queued -= 1;
                }
                if queued == 0 || me.has_errored() {
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            },
            |me| {
                me.pio.irq0().enable_sm_interrupt(SMI::id() as u8);
                me.rx.enable_rx_not_empty_interrupt(PioIRQ::Irq0);
            },
        )
        .await;

        if self.has_errored() {
            Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data))
        } else {
            Ok(())
        }
    }

    /// Writes to the i2c bus consuming bytes for the given iterator.
    pub async fn write_iter<A, U>(&mut self, address: A, bytes: U) -> Result<(), ErrorKind>
    where
        U: IntoIterator<Item = u8>,
        A: AddressMode + 'static + Into<u16>,
    {
        let mut res = self.setup(address, false, false).await;
        if res.is_ok() {
            res = self.write(bytes).await;
        }
        self.stop().await;
        res
    }
}

impl<'pio, P, SMI, SDA, SCL> embedded_hal_async::i2c::ErrorType for I2C<'pio, P, SMI, SDA, SCL>
where
    P: PIOExt + FunctionConfig,
    SMI: StateMachineIndex,
    SDA: PinId,
    SCL: PinId,
    Function<P>: ValidPinMode<SDA> + ValidPinMode<SCL>,
{
    type Error = ErrorKind;
}
impl<'pio, P, SMI, SDA, SCL, A> embedded_hal_async::i2c::I2c<A> for I2C<'pio, P, SMI, SDA, SCL>
where
    P: PIOExt + FunctionConfig + 'pio,
    SMI: StateMachineIndex,
    SDA: PinId,
    SCL: PinId,
    Function<P>: ValidPinMode<SDA> + ValidPinMode<SCL>,
    A: AddressMode + Into<u16> + Copy + 'static,
{
    type WriteFuture<'a>
    = impl Future<Output = Result<(), Self::Error>> + 'a where
        Self: 'a ;

    type ReadFuture<'a> = impl Future<Output = Result<(), Self::Error>> + 'a
    where
        Self: 'a;

    type WriteReadFuture<'a> = impl Future<Output = Result<(), Self::Error>> + 'a
    where
        Self: 'a;

    type TransactionFuture<'a, 'b> = impl Future<Output = Result<(), Self::Error>> + 'a
        where Self: 'a, 'b: 'a;

    fn read<'a>(&'a mut self, address: A, buffer: &'a mut [u8]) -> Self::ReadFuture<'a> {
        async move {
            let mut res = self.setup(address, true, false).await;
            if res.is_ok() {
                res = self.read(buffer).await;
            }
            self.stop().await;
            res
        }
    }

    fn write<'a>(&'a mut self, address: A, bytes: &'a [u8]) -> Self::WriteFuture<'a> {
        self.write_iter(address, bytes.into_iter().cloned())
    }

    fn write_read<'a>(
        &'a mut self,
        address: A,
        bytes: &'a [u8],
        buffer: &'a mut [u8],
    ) -> Self::WriteReadFuture<'a> {
        async move {
            let mut res = self.setup(address, false, false).await;
            if res.is_ok() {
                res = self.write(bytes.into_iter().cloned()).await;
            }
            if res.is_ok() {
                res = self.setup(address, true, true).await;
            }
            if res.is_ok() {
                res = self.read(buffer).await;
            }
            self.stop().await;
            res
        }
    }

    fn transaction<'a, 'b>(
        &'a mut self,
        address: A,
        operations: &'a mut [Operation<'b>],
    ) -> Self::TransactionFuture<'a, 'b> {
        async move {
            let mut first = true;
            let mut res = Ok(());
            for op in operations {
                match op {
                    Operation::Read(buf) => {
                        res = self.setup(address, true, !first).await;
                        if res.is_ok() {
                            res = self.read(buf).await;
                        }
                    }
                    Operation::Write(buffer) => {
                        res = self.setup(address, false, !first).await;
                        if res.is_ok() {
                            res = self.write(buffer.into_iter().cloned()).await;
                        }
                    }
                }
                if res.is_err() {
                    break;
                }
                first = false;
            }
            self.stop().await;
            res
        }
    }
}

use core::{future, marker::PhantomData, task::Poll};

use embedded_hal::i2c::{
    AddressMode, ErrorKind, NoAcknowledgeSource, Operation, SevenBitAddress, TenBitAddress,
};
use fugit::HertzU32;
use pio::{Instruction, InstructionOperands, SideSet};
use rp2040_hal::{
    gpio::{AnyPin, DynPinId, FunctionNull, Pin, PullUp, ValidFunction},
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
    P: PIOExt,
    SMI: StateMachineIndex,
    SDA: AnyPin,
    SCL: AnyPin,
{
    pio: &'pio mut PIO<P>,
    sm: StateMachine<(P, SMI), rp2040_hal::pio::Running>,
    tx: Tx<(P, SMI)>,
    rx: Rx<(P, SMI)>,
    _sda: Pin<SDA::Id, P::PinFunction, PullUp>,
    _scl: Pin<SCL::Id, P::PinFunction, PullUp>,
    waker_setter: Option<fn(core::task::Waker)>,
    pins: PhantomData<(SDA, SCL)>,
}

impl<'pio, P, SMI, SDA, SCL> I2C<'pio, P, SMI, SDA, SCL>
where
    P: PIOExt,
    SMI: StateMachineIndex,
    SDA: AnyPin,
    SCL: AnyPin,
{
    /// Creates a new instance of this driver.
    ///
    /// Note: the PIO must have been reset before using this driver.
    pub fn new(
        pio: &'pio mut PIO<P>,
        sda: SDA,
        scl: SCL,
        sm: UninitStateMachine<(P, SMI)>,
        bus_freq: HertzU32,
        clock_freq: HertzU32,
    ) -> Self
    where
        SDA: AnyPin<Function = FunctionNull>,
        SCL: AnyPin<Function = FunctionNull>,
        SDA::Id: ValidFunction<P::PinFunction>,
        SCL::Id: ValidFunction<P::PinFunction>,
    {
        let mut program = pio_proc::pio_asm!(
            ".side_set 1 opt pindirs"

            "byte_nack:"
            "  jmp  y--     byte_end"
            "  irq  wait    0    rel"
            "  jmp          byte_end"

            "byte_send:"
            "  out  y       1"
            "  set  x       7"

            "bitloop:"
            "  out  pindirs 1                [7]"
            "  nop                    side 1 [2]"
            //      polarity
            "  wait 1       gpio 0           [4]"
            "  in   pins 1                   [7]"
            "  jmp  x--     bitloop   side 0 [7]"

            "  out  pindirs 1                [7]"
            "  nop                    side 1 [7]"
            //      polarity
            "  wait 1       gpio 0           [7]"
            "  jmp  pin     byte_nack side 0 [2]"

            "byte_end:"
            "  push block"

            ".wrap_target"
            "  out  x       6"
            "  jmp  !x      byte_send"
            "  out  null    10"

            "do_exec:"
            "  out  exec    16"
            "  jmp  x--     do_exec"
            ".wrap"
        )
        .program;

        let scl_pin = scl.into();
        let sda_pin = sda.into();
        let scl_pin_id: DynPinId = scl_pin.id();
        let sda_pin_id: DynPinId = sda_pin.id();

        // patch the program to decouple sda and scl
        program.code[7] |= u16::from(scl_pin_id.num);
        program.code[12] |= u16::from(scl_pin_id.num);

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
            .set_pins(sda_pin_id.num, 1)
            .out_pins(sda_pin_id.num, 1)
            .in_pin_base(sda_pin_id.num)
            .side_set_pin_base(scl_pin_id.num)
            .jmp_pin(sda_pin_id.num)
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
        let sda = sda_pin.into_pull_type::<PullUp>();
        let scl = scl_pin.into_pull_type::<PullUp>();

        // This will pull the bus high for a little bit of time
        sm.set_pins([
            (scl_pin_id.num, PinState::High),
            (sda_pin_id.num, PinState::High),
        ]);
        sm.set_pindirs([
            (scl_pin_id.num, PinDir::Output),
            (sda_pin_id.num, PinDir::Output),
        ]);

        // attach SDA pin to pio
        let mut sda = sda.into_function::<P::PinFunction>();
        // configure SDA pin as inverted
        sda.set_output_enable_override(rp2040_hal::gpio::OutputEnableOverride::Invert);

        // attach SCL pin to pio
        let mut scl = scl.into_function::<P::PinFunction>();
        // configure SCL pin as inverted
        scl.set_output_enable_override(rp2040_hal::gpio::OutputEnableOverride::Invert);

        // the PIO now keeps the pin as Input, we can set the pin state to Low.
        sm.set_pins([
            (sda_pin_id.num, PinState::Low),
            (scl_pin_id.num, PinState::Low),
        ]);

        // Set the state machine on the entry point.
        sm.exec_instruction(Instruction {
            operands: InstructionOperands::JMP {
                condition: pio::JmpCondition::Always,
                address: wrap_target,
            },
            delay: 0,
            side_set: None,
        });

        // enable
        let sm = sm.start();

        Self {
            pio,
            sm,
            tx,
            rx,
            _sda: sda,
            _scl: scl,
            waker_setter: None,
            pins: PhantomData,
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

    fn has_errored(&self) -> bool {
        self.pio.get_irq_raw() & (1 << SMI::id()) != 0
    }

    fn rx_purge(&mut self) -> Poll<()> {
        if self.rx.read().is_some() {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
    fn write_data(&mut self, data: u16) -> Poll<()> {
        if self.tx.write_u16_replicated(data) {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
    fn address_sent(&mut self, mut address_len: usize) -> Poll<()> {
        while address_len > 0 && self.rx.read().is_some() {
            address_len -= 1;
        }

        if self.has_errored() || address_len == 0 {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
    fn tx_not_full(&mut self, iter_len: usize) -> Poll<()> {
        if self.has_errored() || (iter_len > 0 && !self.tx.is_full()) || !self.rx.is_empty() {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
    fn data_sent(&mut self, mut queued: usize) -> Poll<()> {
        if self.rx.read().is_some() {
            queued -= 1;
        }
        if queued == 0 || self.has_errored() {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
    fn enable_rx_not_empty(&self) {
        self.rx.enable_rx_not_empty_interrupt(PioIRQ::Irq0);
        self.pio.irq0().enable_sm_interrupt(SMI::id() as u8);
    }
    fn enable_tx_not_full(&self) {
        self.tx.enable_tx_not_full_interrupt(PioIRQ::Irq0);
        self.pio.irq0().enable_sm_interrupt(SMI::id() as u8);
    }
    fn enable_rx_not_empty_and_tx_not_full_data_to_send(&self, iter_len: usize) {
        if iter_len > 0 {
            self.tx.enable_tx_not_full_interrupt(PioIRQ::Irq0);
        }
        self.pio.irq0().enable_sm_interrupt(SMI::id() as u8);
        self.rx.enable_rx_not_empty_interrupt(PioIRQ::Irq0);
    }

    async fn resume_after_error(&mut self) {
        self.sm.drain_tx_fifo();
        self.pio.clear_irq(1 << SMI::id());

        block_on!(self, rx_purge(), enable_rx_not_empty()).await;
    }

    async fn put(&mut self, data: u16) {
        block_on!(self, write_data(data), enable_tx_not_full()).await;
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
        let address_len: usize = if TypeId::of::<SevenBitAddress>() == a_tid {
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

        block_on!(self, address_sent(address_len), enable_rx_not_empty()).await;

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
                block_on!(
                    self,
                    tx_not_full(iter.len()),
                    enable_rx_not_empty_and_tx_not_full_data_to_send(iter.len())
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

        block_on!(self, data_sent(queued), enable_rx_not_empty()).await;

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
    P: PIOExt,
    SMI: StateMachineIndex,
    SDA: AnyPin,
    SCL: AnyPin,
{
    type Error = ErrorKind;
}
impl<'pio, P, SMI, SDA, SCL, A> embedded_hal_async::i2c::I2c<A> for I2C<'pio, P, SMI, SDA, SCL>
where
    P: PIOExt,
    SMI: StateMachineIndex,
    SDA: AnyPin,
    SCL: AnyPin,
    A: AddressMode + Into<u16> + Copy + 'static,
{
    async fn read(&mut self, address: A, buffer: &mut [u8]) -> Result<(), ErrorKind> {
        let mut res = self.setup(address, true, false).await;
        if res.is_ok() {
            res = self.read(buffer).await;
        }
        self.stop().await;
        res
    }

    async fn write(&mut self, address: A, bytes: &[u8]) -> Result<(), ErrorKind> {
        self.write_iter(address, bytes.iter().cloned()).await
    }
    async fn write_read(
        &mut self,
        address: A,
        bytes: &[u8],
        buffer: &mut [u8],
    ) -> Result<(), ErrorKind> {
        let mut res = self.setup(address, false, false).await;
        if res.is_ok() {
            res = self.write(bytes.iter().cloned()).await;
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
    async fn transaction(
        &mut self,
        address: A,
        operations: &mut [Operation<'_>],
    ) -> Result<(), ErrorKind> {
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
                        res = self.write(buffer.iter().cloned()).await;
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
impl<'pio, P, SMI, SDA, SCL, A> i2c_write_iter::non_blocking::WriteIter<A>
    for I2C<'pio, P, SMI, SDA, SCL>
where
    P: PIOExt,
    SMI: StateMachineIndex,
    SDA: AnyPin,
    SCL: AnyPin,
    A: AddressMode + Into<u16> + Copy + 'static,
{
    async fn write_iter<'a, U>(&'a mut self, address: A, bytes: U) -> Result<(), Self::Error>
    where
        U: IntoIterator<Item = u8> + 'a,
    {
        self.write_iter(address, bytes).await
    }
}
impl<'pio, P, SMI, SDA, SCL, A> i2c_write_iter::non_blocking::WriteIterRead<A>
    for I2C<'pio, P, SMI, SDA, SCL>
where
    P: PIOExt,
    SMI: StateMachineIndex,
    SDA: AnyPin,
    SCL: AnyPin,
    A: AddressMode + Into<u16> + Copy + 'static,
{
    async fn write_iter_read<'a, U>(
        &'a mut self,
        address: A,
        bytes: U,
        read: &mut [u8],
    ) -> Result<(), Self::Error>
    where
        U: IntoIterator<Item = u8> + 'a,
    {
        let mut res = self.setup(address, false, false).await;
        if res.is_ok() {
            res = self.write(bytes).await;
        }
        if res.is_ok() {
            res = self.setup(address, true, true).await;
        }
        if res.is_ok() {
            res = self.read(read).await;
        }
        self.stop().await;
        res
    }
}

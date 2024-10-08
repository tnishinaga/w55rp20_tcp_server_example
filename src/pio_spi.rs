use core::slice;
use embassy_rp::{
    pio::{Common, Config, PioPin, ShiftDirection, StateMachine},
    PeripheralRef,
};
use embedded_hal_async::spi::{Error, ErrorType, SpiBus};
use fixed::traits::ToFixed;
use fixed_macro::types::U56F8;
use zerocopy::AsBytes;
use zerocopy_derive::{AsBytes, FromBytes, FromZeroes};

const BUFFER_SIZE_IN_BYTES: usize = 1024;
const BUFFER_SIZE_IN_WORDS: usize = BUFFER_SIZE_IN_BYTES / 4;

pub(crate) mod pio_program {
    use pio::Program;

    pub(crate) fn pio_spi_mode0() -> Program<{ pio::RP2040_MAX_PROGRAM_SIZE }> {
        let mut a = pio::Assembler::<{ pio::RP2040_MAX_PROGRAM_SIZE }>::new_with_side_set(
            pio::SideSet::new(true, 1, false),
        );
        let mut wrap_target = a.label();
        let mut wrap_source = a.label();
        let mut transfer_start = a.label();
        let mut fill_isr = a.label();

        // out: MOSI
        // in: MISO
        // side: SCK
        // set: CS

        // MODE:0
        //  Clock active low
        //  data captured rising-edge

        a.bind(&mut wrap_target);
        {
            // set MOSI and SCK to Low
            a.mov(
                pio::MovDestination::OSR,
                pio::MovOperation::None,
                pio::MovSource::NULL,
            );
            a.out_with_side_set(pio::OutDestination::PINS, 1, 0);

            // get data
            // data structure:
            //      bit length n(must be multiple of 8) - 1 (32bit)
            //      bits... (32bit x m)
            a.pull(false, true);
            a.out(pio::OutDestination::X, 32);

            {
                // spi transfer loop
                a.bind(&mut transfer_start);
                a.pull(true, true);
                // set MOSI pin before clock
                a.out_with_delay_and_side_set(pio::OutDestination::PINS, 1, 4, 0);
                a.in_with_delay_and_side_set(pio::InSource::PINS, 1, 4, 1);
                // rise-clock(2-cycle)
                a.push(true, true);
                // fall-clock(2-cycle)
                a.jmp(pio::JmpCondition::XDecNonZero, &mut transfer_start);
            }

            {
                a.jmp(
                    pio::JmpCondition::OutputShiftRegisterNotEmpty,
                    &mut fill_isr,
                );
                a.jmp(pio::JmpCondition::Always, &mut wrap_target);
                // ISR not filled
                {
                    a.bind(&mut fill_isr);
                    a.push(false, true);
                }
            }
        }
        a.bind(&mut wrap_source);

        a.assemble_with_wrap(wrap_source, wrap_target)
    }
}

#[derive(Debug, AsBytes, FromZeroes, FromBytes)]
#[repr(C, align(4))]
struct PioSpiDmaBuffer([u32; BUFFER_SIZE_IN_WORDS]);

impl PioSpiDmaBuffer {
    pub fn new() -> Self {
        Self([0u32; BUFFER_SIZE_IN_WORDS])
    }
}

impl AsRef<[u32; BUFFER_SIZE_IN_WORDS]> for PioSpiDmaBuffer {
    fn as_ref(&self) -> &[u32; BUFFER_SIZE_IN_WORDS] {
        &self.0
    }
}
impl AsMut<[u32; BUFFER_SIZE_IN_WORDS]> for PioSpiDmaBuffer {
    fn as_mut(&mut self) -> &mut [u32; BUFFER_SIZE_IN_WORDS] {
        &mut self.0
    }
}

pub struct PioSpi<
    'a,
    PIO: embassy_rp::pio::Instance,
    const SM: usize,
    DmaTx: embassy_rp::dma::Channel,
    DmaRx: embassy_rp::dma::Channel,
> {
    // pio: &'a mut Common<'a, PIO>,
    sm: StateMachine<'a, PIO, SM>,
    dma_tx_ref: PeripheralRef<'a, DmaTx>,
    dma_rx_ref: PeripheralRef<'a, DmaRx>,
}

impl<
        'a,
        PIO: embassy_rp::pio::Instance,
        const SM: usize,
        DmaTx: embassy_rp::dma::Channel,
        DmaRx: embassy_rp::dma::Channel,
    > PioSpi<'a, PIO, SM, DmaTx, DmaRx>
{
    pub fn new(
        pio: &mut Common<'a, PIO>,
        mut sm: StateMachine<'a, PIO, SM>,
        dma_tx_ref: PeripheralRef<'a, DmaTx>,
        dma_rx_ref: PeripheralRef<'a, DmaRx>,
        clk: impl PioPin,
        miso: impl PioPin,
        mosi: impl PioPin,
    ) -> Self {
        let (clk_pin, miso_pin, mosi_pin) = (
            pio.make_pio_pin(clk),
            pio.make_pio_pin(miso),
            pio.make_pio_pin(mosi),
        );

        // set pindirs
        sm.set_pin_dirs(embassy_rp::pio::Direction::Out, &[&clk_pin, &mosi_pin]);
        sm.set_pin_dirs(embassy_rp::pio::Direction::In, &[&miso_pin]);

        let mut cfg = Config::default();

        // TODO: support MODE1-3
        cfg.use_program(
            &pio.load_program(&crate::pio_spi::pio_program::pio_spi_mode0()),
            &[&clk_pin],
        );
        cfg.set_in_pins(&[&miso_pin]);
        cfg.set_out_pins(&[&mosi_pin]);
        cfg.set_set_pins(&[&mosi_pin]);
        // using slowest clock to debug
        // defmt::info!("core clock: {}", embassy_rp::clocks::clk_sys_freq());
        // target_freq = core_freq / x;
        // x =  core_freq / target_freq
        // x = 125(MHz) / 50(MHz) = 2.5;
        // TODO: change
        cfg.clock_divider = U56F8!(2).to_fixed();
        // cfg.clock_divider = U56F8!(65535).to_fixed();
        cfg.shift_out.direction = ShiftDirection::Left;
        cfg.shift_in.direction = ShiftDirection::Left;
        sm.set_config(&cfg);

        sm.clear_fifos();
        sm.set_enable(true);

        Self {
            sm,
            dma_tx_ref,
            dma_rx_ref,
        }
    }

    pub async fn transaction_inner(&mut self, dst: &mut [u8], src: &[u8]) -> Option<()> {
        // if BUFFER_SIZE * 4 < dst.len() || BUFFER_SIZE * 4 < src.len() {
        //     defmt::debug!("dst.len() = {}, src.len() = {}", dst.len(), src.len());
        //     panic!("unsupported");
        //     return None;
        // }

        let mut dst = dst;
        let mut src = src;

        while dst.len() != 0 || src.len() != 0 {
            let length = core::cmp::max(src.len(), dst.len());
            let length = core::cmp::min(BUFFER_SIZE_IN_BYTES, length);
            let src_length = core::cmp::min(src.len(), BUFFER_SIZE_IN_BYTES);
            let dst_length = core::cmp::min(dst.len(), BUFFER_SIZE_IN_BYTES);
            let length_in_words = length.next_multiple_of(4) / 4;
            // create pio command
            let transfer_bits = length * 8 - 1;

            let (mut tx_buffer, mut rx_buffer) = (PioSpiDmaBuffer::new(), PioSpiDmaBuffer::new());

            // copy src data to tx_buffer
            tx_buffer.as_bytes_mut()[..src_length].copy_from_slice(&src[..src_length]);

            // change byte order to support MSB
            for t in &mut tx_buffer.as_mut()[..length_in_words] {
                *t = t.swap_bytes();
            }

            let (rx, tx) = self.sm.rx_tx();
            tx.push(transfer_bits.try_into().unwrap());

            embassy_futures::join::join(
                tx.dma_push(
                    self.dma_tx_ref.reborrow(),
                    &tx_buffer.as_ref()[..length_in_words],
                ),
                rx.dma_pull(
                    self.dma_rx_ref.reborrow(),
                    &mut rx_buffer.as_mut()[..length_in_words],
                ),
            )
            .await;

            // change byte order to support MSB
            for t in &mut rx_buffer.as_mut()[..length_in_words] {
                *t = t.swap_bytes();
            }
            // dst.len() != 32の場合は中途半端な値が来るので、調整する
            let shift = (32 - dst.len() % 4 * 8) % 32;
            rx_buffer.as_mut()[length_in_words - 1] =
                rx_buffer.as_mut()[length_in_words - 1] >> shift;
            dst[..dst_length].copy_from_slice(&rx_buffer.as_bytes()[..dst_length]);

            src = &src[src_length..];
            dst = &mut dst[dst_length..];
        }

        Some(())
    }
}

#[derive(Debug)]
pub struct PioSpiError {
    kind: embedded_hal_1::spi::ErrorKind,
}

impl PioSpiError {
    pub fn new(kind: embedded_hal_1::spi::ErrorKind) -> Self {
        Self { kind }
    }
}

impl Error for PioSpiError {
    fn kind(&self) -> embedded_hal_1::spi::ErrorKind {
        self.kind
    }
}

impl<
        'a,
        PIO: embassy_rp::pio::Instance,
        const SM: usize,
        DmaTx: embassy_rp::dma::Channel,
        DmaRx: embassy_rp::dma::Channel,
    > ErrorType for PioSpi<'a, PIO, SM, DmaTx, DmaRx>
{
    type Error = PioSpiError;
}

impl<
        'a,
        PIO: embassy_rp::pio::Instance,
        const SM: usize,
        DmaTx: embassy_rp::dma::Channel,
        DmaRx: embassy_rp::dma::Channel,
    > SpiBus for PioSpi<'a, PIO, SM, DmaTx, DmaRx>
{
    async fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        defmt::trace!("SpiBus read {} bytes", words.len());

        self.transaction_inner(words, &[])
            .await
            .ok_or(PioSpiError::new(embedded_hal_1::spi::ErrorKind::Other))
    }

    async fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        defmt::trace!("SpiBus write {=[u8]:x}", words);
        self.transaction_inner(&mut [], words)
            .await
            .ok_or(PioSpiError::new(embedded_hal_1::spi::ErrorKind::Other))
    }

    async fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        defmt::trace!("SpiBus transfer read {} bytes", read.len());
        defmt::trace!("SpiBus transfer write {=[u8]:x}", write);
        self.transaction_inner(read, write)
            .await
            .ok_or(PioSpiError::new(embedded_hal_1::spi::ErrorKind::Other))
    }

    async fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        defmt::trace!("SpiBus transfer_in_place {=[u8]:x}", words);
        let len = words.len();
        let ptr = words.as_mut_ptr();
        let read = unsafe { slice::from_raw_parts_mut(ptr, len) };
        let write = unsafe { slice::from_raw_parts(ptr, len) };
        self.transaction_inner(read, write)
            .await
            .ok_or(PioSpiError::new(embedded_hal_1::spi::ErrorKind::Other))
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

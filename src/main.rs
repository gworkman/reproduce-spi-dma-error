//! This example shows a complete project, including file structure, and config
//! needed to flash using an ST-Link. The project structure is based on
//! [Knurling's app-template](https://github.com/knurling-rs/app-template).
//! This file demonstrates an overview of this library's features.

//! See the syntax example in the main STM32-HAL repo for a more detailed example.

#![no_main]
#![no_std]

use cortex_m::{
    self,
    delay::{self, Delay},
    interrupt::{free, Mutex},
    peripheral::NVIC,
};
use cortex_m_rt::entry;

// import RefCell
use core::cell::RefCell;

// These lines are part of our setup for debug printing.
use defmt_rtt as _;
use panic_probe as _;

use defmt::println;

// Import parts of this library we use. You could use this style, or perhaps import
// less here.
use stm32_hal2::{
    self, access_global,
    clocks::Clocks,
    dma::{self, ChannelCfg, Dma, DmaChannel, DmaInterrupt},
    gpio::{OutputSpeed, OutputType, Pin, PinMode, Port},
    low_power, make_globals, pac,
    pac::interrupt,
    spi::{Spi, SpiConfig},
};

static mut read_buf: [u8; 4] = [0; 4];
static mut write_buf: [u8; 5] = [0; 5];

make_globals!((SPI, Spi<pac::SPI2>));

#[entry]
fn main() -> ! {
    // Set up ARM Cortex-M peripherals. These are common to many MCUs, including all STM32 ones.
    let cp = cortex_m::Peripherals::take().unwrap();
    // Set up peripherals specific to the microcontroller you're using.
    let dp = pac::Peripherals::take().unwrap();

    // This line is required to prevent the debugger from disconnecting on entering WFI.
    // This appears to be a limitation of many STM32 families. Not required in production code,
    // and significantly increases power consumption in low-power modes.
    stm32_hal2::debug_workaround();

    // Create an initial clock configuration that uses the MCU's internal oscillator (HSI),
    // sets the MCU to its maximum system clock speed.
    let clock_cfg = Clocks::default();

    // Write the clock configuration to the MCU. If you wish, you can modify `clocks` above
    // in accordance with [its docs](https://docs.rs/stm32-hal2/0.2.0/stm32_hal2/clocks/index.html),
    // and the `clock_cfg` example.
    clock_cfg.setup().unwrap();

    println!("Start test");

    let mut ssi_abn = Pin::new(Port::A, 5, PinMode::Output);
    ssi_abn.set_high();

    // setup SPI
    let mut spi2_miso = Pin::new(Port::B, 14, PinMode::Alt(5));
    let mut spi2_mosi = Pin::new(Port::B, 15, PinMode::Alt(5));
    let mut spi2_sck = Pin::new(Port::B, 13, PinMode::Alt(5));
    let mut spi2_cs = Pin::new(Port::B, 12, PinMode::Output);

    spi2_miso.output_speed(OutputSpeed::VeryHigh);
    spi2_mosi.output_speed(OutputSpeed::VeryHigh);
    spi2_sck.output_speed(OutputSpeed::VeryHigh);
    spi2_cs.output_speed(OutputSpeed::VeryHigh);

    spi2_miso.output_type(OutputType::PushPull);
    spi2_mosi.output_type(OutputType::PushPull);
    spi2_sck.output_type(OutputType::PushPull);
    spi2_cs.output_type(OutputType::PushPull);

    spi2_cs.set_high();

    let spi_cfg = SpiConfig {
        mode: stm32_hal2::spi::SpiMode::mode3(),
        ..Default::default()
    };
    let mut spi = Spi::new(dp.SPI2, spi_cfg, stm32_hal2::spi::BaudRate::Div256);

    unsafe {
        write_buf = [0x81, 0, 0, 0, 0];
        // read some data synchronously
        spi2_cs.set_low();
        spi.write(&write_buf).unwrap();
        spi2_cs.set_high();

        // read some data synchronously
        spi2_cs.set_low();
        spi.write(&[0x00]).unwrap();
        spi.transfer(&mut read_buf).unwrap();
        spi2_cs.set_high();
    }

    unsafe {
        println!("Buf: {:x}", read_buf);
        assert_eq!(read_buf, [0x34, 0x36, 0x37, 0x31]); // this is known device ID
    }

    // now try to do the same thing but asynchronously
    let mut dma = Dma::new(dp.DMA1);

    // SPI2_RX is on DMA1_CH4
    dma.enable_interrupt(DmaChannel::C4, DmaInterrupt::TransferComplete);
    // SPI2_TX is on DMA1_CH5
    dma.enable_interrupt(DmaChannel::C5, DmaInterrupt::TransferComplete);

    unsafe {
        NVIC::unmask(pac::Interrupt::DMA1_CH4);
        NVIC::unmask(pac::Interrupt::DMA1_CH5);
    }

    free(|cs| {
        SPI.borrow(cs).replace(Some(spi));
    });

    let mut delay = Delay::new(cp.SYST, clock_cfg.sysclk());

    free(|cs| {
        access_global!(SPI, spi, cs);
        println!("BEFORE");
        println!("SPI2.cr1 {:b}", spi.regs.cr1.read().bits());
        println!("SPI2.cr2 {:b}", spi.regs.cr2.read().bits());
        println!("SPI2.crcpr {:b}", spi.regs.crcpr.read().bits());
        println!("SPI2.dr {:b}", spi.regs.dr.read().bits());
        println!("SPI2.rxcrcr {:b}", spi.regs.rxcrcr.read().bits());
        println!("SPI2.sr {:b}", spi.regs.sr.read().bits());
        println!("SPI2.txcrcr {:b}", spi.regs.txcrcr.read().bits());
    });

    spi2_cs.set_low();
    unsafe {
        free(|cs| {
            access_global!(SPI, spi, cs);
            spi.write_dma(&write_buf, DmaChannel::C5, ChannelCfg::default(), &mut dma);
        });
    }

    // transfer should have completed by now. The interrupt handler
    // should have set the CS pin high and print to console
    delay.delay_ms(1000);

    spi2_cs.set_high();
    delay.delay_ms(100);

    free(|cs| {
        access_global!(SPI, spi, cs);
        println!("AFTER");
        println!("SPI2.cr1 {:b}", spi.regs.cr1.read().bits());
        println!("SPI2.cr2 {:b}", spi.regs.cr2.read().bits());
        println!("SPI2.crcpr {:b}", spi.regs.crcpr.read().bits());
        println!("SPI2.dr {:b}", spi.regs.dr.read().bits());
        println!("SPI2.rxcrcr {:b}", spi.regs.rxcrcr.read().bits());
        println!("SPI2.sr {:b}", spi.regs.sr.read().bits());
        println!("SPI2.txcrcr {:b}", spi.regs.txcrcr.read().bits());
    });

    spi2_cs.set_low();
    unsafe {
        free(|cs| {
            access_global!(SPI, spi, cs);
            spi.read_dma(
                &mut read_buf,
                DmaChannel::C5,
                ChannelCfg::default(),
                dma::DmaPeriph::Dma1,
            );
        });
    }
    // transfer should have completed by now. The interrupt handler
    // should have set the CS pin high and print to console
    delay.delay_ms(1000);

    free(|cs| {
        access_global!(SPI, spi, cs);
        println!("AFTER READ");
        println!("SPI2.cr1 {:b}", spi.regs.cr1.read().bits());
        println!("SPI2.cr2 {:b}", spi.regs.cr2.read().bits());
        println!("SPI2.crcpr {:b}", spi.regs.crcpr.read().bits());
        println!("SPI2.dr {:b}", spi.regs.dr.read().bits());
        println!("SPI2.rxcrcr {:b}", spi.regs.rxcrcr.read().bits());
        println!("SPI2.sr {:b}", spi.regs.sr.read().bits());
        println!("SPI2.txcrcr {:b}", spi.regs.txcrcr.read().bits());
    });

    println!("Done");

    loop {
        low_power::sleep_now();
    }
}

#[interrupt]
fn DMA1_CH4() {
    dma::clear_interrupt(
        dma::DmaPeriph::Dma1,
        DmaChannel::C4,
        DmaInterrupt::TransferComplete,
    );

    free(|cs| {
        access_global!(SPI, spi, cs);
        spi.stop_dma(DmaChannel::C4, None, dma::DmaPeriph::Dma1);
    });

    let mut spi2_cs = Pin::new(Port::B, 12, PinMode::Output);
    spi2_cs.set_high();

    println!("CH4 interrupt called")
}

#[interrupt]
fn DMA1_CH5() {
    dma::clear_interrupt(
        dma::DmaPeriph::Dma1,
        DmaChannel::C5,
        DmaInterrupt::TransferComplete,
    );

    free(|cs| {
        access_global!(SPI, spi, cs);
        spi.stop_dma(DmaChannel::C5, None, dma::DmaPeriph::Dma1);
    });

    let mut spi2_cs = Pin::new(Port::B, 12, PinMode::Output);
    spi2_cs.set_high();

    println!("CH5 interrupt called")
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

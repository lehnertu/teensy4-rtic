//! Teensy 4.1 - log messages and data tags to SD card
//!
//! This template uses [RTIC v2](https://rtic.rs/2/book/en/)
//! for structuring the application.
//! 
//! create hex file : cargo objcopy --release -- -O ihex teensy4_rtic_sdcard.hex
//! flash with : teensy_loader_cli --mcu=TEENSY41 -w teensy4_rtic_sdcard.hex
//! connect USB log : minicom -D /dev/ttyACM0

#![no_std]
#![no_main]

mod card_logger;

use teensy4_panic as _;
use teensy4_panic::sos;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    sos()
}

// the Micro-SD card slot is connected as:
// 1: DAT2 : -     : SD_B0_04 (p47) : H2 : USDHC1_DATA2
// 2: DAT3 : CS    : SD_B0_05 (p46) : J2 : USDHC1_DATA3
// 3: CMD  : MOSI  : SD_B0_00 (p45) : J4 : USDHC1_CMD
// 4: 3.3V : +3.3V
// 5: CLK  : SCK   : SD_B0_01 (p44) : J3 : USDHC1_CLK
// 6: GND  : GND
// 7: DAT0 : MISO  : SD_B0_02 (p43) : J1 : USDHC1_DATA0
// 8: DAT1 : -     : SD_B0_03 (p42) : K1 : USDHC1_DATA1

// SPI cannot be used with the pin connections of the card socket
// we have to use SDIO

// Timing dividers 128 and 1 from 49.5 Mhz (nominal 387 kHz) => 500 kHz clock (with scope)
// Timing dividers 8 and 1 from 49.5 Mhz => 390 µs/block (confirmed with scope)
// about 8 MHz clock seen on scope => source clock seems to be 64 MHz instead of 49.5 MHz
// 16 µs request, 1 µs break, 370 µs block (3000 cycles, maybe some wait states before actual data)
// transmission should be 128 µs = 512*8 bits / 4*8 MHz

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers=[KPP])]
mod app {

    use rtic_monotonics::systick::prelude::*;

    use teensy4_bsp as bsp;
    // depends on imxrt-hal "0.5.3" features=["imxrt1060"]
    // depends on imxrt-ral "0.5" features=["imxrt1062"]
    // depends on embedded-hal "0.2"
    use bsp::board;
    use bsp::hal::iomuxc;
    use bsp::hal::ccm::clock_gate;
    use imxrt_log as logging;
    // use bsp::hal as hal;
    use imxrt_ral;
    use cortex_m::peripheral::DWT;

    use imxrt_usdhc::{BlockingSdioHost, DataRate, SDRPrescaler, Timing, Usdhc};
    use crate::card_logger::{CardLogger};

    // Create the type `Mono`. It will manage the SysTick timer, and use it to
    // generate 1000 interrupts per second
    // the systick counter is delivered as a system (up)time in ms
    systick_monotonic!(Mono, 1000);
    pub(crate) fn system_time_ms() -> u32 {
        Mono::now().ticks()
    }

    /// These resources are shared across tasks.
    #[shared]
    struct Shared {
        card_logger: CardLogger,
    }

    /// These resources are local to individual tasks.
    #[local]
    struct Local {
        led: board::Led,
        poller: logging::Poller,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {

        let mut core = cx.core;

        // Enable DWT cycle counter for timing measurements.
        core.DCB.enable_trace();
        DWT::unlock();
        core.DWT.enable_cycle_counter();

        // Specify 't40', 't41', or 'tmm' depending on which board you're using
        let board::Resources {
            mut pins,
            mut gpio2,
            usb,
            mut ccm,
            ..
        } = board::t41(cx.device);

        Mono::start(core.SYST, board::ARM_FREQUENCY);

        let led = board::led(&mut gpio2, pins.p13);

        let poller = logging::log::usbd(usb, logging::Interrupts::Enabled).unwrap();

        log::info!("Teensy 4.1 - RTIC controller");
        log::info!("starting initialization process");

        // --------------------
        // uSDHC peripheral (for SD card)
        // --------------------

        iomuxc::usdhc::prepare(&mut pins.p45); // CMD  -> GPIO_SD_B0_00 -> USDHC1_CMD
        iomuxc::usdhc::prepare(&mut pins.p44); // CLK  -> GPIO_SD_B0_01 -> USDHC1_CLK
        iomuxc::usdhc::prepare(&mut pins.p43); // DAT0 -> GPIO_SD_B0_02 -> USDHC1_DATA0
        iomuxc::usdhc::prepare(&mut pins.p42); // DAT1 -> GPIO_SD_B0_03 -> USDHC1_DATA1
        iomuxc::usdhc::prepare(&mut pins.p47); // DAT2 -> GPIO_SD_B0_04 -> USDHC1_DATA2
        iomuxc::usdhc::prepare(&mut pins.p46); // DAT3 -> GPIO_SD_B0_05 -> USDHC1_DATA3

        // Configure USDHC1 root clock: source = PLL2_PFD2 (396 MHz), divided by 8 → ~49.5 MHz
        // CSCMR1[USDHCn_CLK_SEL] selects 0: PLL2_PFD2 (396 MHz), 1: PLL2_PFD0 (352 MHz)
        // CSCMR1[USDHC1_CLK_SEL] = 0 (PLL2_PFD2, also the reset default)
        // CSCDR1[USDHC1_PODF]    = 0b111 (divide by 8), update while gated off
        imxrt_ral::modify_reg!(imxrt_ral::ccm, ccm, CSCMR1, USDHC1_CLK_SEL: 0);
        imxrt_ral::modify_reg!(imxrt_ral::ccm, ccm, CSCDR1, USDHC1_PODF: 0b111);

        // Enable USDHC1 clock gate (CCGR6, CG1)
        clock_gate::usdhc::<1>().set(&mut ccm, clock_gate::ON);

        // Create the uSDHC peripheral instance.
        let mut usdhc = unsafe { Usdhc::new(imxrt_ral::usdhc::USDHC1 as *const ()) };

        // Initial identification clock: 49.5 MHz / SDCLKFS(128) / DVS(1) ≈ 386 kHz (≤ 400 kHz per SD spec)
        usdhc.set_timing(Timing {
            divisor: 1,
            data_rate: DataRate::SingleDataRate(SDRPrescaler::Divide128),
        });

        let mut delay_ms = |ms: u32| {
            let cycles_per_ms = board::ARM_FREQUENCY / 1_000;
            cortex_m::asm::delay(cycles_per_ms.saturating_mul(ms));
        };

        // This initializes the card and reads its info registers (CID, CSD, SCR, SD status).
        // If supported it will switch to 4-bit mode
        // It seems to leave the clock at the initial slow speed
        let mut host = BlockingSdioHost::new(usdhc, &mut delay_ms).unwrap();

        delay_ms(50);
        log::info!("RCA: {:#06X}", host.rca().address());
        delay_ms(50);
        log::info!("CID: {:?}", host.cid());
        delay_ms(50);
        log::info!("CSD: {:?}", host.csd());
        delay_ms(50);
        log::info!("SCR: {:?}", host.scr());
        delay_ms(50);
        log::info!("SD status: {:?}", host.sd_status());

        // Log speed capabilities
        delay_ms(50);
        let csd = host.csd();
        let sd_status = host.sd_status();
        let tran_speed = csd.transfer_rate();
        let speed_class = sd_status.speed_class();
        let bus_width = sd_status.bus_width();
        log::info!(
            "Speed: TRAN_SPEED={}Mbit/s, Speed_Class={}, Bus_Width={:?}",
            tran_speed,
            speed_class,
            bus_width
        );

        // set higher clock speed: 49.5 MHz / SDCLKFS(8) / DVS(1) ≈ 6.2 MHz
        // TODO: verify that this clock rate is actually supported
        host.transport_mut().set_timing(Timing {
            divisor: 1,
            data_rate: DataRate::SingleDataRate(SDRPrescaler::Divide8),
        });
        delay_ms(50);

        // CardLogger takes ownership of the host.
        let mut card_logger = CardLogger::new(host);
        let _ = card_logger.log_text(b"CardLogger initialized");
        let _ = card_writer::spawn();
        let _ = card_ping::spawn();

        (
            Shared { card_logger },
            Local {
                led,
                poller,
            },
        )
    }

    /// This is a low-priority task that continuously checks if there are
    /// filled logging blocks that need to be written to SD card.
    /// It takes the LED pin for a write indicator.
    /// If there exists a full block a blocking write is performed.
    #[task(priority = 1, shared=[card_logger], local=[led])]
    async fn card_writer(mut cx: card_writer::Context) {
        loop {
            let flush_result = cx.shared.card_logger.lock(|logger| logger.flush(&mut *cx.local.led));
            match flush_result {
                Ok(()) => Mono::delay(1.millis()).await,
                Err(_err) => {
                    // at present all error handling we do is to wait a little and try again
                    Mono::delay(5.millis()).await;
                }
            }
        }
    }

    #[task(shared = [card_logger])]
    async fn card_ping(mut cx: card_ping::Context) {
        loop {
            let _ = cx
                .shared
                .card_logger
                .lock(|logger| logger.log_text(b"ping"));
            Mono::delay(100.millis()).await;
        }
    }

    #[task(binds = USB_OTG1, local=[poller])]
    fn usb_interrupt(cx: usb_interrupt::Context) {
        cx.local.poller.poll();
    }
}
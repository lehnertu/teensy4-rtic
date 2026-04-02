//! Teensy 4.1 - read/write blocks on SD card
//!
//! This template uses [RTIC v2](https://rtic.rs/2/book/en/)
//! for structuring the application.
//! 
//! create hex file : cargo objcopy --release -- -O ihex teensy4_rtic_sdcard.hex
//! flash with : teensy_loader_cli --mcu=TEENSY41 -w teensy4_rtic_sdcard.hex
//! connect USB log : minicom -D /dev/ttyACM0

#![no_std]
#![no_main]

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

// Timing dividers 128 and 1 from 49.5 Mhz (nominal 387 kHz) => 2760 µs/block = 370 kHz effective clock
// Timing dividers 16 and 1 from 49.5 Mhz (nominal 3.1 MHz) => 574 µs/block = 1.78 MHz effective clock
// Timing dividers 8 and 1 from 49.5 Mhz => 415 µs/block = 2.47 MHz effective clock

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers=[KPP])]
mod app {

    use core::fmt;

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

    // Create the type `Mono`. It will manage the SysTick timer, and use it to
    // generate 1000 interrupts per second
    systick_monotonic!(Mono, 1000);

    struct HexLine<'a>(&'a [u8]);

    impl fmt::Display for HexLine<'_> {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            for (idx, byte) in self.0.iter().enumerate() {
                if idx != 0 {
                    f.write_str(" ")?;
                }
                write!(f, "{:02X}", byte)?;
            }
            Ok(())
        }
    }

    /// These resources are shared across tasks.
    #[shared]
    struct Shared {
        host: BlockingSdioHost,
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
        let host = BlockingSdioHost::new(usdhc, &mut delay_ms).unwrap();

        // Spawn SD init task (it will delay itself so we can see the log messages)
        sd_init::spawn().unwrap();

        (
            Shared { host },
            Local {
                led,
                poller,
            },
        )
    }

    #[task(shared = [host])]
    async fn sd_init(mut cx: sd_init::Context) {

        // only after some time the USB connection is ready to receive logs
        Mono::delay(5_000.millis()).await;

        Mono::delay(50.millis()).await;
        cx.shared.host.lock(|host| {
            log::info!("RCA: {:#06X}", host.rca().address());
        });
        Mono::delay(50.millis()).await;
        cx.shared.host.lock(|host| {
            log::info!("CID: {:?}", host.cid());
        });
        Mono::delay(50.millis()).await;
        cx.shared.host.lock(|host| {
            log::info!("CSD: {:?}", host.csd());
        });
        Mono::delay(50.millis()).await;
        cx.shared.host.lock(|host| {
            log::info!("SCR: {:?}", host.scr());
        });
        Mono::delay(50.millis()).await;
        cx.shared.host.lock(|host| {
            log::info!("SD status: {:?}", host.sd_status());
        });
        
        // Log speed capabilities
        Mono::delay(50.millis()).await;
        cx.shared.host.lock(|host| {
            let csd = host.csd();
            let sd_status = host.sd_status();
            let tran_speed = csd.transfer_rate();
            let speed_class = sd_status.speed_class();
            let bus_width = sd_status.bus_width();
            log::info!("Speed: TRAN_SPEED={}Mbit/s, Speed_Class={}, Bus_Width={:?}", 
                       tran_speed, speed_class, bus_width);
        });

        // After card init and info logging, start periodic block 0 dumps.
        log_block0::spawn().unwrap();
    }

    #[task(shared = [host], local = [led])]
    async fn log_block0(mut cx: log_block0::Context) {

        // Idle state low. During a block read, set high for timing on scope.
        cx.local.led.clear();

        for _ in 0..3 {
            // read block 0 of the card
            let mut block0 = [0u8; 512];
            cx.local.led.set();
            let read_cycles = cx.shared.host.lock(|host| {
                let start = DWT::cycle_count();
                host.read_block(0, &mut block0).unwrap();
                DWT::cycle_count().wrapping_sub(start)
            });
            cx.local.led.clear();
            let read_us = (read_cycles as u64 * 1_000_000) / (board::ARM_FREQUENCY as u64);

            Mono::delay(50.millis()).await;
            log::info!("read_block(0): {} cycles ({} us)", read_cycles, read_us);
            Mono::delay(50.millis()).await;
            log::info!("Block 0:");
            for (row, chunk) in block0.chunks_exact(32).enumerate() {
                Mono::delay(50.millis()).await;
                log::info!("{:03X}: {}", row * 32, HexLine(chunk));
            }

            // Repeat block 0 dump every 2 seconds after finishing one dump.
            Mono::delay(2_000.millis()).await;
        }

        // After 3 times reading and logging block 0,
        // switch to high speed mode and run 3 high-speed reads before entering infinite high-speed logging.
        log_block0_high_speed::spawn().unwrap();
    }

    #[task(shared = [host])]
    async fn log_block0_high_speed(mut cx: log_block0_high_speed::Context) {

        // set higher clock speed: 49.5 MHz / SDCLKFS(8) / DVS(1) ≈ 6.2 MHz
        cx.shared.host.lock(|host| {
            host.transport_mut().set_timing(Timing {
                divisor: 1,
                data_rate: DataRate::SingleDataRate(SDRPrescaler::Divide8),
            });
        });
        Mono::delay(50.millis()).await;

        for _ in 0..3 {
            // read block 0 of the card
            let mut block0 = [0u8; 512];

            let read_cycles = cx.shared.host.lock(|host| {
                let start = DWT::cycle_count();
                host.read_block(0, &mut block0).unwrap();
                DWT::cycle_count().wrapping_sub(start)
            });
            let read_us = (read_cycles as u64 * 1_000_000) / (board::ARM_FREQUENCY as u64);

            Mono::delay(50.millis()).await;
            log::info!("read_block(0): {} cycles ({} us)", read_cycles, read_us);
            Mono::delay(50.millis()).await;
            log::info!("Block 0:");
            for (row, chunk) in block0.chunks_exact(32).enumerate() {
                Mono::delay(50.millis()).await;
                log::info!("{:03X}: {}", row * 32, HexLine(chunk));
            }

            // Repeat block 0 dump every 2 seconds after finishing one dump.
            Mono::delay(2_000.millis()).await;
        }

        // Write block 0 once, then continue with forever high-speed logging.
        write_block0::spawn().unwrap();
    }

    #[task(shared = [host])]
    async fn write_block0(mut cx: write_block0::Context) {

        // Start from an empty (zeroed) payload.
        let mut block0 = [0u8; 512];
        // Modify this buffer before writing.
        let tag = "Teensy RTIC FS tag log";
        let tag_bytes = tag.as_bytes();
        block0[..tag_bytes.len()].copy_from_slice(tag_bytes);

        let write_result = cx.shared.host.lock(|host| host.write_block(0, &block0));

        Mono::delay(50.millis()).await;
        match write_result {
            Ok(()) => {
                log::info!("write_block(0): done (512-byte buffer written)");
            }
            Err(err) => {
                log::error!("write_block(0) failed: {:?}", err);
            }
        }

        log_block0_high_speed_forever::spawn().unwrap();
    }

    #[task(shared = [host])]
    async fn log_block0_high_speed_forever(mut cx: log_block0_high_speed_forever::Context) {

        loop {
            // read block 0 of the card
            let mut block0 = [0u8; 512];

            let read_cycles = cx.shared.host.lock(|host| {
                let start = DWT::cycle_count();
                host.read_block(0, &mut block0).unwrap();
                DWT::cycle_count().wrapping_sub(start)
            });
            let read_us = (read_cycles as u64 * 1_000_000) / (board::ARM_FREQUENCY as u64);

            Mono::delay(50.millis()).await;
            log::info!("read_block(0): {} cycles ({} us)", read_cycles, read_us);
            Mono::delay(50.millis()).await;
            log::info!("Block 0:");
            for (row, chunk) in block0.chunks_exact(32).enumerate() {
                Mono::delay(50.millis()).await;
                log::info!("{:03X}: {}", row * 32, HexLine(chunk));
            }

            // Repeat block 0 dump every 2 seconds after finishing one dump.
            Mono::delay(2_000.millis()).await;
        }
    }

    #[task(binds = USB_OTG1, local=[poller])]
    fn usb_interrupt(cx: usb_interrupt::Context) {
        cx.local.poller.poll();
    }
}
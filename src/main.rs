// main.rs

#![no_std]
#![no_main]
#![feature(alloc_error_handler)]
// #![allow(non_snake_case)]

// #![deny(warnings)]
#![allow(unused_mut)]

extern crate alloc;
extern crate no_std_compat as std;

use alloc_cortex_m::CortexMHeap;
use core::alloc::Layout;
use panic_halt as _;

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

#[allow(clippy::empty_loop)]
#[alloc_error_handler]
fn oom(_: Layout) -> ! {
    loop {}
}

// use either RTIC line depending on hardware

// #[rtic::app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [DMA1_CHANNEL1, DMA1_CHANNEL2, DMA1_CHANNEL3])]
// #[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [DMA2_STREAM1, DMA2_STREAM2, DMA2_STREAM3])]
#[rtic::app(device = nrf52840_hal::pac, peripherals = true, dispatchers = [SWI3_EGU3, SWI4_EGU4, SWI5_EGU5])]
mod app {
    use cortex_m::asm;
    use nrf52840_hal as hal;

    use hal::usbd::{UsbPeripheral, Usbd};
    use hal::wdt;

    use dwt_systick_monotonic::{DwtSystick, ExtU32};
    use std::prelude::v1::*;
    use usb_device::{
        device::{UsbDeviceBuilder, UsbVidPid},
        prelude::*,
    };
    use usbd_serial::{SerialPort, USB_CLASS_CDC};

    /*
       use hal::{gpio::*, prelude::*};
       use hal::{pac::TIM2, timer::*};
    */

    const FREQ: u32 = 64_000_000;
    #[monotonic(binds = SysTick, default = true)]
    type MyMono = DwtSystick<FREQ>;

    #[shared]
    struct Shared {
        led_on: bool,
        usb_dev: UsbDevice<'static, Usbd<UsbPeripheral<'static>>>,
        serial: usbd_serial::SerialPort<'static, Usbd<UsbPeripheral<'static>>>,
    }

    #[local]
    struct Local {
        watchdog: wdt::WatchdogHandle<wdt::handles::Hdl0>,
        i: u8,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        // static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<UsbBusType>> = None;

        let dp = ctx.device;
        let clocks = hal::clocks::Clocks::new(dp.CLOCK)
            .enable_ext_hfosc()
            .set_lfclk_src_external(hal::clocks::LfOscConfiguration::ExternalNoBypass)
            .start_lfclk();

        // Initialize the monotonic
        let mono = DwtSystick::new(&mut ctx.core.DCB, ctx.core.DWT, ctx.core.SYST, FREQ);

        /*
               let serial = usbd_serial::SerialPort::new(unsafe { USB_BUS.as_ref().unwrap() });
               let usb_dev = UsbDeviceBuilder::new(
                   unsafe { USB_BUS.as_ref().unwrap() },
                   UsbVidPid(0x16c0, 0x27dd),
               )
               .manufacturer("Siuro Hacklab")
               .product("PWM controller")
               .serial_number("pwm42")
               .device_class(usbd_serial::USB_CLASS_CDC)
               .build();
        */

        // start petting the watchdog
        periodic::spawn().ok();

        // Start the hardware watchdog
        let mut watchdog = wdt::Watchdog::try_new(dp.WDT).unwrap();
        watchdog.set_lfosc_ticks(2 * 32768); // 2 seconds
        let wdt::Parts {
            watchdog: _wd,
            handles,
        } = watchdog.activate::<wdt::count::One>();
        let wdh0 = handles.0;

        let usb_bus = Usbd::new(UsbPeripheral::new(dp.USBD, &clocks));
        let mut serial = SerialPort::new(&usb_bus);

        let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")
            .device_class(USB_CLASS_CDC)
            .max_packet_size_0(64) // (makes control transfers 8x faster)
            .build();

        (
            Shared {
                led_on: false,
                usb_dev,
                serial,
            },
            Local {
                watchdog: wdh0,
                i: 0,
            },
            init::Monotonics(mono),
        )
    }

    // Background task, runs whenever no other tasks are running
    #[idle()]
    fn idle(_ctx: idle::Context) -> ! {
        loop {
            // Wait for interrupt...
            asm::wfi();
        }
    }

    // pet the watchdog to avoid hardware reset.
    #[task(priority=1, local=[watchdog])]
    fn periodic(ctx: periodic::Context) {
        ctx.local.watchdog.pet();
        periodic::spawn_after(500u32.millis()).ok();
    }
}
// EOF

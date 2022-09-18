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

use cortex_m::asm;
use nrf52840_hal as hal;

use dwt_systick_monotonic::{DwtSystick, ExtU32};
use std::prelude::v1::*;
use usb_device::{
    class_prelude::*,
    device::{UsbDeviceBuilder, UsbVidPid},
    prelude::*,
};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use hal::usbd::{UsbPeripheral, Usbd};
use hal::{clocks::*, wdt, Clocks};
use hal::{gpio::*, prelude::*};

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

#[allow(clippy::empty_loop)]
#[alloc_error_handler]
fn oom(_: Layout) -> ! {
    loop {}
}

const MY_USB_VID: u16 = 0x16c0;
const MY_USB_PID: u16 = 0x27dd;
const MY_USB_MFC: &str = "Siuro hacklab";
const MY_USB_PROD: &str = "nRF radio test1";
const MY_USB_SER: &str = "asdf1234";

#[rtic::app(device = nrf52840_hal::pac, peripherals = true, dispatchers = [SWI3_EGU3, SWI4_EGU4, SWI5_EGU5])]
mod app {
    use crate::*;

    const FREQ: u32 = 64_000_000;
    #[monotonic(binds = SysTick, default = true)]
    type MyMono = DwtSystick<FREQ>;

    #[shared]
    struct Shared {
        usb_dev: UsbDevice<'static, Usbd<UsbPeripheral<'static>>>,
        serial: usbd_serial::SerialPort<'static, Usbd<UsbPeripheral<'static>>>,
        led: Pin<Output<PushPull>>,
    }

    #[local]
    struct Local {
        // watchdog: wdt::WatchdogHandle<wdt::handles::Hdl0>,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        static mut USB_BUS: Option<UsbBusAllocator<Usbd<UsbPeripheral>>> = None;
        static mut CLOCKS: Option<Clocks<ExternalOscillator, Internal, LfOscStopped>> = None;

        let dp = ctx.device;
        let p0 = hal::gpio::p0::Parts::new(dp.P0);
        // let p1 = hal::gpio::p1::Parts::new(dp.P1);
        let mut led1 = p0.p0_06.into_push_pull_output(Level::High).degrade();
        // let mut led = p0.p0_08.into_push_pull_output(Level::High).degrade();
        // let mut led = p1.p1_09.into_push_pull_output(Level::High).degrade();
        let mut led = p0.p0_12.into_push_pull_output(Level::High).degrade();

        let clocks = hal::clocks::Clocks::new(dp.CLOCK).enable_ext_hfosc();

        // .set_lfclk_src_external(hal::clocks::LfOscConfiguration::ExternalNoBypass)
        // .start_lfclk();

        unsafe {
            CLOCKS.replace(clocks);
        }

        // Initialize the monotonic
        let mono = DwtSystick::new(&mut ctx.core.DCB, ctx.core.DWT, ctx.core.SYST, FREQ);

        /*
        // Start the hardware watchdog
        let mut watchdog = wdt::Watchdog::try_new(dp.WDT).unwrap();
        watchdog.set_lfosc_ticks(2 * 32768); // 2 seconds
        let wdt::Parts {
            watchdog: _wd,
            handles,
        } = watchdog.activate::<wdt::count::One>();
        let wdh0 = handles.0;
        // start petting the watchdog
        periodic::spawn().ok();
        */

        let usb_bus = Usbd::new(UsbPeripheral::new(dp.USBD, unsafe {
            CLOCKS.as_ref().unwrap()
        }));
        unsafe {
            USB_BUS.replace(usb_bus);
        }

        let mut serial = SerialPort::new(unsafe { USB_BUS.as_ref().unwrap() });

        let mut usb_dev = UsbDeviceBuilder::new(
            unsafe { USB_BUS.as_ref().unwrap() },
            UsbVidPid(MY_USB_VID, MY_USB_PID),
        )
        .manufacturer(MY_USB_MFC)
        .product(MY_USB_PROD)
        .serial_number(MY_USB_SER)
        .device_class(USB_CLASS_CDC)
        .max_packet_size_0(64) // (makes control transfers 8x faster)
        .build();

        ping::spawn().ok();
        led1.set_low().ok();

        (
            Shared {
                usb_dev,
                serial,
                led,
            },
            Local {},
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

    /*
       // pet the watchdog to avoid hardware reset.
       #[task(priority=1, local=[watchdog])]
       fn periodic(ctx: periodic::Context) {
           ctx.local.watchdog.pet();
           periodic::spawn_after(500u32.millis()).ok();
       }
    */

    #[task(priority=2, shared=[serial])]
    fn ping(ctx: ping::Context) {
        let ping::SharedResources { mut serial } = ctx.shared;

        (&mut serial,).lock(|serial| {
            serial.write("\r***PING!\n\r".as_bytes()).ok();
        });
        ping::spawn_after(1000u32.millis()).ok();
    }

    #[task(priority=5, binds=USBD, shared=[usb_dev, serial, led])]
    fn usb_poll(ctx: usb_poll::Context) {
        let usb_poll::SharedResources {
            mut usb_dev,
            mut serial,
            mut led,
        } = ctx.shared;

        (&mut usb_dev, &mut serial, &mut led).lock(|usb_dev, serial, led| {
            led.set_low().ok();
            if !usb_dev.poll(&mut [serial]) {
                return;
            }
            let mut buf = [0u8; 64];
            let count = match serial.read(&mut buf) {
                Ok(c) if c > 0 => c,
                _ => {
                    return;
                }
            };
            // Echo back in upper case
            for c in buf[0..count].iter_mut() {
                if 0x61 <= *c && *c <= 0x7a {
                    *c &= !0x20;
                }
            }
            serial.write(&buf[0..count]).ok();
        });
    }
}
// EOF

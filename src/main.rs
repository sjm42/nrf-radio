// main.rs

#![no_std]
#![no_main]
// #![allow(unused_mut)]
// #![allow(non_snake_case)]
// #![deny(warnings)]

use panic_halt as _;

use cortex_m::asm;
use nrf52840_hal as hal;

// use fugit::*;
use hal::usbd::{UsbPeripheral, Usbd};
use hal::{clocks::*, wdt, Clocks};
use hal::{gpio::*, pac::*, prelude::*};
use usb_device::{
    class_prelude::*,
    device::{UsbDeviceBuilder, UsbVidPid},
    prelude::*,
};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

mod mono;
use mono::{ExtU32, MonoTimer};

// const FREQ: u32 = 64_000_000;
const MY_USB_VID: u16 = 0x16c0;
const MY_USB_PID: u16 = 0x27dd;
const MY_USB_MFC: &str = "Siuro hacklab";
const MY_USB_PROD: &str = "nRF radio test1";
const MY_USB_SER: &str = "asdf1234";

#[rtic::app(device = hal::pac, peripherals = true, dispatchers = [SWI0_EGU0, SWI1_EGU1, SWI2_EGU2, SWI3_EGU3, SWI4_EGU4])]
mod app {

    use crate::*;

    #[monotonic(binds = TIMER1, default = true)]
    type MyMono = MonoTimer<TIMER1>;

    #[shared]
    struct Shared {
        usb_dev: UsbDevice<'static, Usbd<UsbPeripheral<'static>>>,
        serial: usbd_serial::SerialPort<'static, Usbd<UsbPeripheral<'static>>>,
    }

    #[local]
    struct Local {
        led1: Pin<Output<PushPull>>,
        led2: Pin<Output<PushPull>>,
        led3: Pin<Output<PushPull>>,
        led4: Pin<Output<PushPull>>,
        wdh0: wdt::WatchdogHandle<wdt::handles::Hdl0>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        static mut USB_BUS: Option<UsbBusAllocator<Usbd<UsbPeripheral>>> = None;
        static mut CLOCKS: Option<Clocks<ExternalOscillator, ExternalOscillator, LfOscStarted>> =
            None;

        let dp = ctx.device;
        let clocks = hal::clocks::Clocks::new(dp.CLOCK)
            .enable_ext_hfosc()
            .set_lfclk_src_external(hal::clocks::LfOscConfiguration::NoExternalNoBypass)
            .start_lfclk();

        unsafe {
            CLOCKS.replace(clocks);
        }

        // Initialize the monotonic
        let mono = MonoTimer::new(dp.TIMER1);

        let p0 = hal::gpio::p0::Parts::new(dp.P0);
        let p1 = hal::gpio::p1::Parts::new(dp.P1);

        let mut led1 = p0.p0_06.into_push_pull_output(Level::High).degrade();
        let led2 = p0.p0_12.into_push_pull_output(Level::High).degrade();
        let mut led3 = p0.p0_08.into_push_pull_output(Level::High).degrade();
        let led4 = p1.p1_09.into_push_pull_output(Level::High).degrade();

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
        if usb_dev.poll(&mut [&mut serial]) {
            led3.set_low().ok();
        }

        // ping::spawn_after(500u32.millis()).ok();
        // usb_poll::spawn().ok();

        led1.set_low().ok();

        //led.set_low().ok();
        // led_set::spawn().ok();
        led_blink::spawn_after(1000u32.millis()).ok();

        (
            Shared { usb_dev, serial },
            Local {
                led1,
                led2,
                led3,
                led4,
                wdh0,
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
    #[task(priority=1, local=[wdh0])]
    fn periodic(ctx: periodic::Context) {
        ctx.local.wdh0.pet();
        periodic::spawn_after(757u32.millis()).ok();
    }

    #[task(priority=2, capacity=2, local=[led1])]
    fn led_blink(ctx: led_blink::Context) {
        let led = ctx.local.led1;

        if led.is_set_low().unwrap() {
            led.set_high().ok();
        } else {
            led.set_low().ok();
        }

        led_blink::spawn_after(250u32.millis()).ok();
    }

    /*
    #[task(priority=2, capacity=2, shared=[serial])]
    fn ping(ctx: ping::Context) {
        let ping::SharedResources { mut serial } = ctx.shared;

        (&mut serial,).lock(|serial| {
            serial.write("\r***PING!\n\r".as_bytes()).ok();
        });
        ping::spawn_after(1000u32.millis()).ok();
    }
    */

    #[task(priority=5, binds=USBD, shared=[usb_dev, serial], local=[led2])]
    fn usb_poll(ctx: usb_poll::Context) {
        let usb_poll::SharedResources {
            mut usb_dev,
            mut serial,
        } = ctx.shared;
        let led = ctx.local.led2;
        led.set_low().ok();

        (&mut usb_dev, &mut serial).lock(|usb_dev, serial| {
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

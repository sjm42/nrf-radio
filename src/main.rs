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
        #[lock_free]
        led1: Pin<Output<PushPull>>,
        #[lock_free]
        led1_on: bool,
        led2: Pin<Output<PushPull>>,
        led2_on: bool,
        led3: Pin<Output<PushPull>>,
        led3_on: bool,
        led4: Pin<Output<PushPull>>,
        led4_on: bool,
    }

    #[local]
    struct Local {
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
        let led3 = p0.p0_08.into_push_pull_output(Level::High).degrade();
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
        wdt_periodic::spawn().ok();

        let usb_bus = Usbd::new(UsbPeripheral::new(dp.USBD, unsafe {
            CLOCKS.as_ref().unwrap()
        }));
        unsafe {
            USB_BUS.replace(usb_bus);
        }

        let serial = SerialPort::new(unsafe { USB_BUS.as_ref().unwrap() });
        let usb_dev = UsbDeviceBuilder::new(
            unsafe { USB_BUS.as_ref().unwrap() },
            UsbVidPid(MY_USB_VID, MY_USB_PID),
        )
        .manufacturer(MY_USB_MFC)
        .product(MY_USB_PROD)
        .serial_number(MY_USB_SER)
        .device_class(USB_CLASS_CDC)
        .max_packet_size_0(64) // (makes control transfers 8x faster)
        .build();

        // ping::spawn_after(500u32.millis()).ok();

        led1.set_low().ok();
        led_blink::spawn_after(1000u32.millis()).ok();
        usb_periodic::spawn().ok();

        (
            Shared {
                usb_dev,
                serial,
                led1,
                led2,
                led3,
                led4,
                led1_on: true,
                led2_on: false,
                led3_on: false,
                led4_on: false,
            },
            Local { wdh0 },
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
    fn wdt_periodic(ctx: wdt_periodic::Context) {
        ctx.local.wdh0.pet();
        wdt_periodic::spawn_after(757u32.millis()).ok();
    }

    #[task(priority=2, capacity=2, shared=[led1, led1_on])]
    fn led_blink(ctx: led_blink::Context) {
        let led_blink::SharedResources { led1, led1_on } = ctx.shared;

        if *led1_on {
            led1.set_high().ok();
            *led1_on = false;
        } else {
            led1.set_low().ok();
            *led1_on = true;
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

    #[task(priority=2, capacity=8, shared=[led2, led2_on])]
    fn led2_blink(ctx: led2_blink::Context, ms: u32) {
        let led2_blink::SharedResources {
            mut led2,
            mut led2_on,
        } = ctx.shared;

        (&mut led2, &mut led2_on).lock(|led2, led2_on| {
            if !(*led2_on) {
                led2.set_low().ok();
                *led2_on = true;
                led2_off::spawn_after(ms.millis()).unwrap();
            }
        });
    }

    #[task(priority=2, shared=[led2, led2_on])]
    fn led2_off(ctx: led2_off::Context) {
        let led2_off::SharedResources {
            mut led2,
            mut led2_on,
        } = ctx.shared;

        (&mut led2, &mut led2_on).lock(|led2, led2_on| {
            led2.set_high().ok();
            *led2_on = false;
        });
    }

    #[task(priority=2, capacity=8, shared=[led3, led3_on])]
    fn led3_blink(ctx: led3_blink::Context, ms: u32) {
        let led3_blink::SharedResources {
            mut led3,
            mut led3_on,
        } = ctx.shared;

        (&mut led3, &mut led3_on).lock(|led3, led3_on| {
            if !(*led3_on) {
                led3.set_low().ok();
                *led3_on = true;
                led3_off::spawn_after(ms.millis()).unwrap();
            }
        });
    }

    #[task(priority=2, shared=[led3, led3_on])]
    fn led3_off(ctx: led3_off::Context) {
        let led3_off::SharedResources {
            mut led3,
            mut led3_on,
        } = ctx.shared;

        (&mut led3, &mut led3_on).lock(|led3, led3_on| {
            led3.set_high().ok();
            *led3_on = false;
        });
    }

    #[task(priority=5, binds=USBD, shared=[usb_dev, serial])]
    fn usb_interrupt(ctx: usb_interrupt::Context) {
        let usb_interrupt::SharedResources {
            mut usb_dev,
            mut serial,
        } = ctx.shared;

        (&mut usb_dev, &mut serial).lock(|usb_dev, serial| {
            usb_poll(usb_dev, serial);
        });
        led2_blink::spawn(10).ok();
    }

    #[task(priority=3,  shared=[usb_dev, serial])]
    fn usb_periodic(ctx: usb_periodic::Context) {
        let usb_periodic::SharedResources {
            mut usb_dev,
            mut serial,
        } = ctx.shared;

        (&mut usb_dev, &mut serial).lock(|usb_dev, serial| {
            usb_poll(usb_dev, serial);
        });
        led3_blink::spawn(1).ok();
        usb_periodic::spawn_after(10.millis()).ok();
    }

    fn usb_poll<'a>(
        usb_dev: &mut UsbDevice<Usbd<UsbPeripheral<'a>>>,
        serial: &mut SerialPort<Usbd<UsbPeripheral<'a>>>,
    ) -> bool {
        if !usb_dev.poll(&mut [serial]) {
            return false;
        }
        let mut buf = [0u8; 64];
        let count = match serial.read(&mut buf) {
            Ok(c) if c > 0 => c,
            _ => {
                return false;
            }
        };
        serial_echo(serial, &mut buf[0..count]);
        true
    }

    fn serial_echo<'a>(serial: &'a mut SerialPort<Usbd<UsbPeripheral>>, data: &'a mut [u8]) {
        for c in data.iter_mut() {
            if 0x61 <= *c && *c <= 0x7a {
                *c &= !0x20;
            }
        }
        serial.write(data).ok();
    }
}
// EOF

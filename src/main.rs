// main.rs

#![no_std]
#![no_main]
#![feature(alloc_error_handler)]
// #![allow(unused_mut)]
// #![allow(non_snake_case)]
// #![deny(warnings)]

extern crate alloc;
extern crate no_std_compat as std;
use panic_halt as _;

use cortex_m::asm;
use nrf52840_hal as hal;

use hal::ieee802154::{self, Channel};
use hal::usbd::{UsbPeripheral, Usbd};
use hal::{clocks::*, wdt, Clocks};
use hal::{gpio::*, pac::*, prelude::*};
use usb_device::{
    class_prelude::*,
    device::{UsbDeviceBuilder, UsbVidPid},
    prelude::*,
};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use alloc_cortex_m::CortexMHeap;
use core::alloc::Layout;
use core::fmt::{self, Write};
use std::prelude::v1::*;

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

#[allow(clippy::empty_loop)]
#[alloc_error_handler]
fn oom(_: Layout) -> ! {
    loop {}
}
mod mono;
use mono::{ExtU32, MonoTimer};

pub struct MyUsbSerial {
    pub serial: SerialPort<'static, Usbd<UsbPeripheral<'static>>>,
}

impl fmt::Write for MyUsbSerial {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        s.bytes()
            .try_for_each(|c| {
                if c == b'\n' {
                    self.serial.write(&[b'\r'])?;
                }
                self.serial.write(&[c]).map(|_| ())
            })
            .map_err(|_| fmt::Error)
    }
}

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
        serial: MyUsbSerial,
        radio: ieee802154::Radio<'static>,
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
        let start = cortex_m_rt::heap_start() as usize;
        let size = 64 * 1024; // in bytes
        unsafe { crate::ALLOCATOR.init(start, size) }

        static mut USB_BUS: Option<UsbBusAllocator<Usbd<UsbPeripheral>>> = None;
        static mut CLOCKS: Option<Clocks<ExternalOscillator, ExternalOscillator, LfOscStarted>> =
            None;

        let dp = ctx.device;
        let clocks = hal::clocks::Clocks::new(dp.CLOCK)
            .enable_ext_hfosc()
            .set_lfclk_src_external(hal::clocks::LfOscConfiguration::NoExternalNoBypass)
            .start_lfclk();
        let clocks = unsafe { CLOCKS.insert(clocks) };

        // Initialize the monotonic
        let mono = MonoTimer::new(dp.TIMER1);

        let p0 = hal::gpio::p0::Parts::new(dp.P0);
        let p1 = hal::gpio::p1::Parts::new(dp.P1);

        let mut led1 = p0.p0_06.into_push_pull_output(Level::High).degrade();
        let led2 = p0.p0_12.into_push_pull_output(Level::High).degrade();
        let led3 = p0.p0_08.into_push_pull_output(Level::High).degrade();
        let led4 = p1.p1_09.into_push_pull_output(Level::High).degrade();

        let radio = {
            let mut radio = ieee802154::Radio::init(dp.RADIO, clocks);
            // set TX power to its maximum value
            radio.set_txpower(ieee802154::TxPower::Pos8dBm);
            radio
        };

        // Init USB
        let usb_bus = Usbd::new(UsbPeripheral::new(dp.USBD, clocks));
        let usb_bus = unsafe { USB_BUS.insert(usb_bus) };

        let serial = SerialPort::new(usb_bus);
        let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(MY_USB_VID, MY_USB_PID))
            .manufacturer(MY_USB_MFC)
            .product(MY_USB_PROD)
            .serial_number(MY_USB_SER)
            .device_class(USB_CLASS_CDC)
            .max_packet_size_0(64) // (makes control transfers 8x faster)
            .build();

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

        usb_periodic::spawn().ok();
        led_blink::spawn_after(1000u32.millis()).ok();
        radio_scan::spawn_after(3.secs()).ok();
        // ping::spawn_after(5.secs()).ok();

        led1.set_low().ok();

        (
            Shared {
                usb_dev,
                serial: MyUsbSerial { serial },
                radio,
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

    #[task(priority=2, capacity=2, shared=[serial])]
    fn ping(ctx: ping::Context) {
        let ping::SharedResources { mut serial } = ctx.shared;

        (&mut serial,).lock(|serial| {
            write!(serial, "\r***PING!\n\r").ok();
        });
        ping::spawn_after(5.secs()).ok();
    }

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
            usb_poll(usb_dev, &mut serial.serial);
        });
        led2_blink::spawn(50).ok();
    }

    #[task(priority=3,  shared=[usb_dev, serial])]
    fn usb_periodic(ctx: usb_periodic::Context) {
        let usb_periodic::SharedResources {
            mut usb_dev,
            mut serial,
        } = ctx.shared;

        (&mut usb_dev, &mut serial).lock(|usb_dev, serial| {
            usb_poll(usb_dev, &mut serial.serial);
        });
        // led3_blink::spawn(1).ok();
        usb_periodic::spawn_after(3.millis()).ok();
    }

    fn usb_poll(
        usb_dev: &mut UsbDevice<'static, Usbd<UsbPeripheral<'static>>>,
        serial: &mut SerialPort<'static, Usbd<UsbPeripheral<'static>>>,
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

    fn serial_echo<'a>(serial: &mut SerialPort<'a, Usbd<UsbPeripheral<'a>>>, data: &mut [u8]) {
        for c in data.iter_mut() {
            if 0x61 <= *c && *c <= 0x7a {
                *c &= !0x20;
            }
        }
        serial.write(data).ok();
    }

    fn to_chan(num: u16) -> Option<Channel> {
        match num {
            5 => Some(Channel::_11),
            10 => Some(Channel::_12),
            15 => Some(Channel::_13),
            20 => Some(Channel::_14),
            25 => Some(Channel::_15),
            30 => Some(Channel::_16),
            35 => Some(Channel::_17),
            40 => Some(Channel::_18),
            45 => Some(Channel::_19),
            50 => Some(Channel::_20),
            55 => Some(Channel::_21),
            60 => Some(Channel::_22),
            65 => Some(Channel::_23),
            70 => Some(Channel::_24),
            75 => Some(Channel::_25),
            80 => Some(Channel::_26),
            _ => None,
        }
    }

    #[task(priority=2, capacity=2, shared=[radio, serial])]
    fn radio_scan(ctx: radio_scan::Context) {
        let radio_scan::SharedResources {
            mut radio,
            mut serial,
        } = ctx.shared;
        // let packet = ieee802154::Packet::new();

        (&mut radio, &mut serial).lock(|radio, serial| {
            let mut edsum: u16 = 0;
            (5..=80).step_by(5).for_each(|i| {
                let channel = to_chan(i).unwrap_or(Channel::_11);
                let mhz = 2400u16 + channel as u16;
                radio.set_channel(channel);
                let ed = radio.energy_detection_scan(80);
                write!(serial, "Radio {} MHz ed: {}\r\n", mhz, ed).ok();
                edsum += ed as u16;
            });
            write!(serial, "*** Radio max ed: {}\r\n\r\n", edsum).ok();
        });

        radio_scan::spawn_after(1800u32.millis()).ok();
    }
}

// EOF

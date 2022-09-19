// main.rs

#![no_std]
#![no_main]
// #![allow(unused_mut)]

use panic_halt as _;

mod mono;
use mono::{ExtU32, MonoTimer};
use nrf52840_hal as hal;
// use cortex_m::asm;

use hal::{gpio::*, pac::*, prelude::*};

#[rtic::app(device = hal::pac, peripherals = true, dispatchers = [SWI0_EGU0, SWI1_EGU1, SWI2_EGU2, SWI3_EGU3, SWI4_EGU4])]
mod app {

    use crate::*;

    #[monotonic(binds = TIMER0, default = true)]
    type MyMono = MonoTimer<TIMER0>;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        i: u64,
        led: Pin<Output<PushPull>>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let dp = ctx.device;
        let _clocks = hal::clocks::Clocks::new(dp.CLOCK).enable_ext_hfosc();
        // .set_lfclk_src_external(hal::clocks::LfOscConfiguration::NoExternalNoBypass)
        // .start_lfclk();

        let mono = MonoTimer::new(dp.TIMER0);

        let p0 = hal::gpio::p0::Parts::new(dp.P0);
        let mut led1 = p0.p0_06.into_push_pull_output(Level::High).degrade();
        let mut led = p0.p0_12.into_push_pull_output(Level::High).degrade();

        led1.set_low().ok();
        led.set_low().ok();
        led_blink::spawn().ok();

        (Shared {}, Local { i: 0, led }, init::Monotonics(mono))
    }

    // Background task, runs whenever no other tasks are running
    #[idle(local=[i])]
    fn idle(ctx: idle::Context) -> ! {
        let idle::LocalResources { i } = ctx.local;

        loop {
            // Wait for interrupt...
            // asm::wfi();

            *i += 1; // we keep counting instead
        }
    }

    #[task(local=[led])]
    fn led_blink(ctx: led_blink::Context) {
        let led = ctx.local.led;
        if led.is_set_low().unwrap() {
            led.set_high().ok();
        } else {
            led.set_low().ok();
        }
        led_blink::spawn_after(1.secs()).ok();
    }
}
// EOF

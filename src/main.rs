//! This is a LED timer to help with morning stretches. It will blink one color between stretch timing,
//! and then blink another color for 60 seconds to time a stretch, and repeat.

#![no_std]
#![no_main]

use embedded_hal::prelude::_embedded_hal_watchdog_WatchdogDisable;
use embedded_time::rate::Extensions;
use esp32c3_hal::system::SystemExt;
use esp32c3_hal::{
    clock::ClockControl,
    pac,
    pulse_control::ClockSource,
    timer::TimerGroup,
    utils::{smartLedAdapter, SmartLedsAdapter},
    PulseControl, Rtc, IO,
};
use esp_backtrace as _;
use esp_hal_common::pulse_control::*;
use esp_hal_common::systimer::SystemTimer;
use esp_println::println;
use fugit::ExtU64;
use lc::animations::{Animatable, Animation};
use lc::utility::default_translation_array;
use lc::{default_animations, LightingController, LogicalStrip};
use lighting_controller as lc;
use rgb::RGB8;
use riscv_rt::entry;
use smart_leds::{brightness, colors::*, gamma, SmartLedsWrite};

#[entry]
fn main() -> ! {
    let peripherals = pac::Peripherals::take().unwrap();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt0 = timer_group0.wdt;
    let mut io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // Disable watchdogs
    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();

    // Configure RMT peripheral globally
    let mut pulse = PulseControl::new(
        peripherals.RMT,
        &mut system.peripheral_clock_control,
        ClockSource::APB,
        0,
        0,
        0,
    )
    .unwrap();

    pulse
        .channel0
        .set_idle_output_level(false)
        .set_carrier_modulation(false)
        .set_channel_divider(1)
        .set_idle_output(true);

    // We use one of the RMT channels to instantiate a `SmartLedsAdapter` which can
    // be used directly with all `smart_led` implementations
    // 16 is the number of LEDs in the smartLedAdapter.
    let mut led = <smartLedAdapter!(16)>::new(pulse.channel0, io.pins.gpio9);

    const NUM_LEDS: usize = 16;

    let frame_rate = 60.Hz();
    let frame_rate_in_ticks = SystemTimer::TICKS_PER_SECOND / frame_rate.0 as u64;
    let mut color_buffer: [RGB8; NUM_LEDS] = [BLACK; NUM_LEDS];

    let ls = LogicalStrip::new(&mut color_buffer);
    let translation_array: [usize; NUM_LEDS] = default_translation_array(0);
    let animation =
        &mut Animation::new(default_animations::ANI_TEST, translation_array, frame_rate);
    let animation_array: [&mut dyn Animatable; 1] = [animation];
    let mut lc = LightingController::new(ls, animation_array, frame_rate);

    println!("Debug printing enabled.");

    SystemTimer::TICKS_PER_SECOND;
    let mut last_update_time = SystemTimer::now();
    loop {
        if SystemTimer::now() > (last_update_time + frame_rate_in_ticks) {
            last_update_time = SystemTimer::now();

            lc.update();
            // println!("{:?}", lc.logical_strip.color_buffer.iter().copied());

            led.write(brightness(
                gamma(lc.logical_strip.color_buffer.iter().copied()),
                100,
            ))
            .unwrap();
        }
    }
}

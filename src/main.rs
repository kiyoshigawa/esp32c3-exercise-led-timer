//! This is a LED timer to help with morning stretches. It will blink one color between stretch timing,
//! and then blink another color for 60 seconds to time a stretch, and repeat.

#![no_std]
#![no_main]

use embedded_hal::prelude::_embedded_hal_watchdog_WatchdogDisable;
use embedded_hal::timer;
use embedded_time::rate::Extensions;
use esp32c3_hal::{
    clock::ClockControl,
    peripherals,
    prelude::_esp_hal_system_SystemExt,
    pulse_control::ClockSource,
    timer::TimerGroup,
    utils::{smartLedAdapter, SmartLedsAdapter},
    PulseControl, Rtc, IO,
};
use esp_backtrace as _;
use esp_hal_common::ledc::channel::Number::Channel0;
use esp_hal_common::ledc::timer::config::Config;
use esp_hal_common::ledc::timer::*;
use esp_hal_common::ledc::{channel, LSGlobalClkSource, LowSpeed, LEDC};
use esp_hal_common::prelude::{_esp_hal_ledc_channel_ChannelIFace, _esp_hal_ledc_timer_TimerIFace};
use esp_hal_common::pulse_control::*;
use esp_hal_common::systimer::SystemTimer;
use esp_println::println;
use fugit::RateExtU32;
use lc::animations::{Animatable, Animation};
use lc::utility::default_translation_array;
use lc::{default_animations, LightingController, LogicalStrip};
use lighting_controller as lc;
use rgb::RGB8;
use riscv_rt::entry;
use smart_leds::{brightness, colors::*, gamma, SmartLedsWrite};

#[entry]
fn main() -> ! {
    let peripherals = peripherals::Peripherals::take();
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

    let led_pwm_pin = io.pins.gpio0.into_push_pull_output();

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

    let mut test_pwm_led = LEDC::new(
        peripherals.LEDC,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    test_pwm_led.set_global_slow_clock(LSGlobalClkSource::APBClk);
    let mut test_led_timer = test_pwm_led.get_timer::<LowSpeed>(Number::Timer2);

    test_led_timer
        .configure(Config {
            duty: config::Duty::Duty5Bit,
            clock_source: LSClockSource::APBClk,
            frequency: RateExtU32::kHz(25u32),
        })
        .unwrap();

    let mut pwm_channel = test_pwm_led.get_channel(Channel0, led_pwm_pin);
    pwm_channel
        .configure(channel::config::Config {
            timer: &(test_led_timer),
            duty_pct: 0,
        })
        .unwrap();

    // We use one of the RMT channels to instantiate a `SmartLedsAdapter` which can
    // be used directly with all `smart_led` implementations
    // 16 is the number of LEDs in the smartLedAdapter.
    let mut led = <smartLedAdapter!(16)>::new(pulse.channel0, io.pins.gpio9);

    const NUM_LEDS: usize = 16;

    let frame_rate = embedded_time::rate::Extensions::Hz(60);
    let frame_rate_in_ticks = SystemTimer::TICKS_PER_SECOND / frame_rate.0 as u64;
    let mut color_buffer: [RGB8; NUM_LEDS] = [BLACK; NUM_LEDS];

    let mut ls = LogicalStrip::new(&mut color_buffer);
    let translation_array: [usize; NUM_LEDS] = default_translation_array(0);
    let animation = &mut Animation::<NUM_LEDS>::new(default_animations::ANI_TEST, frame_rate);
    let animation_array: [&mut dyn Animatable; 1] = [animation];
    let mut lc = LightingController::new(animation_array, frame_rate);

    println!("Debug printing enabled.");

    let mut pwm_index = 0;
    let mut last_update_time = SystemTimer::now();
    loop {
        if SystemTimer::now() > (last_update_time + frame_rate_in_ticks) {
            pwm_index += 1;
            if pwm_index > 100 {
                pwm_index = 0;
            }
            pwm_channel
                .configure(channel::config::Config {
                    timer: &(test_led_timer),
                    duty_pct: pwm_index,
                })
                .unwrap();

            last_update_time = SystemTimer::now();

            lc.update(&mut ls);
            // println!("{:?}", lc.logical_strip.color_buffer.iter().copied());

            led.write(brightness(gamma(ls.color_buffer.iter().copied()), 100))
                .unwrap();
        }
    }
}

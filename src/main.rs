//! This is a LED timer to help with morning stretches. It will blink one color between stretch timing,
//! and then blink another color for 60 seconds to time a stretch, and repeat.

#![no_std]
#![no_main]

use embedded_hal::prelude::{
    _embedded_hal_blocking_delay_DelayMs, _embedded_hal_watchdog_WatchdogDisable,
};
use embedded_time::rate::Extensions;
use esp32c3_hal::system::SystemExt;
use esp32c3_hal::{
    clock::ClockControl,
    pac,
    pac::Peripherals,
    pulse_control::ClockSource,
    timer::TimerGroup,
    utils::{smartLedAdapter, SmartLedsAdapter},
    Delay, PulseControl, Rtc, IO,
};
use esp_backtrace as _;
use esp_hal_common::gpio;
use esp_hal_common::pulse_control::*;
use esp_println::println;
use fugit::NanosDuration;
use lc::animations::{Animatable, Animation};
use lc::utility::default_translation_array;
use lc::{default_animations, LightingController, LogicalStrip};
use lighting_controller as lc;
use rgb::RGB8;
use riscv::asm::delay;
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
    let mut configured_pulse = pulse.channel0.assign_pin(io.pins.gpio9);

    // We use one of the RMT channels to instantiate a `SmartLedsAdapter` which can
    // be used directly with all `smart_led` implementations
    // 16 is the number of LEDs in the smartLedAdapter.
    // let mut led = <smartLedAdapter!(4)>::new(pulse.channel0, io.pins.gpio9);
    // let one_high_pulse: u32 = (0 << 31) | (45 << 16) | (1 << 15) | (51);
    // let one_low_pulse: u32 = (0 << 31) | (70 << 16) | (1 << 15) | (26);

    let high_pulse = PulseCode {
        level1: true,
        length1: NanosDuration::<u32>::from_ticks(96),
        level2: false,
        length2: NanosDuration::<u32>::from_ticks(104),
    };
    let low_pulse = PulseCode {
        level1: true,
        length1: NanosDuration::<u32>::from_ticks(40),
        level2: false,
        length2: NanosDuration::<u32>::from_ticks(160),
    };
    let end_pulse = PulseCode {
        level1: false,
        length1: NanosDuration::<u32>::from_ticks(0),
        level2: false,
        length2: NanosDuration::<u32>::from_ticks(0),
    };

    const NUM_PULSES: usize = 72;
    let mut high_pulses = [high_pulse; NUM_PULSES + 1];
    let mut low_pulses = [high_pulse; NUM_PULSES + 1];
    high_pulses[high_pulses.len() - 1] = end_pulse;
    low_pulses[low_pulses.len() - 1] = end_pulse;

    // let mut pulses_as_u32 = [0; NUM_PULSES + 1];
    //
    // high_pulses
    //     .into_iter()
    //     .map(|x| u32::from(x))
    //     .enumerate()
    //     .for_each(|(i, x)| pulses_as_u32[i] = x);
    //
    // println!("{:?}", pulses_as_u32);

    // Initialize the Delay peripheral, and use it to toggle the LED state in a
    // loop.
    let mut delay = Delay::new(&clocks);

    const NUM_LEDS: usize = 4;

    let frame_rate = 60.Hz();
    let mut color_buffer: [RGB8; NUM_LEDS] = [CYAN; NUM_LEDS];
    color_buffer[1] = RED;
    color_buffer[2] = BLUE;
    color_buffer[3] = PURPLE;

    let ls = LogicalStrip::new(&mut color_buffer);
    let translation_array: [usize; NUM_LEDS] = default_translation_array(0);
    let animation =
        &mut Animation::new(default_animations::ANI_TEST, translation_array, frame_rate);
    let animation_array: [&mut dyn Animatable; 1] = [animation];
    let mut lc = LightingController::new(ls, animation_array, frame_rate);

    println!("Debug printing enabled.");
    configured_pulse.send_pulse_sequence(RepeatMode::SingleShot, &mut low_pulses);

    loop {
        // need to add time limiter, will do later
        // lc.update();
        // println!("{:?}", lc.logical_strip.color_buffer.iter().copied());
        // TODO: Figure out why led.write breaks when using more than 1 LED.
        // It looks like the problem is that we can't write the RAM buffer on the
        // RMT peripheral faster than it is sending it, hence the extra LED pulse sends.

        // led.write(brightness(gamma(color_buffer.iter().copied()), 100))
        //     .unwrap();
        delay.delay_ms(10000_u32);
        println!("One main loop has passed");
        configured_pulse.send_pulse_sequence(RepeatMode::SingleShot, &mut high_pulses);
    }
}

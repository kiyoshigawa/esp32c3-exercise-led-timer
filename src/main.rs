//! This is a LED timer to help with morning stretches. It will blink one color between stretch timing,
//! and then blink another color for 60 seconds to time a stretch, and repeat.

#![no_std]
#![no_main]

use esp32c3_hal::{
    clock::ClockControl,
    pac,
    prelude::*,
    pulse_control::ClockSource,
    timer::TimerGroup,
    utils::{smartLedAdapter, SmartLedsAdapter},
    Delay, PulseControl, Rtc, IO,
};
use esp_backtrace as _;
use esp_println::println;
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
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // Disable watchdogs
    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();

    // Configure RMT peripheral globally
    let pulse = PulseControl::new(
        peripherals.RMT,
        &mut system.peripheral_clock_control,
        ClockSource::APB,
        0,
        0,
        0,
    )
    .unwrap();

    // We use one of the RMT channels to instantiate a `SmartLedsAdapter` which can
    // be used directly with all `smart_led` implementations
    let mut led = <smartLedAdapter!(1)>::new(pulse.channel0, io.pins.gpio8);

    // Initialize the Delay peripheral, and use it to toggle the LED state in a
    // loop.
    let mut delay = Delay::new(&clocks);

    let setup_color = [RED];
    let stretch_color = [GREEN];
    let blink_color = [BLACK];

    println!("Debug printing enabled.");

    loop {
        for _ in 0..3 {
            led.write(brightness(gamma(setup_color.iter().cloned()), 30))
                .unwrap();
            delay.delay_ms(4_750u16);
            led.write(brightness(gamma(blink_color.iter().cloned()), 30))
                .unwrap();
            delay.delay_ms(250u16);
        }
        for _ in 0..6 {
            led.write(brightness(gamma(stretch_color.iter().cloned()), 30))
                .unwrap();
            delay.delay_ms(9_750u16);
            led.write(brightness(gamma(blink_color.iter().cloned()), 30))
                .unwrap();
            delay.delay_ms(250u16);
        }
    }
}

#![no_std]
#![no_main]

use cortex_m::delay::Delay;
use embedded_hal::digital::v2::OutputPin;
use rp_pico::hal::gpio::bank0::Gpio13;
use rp_pico::hal::gpio::{bank0, FunctionSio, FunctionSioOutput, Pin, PullDown, SioOutput};
use rp_pico::hal::{clocks, clocks::Clock, sio::Sio, watchdog::Watchdog};
use rp_pico::pac::{CorePeripherals, Peripherals};
use rp_pico::Pins;

extern crate panic_halt;

fn set_leds_dcf77(
    l12: &mut Pin<bank0::Gpio12, FunctionSioOutput, PullDown>,
    l13: &mut Pin<Gpio13, FunctionSio<SioOutput>, PullDown>,
    l14: &mut Pin<bank0::Gpio14, FunctionSioOutput, PullDown>,
    s: [bool; 7],
) {
    if s[4] {
        l12.set_high().unwrap();
    } else {
        l12.set_low().unwrap();
    }
    if s[5] {
        l13.set_high().unwrap();
    } else {
        l13.set_low().unwrap();
    }
    if s[6] {
        l14.set_high().unwrap();
    } else {
        l14.set_low().unwrap();
    }
}

#[rp_pico::entry]
fn main() -> ! {
    let mut pac = Peripherals::take().unwrap();
    let core = CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // boilerplate from the rp2040 template:
    let clocks = clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let mut dcf77_led_time = pins.gpio12.into_push_pull_output();
    let mut dcf77_led_bit = pins.gpio13.into_push_pull_output();
    let mut dcf77_led_error = pins.gpio14.into_push_pull_output();
    let mut msf_led_time = pins.gpio2.into_push_pull_output();
    let mut msf_led_bit_a = pins.gpio3.into_push_pull_output();
    let mut msf_led_bit_b = pins.gpio4.into_push_pull_output();
    let mut msf_led_error = pins.gpio5.into_push_pull_output();

    let mut state: [bool; 7] = [false, false, false, false, true, true, true];

    let mut set_leds_msf = |s: &[bool;7]| {
        if s[0] {
            msf_led_time.set_high().unwrap();
        } else {
            msf_led_time.set_low().unwrap();
        }
        if s[1] {
            msf_led_bit_a.set_high().unwrap();
        } else {
            msf_led_bit_a.set_low().unwrap();
        }
        if s[2] {
            msf_led_bit_b.set_high().unwrap();
        } else {
            msf_led_bit_b.set_low().unwrap();
        }
        if s[3] {
            msf_led_error.set_high().unwrap();
        } else {
            msf_led_error.set_low().unwrap();
        }
    };

    // each pin is a different type so using an array (or a vec!, not in no_std) does not work.
    // passing pins to set_leds() instead of the individual pins does not work:
    // - cannot `let mut` a Pin<...> twice because pins.gpioXY is moved if we declare pins:&Pins
    // - declaring pins without borrow gives move-after-use in main()
    loop {
        set_leds_msf(&state); // must borrow state here because implicit borrow is immutable which confuses the for loop below
        set_leds_dcf77(
            &mut dcf77_led_time,
            &mut dcf77_led_bit,
            &mut dcf77_led_error,
            state,
        );
        delay.delay_ms(300);
        for b in state.iter_mut() {
            *b = !*b;
        }
    }
}

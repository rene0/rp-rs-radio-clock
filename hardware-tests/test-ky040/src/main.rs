#![no_std]
#![no_main]

use embedded_hal::digital::v2::{InputPin, OutputPin};
extern crate panic_halt;
use rp_pico::hal::sio::Sio;
use rp_pico::pac::Peripherals;
use rp_pico::Pins;

// see also some examples from the internet:
// https://www.brainy-bits.com/post/best-code-to-use-with-a-ky-040-rotary-encoder-let-s-find-out
// https://gist.github.com/mumrah/ccd82aa13e784e77b788810dbaa8f4a3 for software SW debounce
// https://michiel.vanderwulp.be/domotica/Modules/KY040-Rotary-Encoder/KY-040-Science.pdf
// https://www.best-microcontroller-projects.com/rotary-encoder.html
// https://arduino.stackexchange.com/questions/69307/ky-040-rotary-encoder-skipping-steps
fn test_ky040(pins: Pins) -> ! {
    let sw_pin = pins.gpio7.into_pull_up_input(); // into_pull_down_input() always gives LOW output
    let dt_pin = pins.gpio8.into_floating_input();
    let clk_pin = pins.gpio9.into_floating_input();
    let mut led_pin = pins.led.into_push_pull_output();
    let mut msf_led_bit_a = pins.gpio3.into_push_pull_output();
    let mut msf_led_bit_b = pins.gpio4.into_push_pull_output();
    let mut rotation_start = clk_pin.is_high().unwrap();
    let mut rotation_now;
    // test "write to I/O-port" and pushing a button --> OK
    loop {
        // SW logic is negative
        if sw_pin.is_low().unwrap() {
            led_pin.set_high().unwrap();
        } else {
            led_pin.set_low().unwrap();
        }
        rotation_now = clk_pin.is_high().unwrap();
        if rotation_now != rotation_start {
            // Use the DT pin to found out the direction.
            // The equivalent code in MicroPython needs debouncing, but at
            // least with two indicator LEDs this code seems fine.
            if dt_pin.is_high().unwrap() != rotation_now {
                // clockwise
                msf_led_bit_a.set_low().unwrap();
                msf_led_bit_b.set_high().unwrap();
            } else {
                // counterclockwise
                msf_led_bit_a.set_high().unwrap();
                msf_led_bit_b.set_low().unwrap();
            }
        }
        rotation_start = rotation_now;
    }
}

#[rp_pico::entry]
fn main() -> ! {
    let mut pac = Peripherals::take().unwrap();
    let sio = Sio::new(pac.SIO);

    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    test_ky040(pins);
}

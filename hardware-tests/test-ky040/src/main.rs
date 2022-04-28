#![no_std]
#![no_main]

use bsp::hal::{pac, sio::Sio};
use cortex_m_rt::entry;
use defmt_rtt as _;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use panic_halt as _;
use rp_pico as bsp;

fn test_ky040(pins: bsp::Pins) -> ! {
    let sw_pin = pins.gpio7.into_pull_down_input(); // perhaps into_floating_input() works too
    let dt_pin = pins.gpio8.into_pull_down_input();
    let clk_pin = pins.gpio9.into_pull_down_input();
    let mut led_pin = pins.led.into_push_pull_output();
    let mut npl_led_bit_a = pins.gpio3.into_push_pull_output();
    let mut npl_led_bit_b = pins.gpio4.into_push_pull_output();
    let mut rotation_start = clk_pin.is_low().unwrap();
    let mut rotation_now;
    // test "write to I/O-port" and pushing a button --> OK
    loop {
        if sw_pin.is_low().unwrap() {
            led_pin.set_high().unwrap();
        } else {
            led_pin.set_low().unwrap();
        }
        rotation_now = clk_pin.is_low().unwrap();
        if rotation_now != rotation_start {
            // Use the DT pin to found out the direction.
            // This probably needs debouncing.
            if dt_pin.is_low().unwrap() != rotation_now {
                // clockwise
                npl_led_bit_a.set_low().unwrap();
                npl_led_bit_b.set_high().unwrap();
            } else {
                // counterclockwise
                npl_led_bit_a.set_high().unwrap();
                npl_led_bit_b.set_low().unwrap();
            }
        }
        rotation_start = rotation_now;
    }
}

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let sio = Sio::new(pac.SIO);

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    test_ky040(pins);
}

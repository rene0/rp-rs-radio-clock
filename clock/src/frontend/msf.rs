use crate::{frontend, set_time_led};
use core::cmp::Ordering;
use core::fmt::Write;
use embedded_hal::digital::v2::OutputPin;
use heapless::String;
use msf60_utils::MSFUtils;
use rp_pico::hal::gpio::{bank0, FunctionSioOutput, Pin, PullDown};

/// Put the LEDs in their initial state.
pub fn init_leds(
    led_time: &mut Pin<bank0::Gpio2, FunctionSioOutput, PullDown>,
    led_bit_a: &mut Pin<bank0::Gpio3, FunctionSioOutput, PullDown>,
    led_bit_b: &mut Pin<bank0::Gpio4, FunctionSioOutput, PullDown>,
    led_error: &mut Pin<bank0::Gpio5, FunctionSioOutput, PullDown>,
) {
    led_time.set_high().unwrap();
    led_bit_a.set_low().unwrap();
    led_bit_b.set_low().unwrap();
    led_error.set_high().unwrap();
}

/// Turn the time led on or off depending on whether a new second or minute arrived.
pub fn update_time_led(
    tick: u8,
    msf: &MSFUtils,
    led_time: &mut Pin<bank0::Gpio2, FunctionSioOutput, PullDown>,
) {
    set_time_led!(tick, msf, led_time);
}

/// Turn the bit LEDs (is-one and error) on or off.
pub fn update_bit_leds(
    tick: u8,
    msf: &MSFUtils,
    led_bit_a: &mut Pin<bank0::Gpio3, FunctionSioOutput, PullDown>,
    led_bit_b: &mut Pin<bank0::Gpio4, FunctionSioOutput, PullDown>,
    led_error: &mut Pin<bank0::Gpio5, FunctionSioOutput, PullDown>,
) {
    if tick == 0 {
        led_bit_a.set_low().unwrap();
        led_bit_b.set_low().unwrap();
        led_error.set_low().unwrap();
    } else if msf.get_current_bit_a().is_none() || msf.get_current_bit_b().is_none() {
        led_bit_a.set_low().unwrap();
        led_bit_b.set_low().unwrap();
        led_error.set_high().unwrap();
    } else {
        if msf.get_current_bit_a() == Some(true) {
            led_bit_a.set_high().unwrap();
        } else {
            led_bit_a.set_low().unwrap();
        }
        if msf.get_current_bit_b() == Some(true) {
            led_bit_b.set_high().unwrap();
        } else {
            led_bit_b.set_low().unwrap();
        }
        led_error.set_low().unwrap();
    }
}

/// Return the status overview as a compact string
pub fn str_status(msf: &MSFUtils) -> String<14> {
    let mut str_buf = String::<14>::from("");
    write!(
        str_buf,
        "{}{}{}{}{} {}{}",
        frontend::str_jumps(msf.get_radio_datetime()),
        frontend::str_parity(msf.get_parity_4(), true, 'd'),
        frontend::str_parity(msf.get_parity_3(), true, 'c'),
        frontend::str_parity(msf.get_parity_2(), true, 'b'),
        frontend::str_parity(msf.get_parity_1(), true, 'a'),
        str_minute_length(msf),
        if msf.end_of_minute_marker_present(false) {
            ' '
        } else {
            '!'
        },
    )
    .unwrap();
    str_buf
}

/// Return a string version of the given value, truncated to one digit with optional minus sign,
/// or ** for None.
fn str_i2(value: Option<i8>) -> String<2> {
    let mut s = String::<2>::from("");
    if value.is_some() {
        write!(s, "{:>-2}", value.unwrap()).unwrap();
    } else {
        write!(s, "**").unwrap();
    }
    s
}

/// Return a compact string with miscellaneous information
pub fn str_misc(msf: &MSFUtils) -> String<3> {
    let mut str_buf = String::<3>::from("");
    write!(
        str_buf,
        "{}{}",
        frontend::str_dst(msf.get_radio_datetime()),
        str_i2(msf.get_dut1())
    )
    .unwrap();
    str_buf
}

/// Return a character representation of the minute length status.
fn str_minute_length(msf: &MSFUtils) -> char {
    match msf.get_second().cmp(&msf.get_minute_length()) {
        Ordering::Less => '<',
        Ordering::Equal => ' ',
        Ordering::Greater => '>',
    }
}

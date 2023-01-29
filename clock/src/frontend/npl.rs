use crate::frontend;
use core::cmp::Ordering;
use core::fmt::Write;
use embedded_hal::digital::v2::OutputPin;
use heapless::String;
use npl_utils::NPLUtils;
use rp_pico::hal::gpio::{bank0, Pin, PushPullOutput};

/// Put the LEDs in their initial state.
pub fn init_leds(
    led_time: &mut Pin<bank0::Gpio2, PushPullOutput>,
    led_bit_a: &mut Pin<bank0::Gpio3, PushPullOutput>,
    led_bit_b: &mut Pin<bank0::Gpio4, PushPullOutput>,
    led_error: &mut Pin<bank0::Gpio5, PushPullOutput>,
) {
    led_time.set_high().unwrap();
    led_bit_a.set_low().unwrap();
    led_bit_b.set_low().unwrap();
    led_error.set_high().unwrap();
}

/// Turn the time led on or off depending on whether a new second or minute arrived.
pub fn update_time_led(tick: u8, npl: &NPLUtils, led_time: &mut Pin<bank0::Gpio2, PushPullOutput>) {
    if tick == 0 {
        led_time.set_high().unwrap();
    } else if (!npl.get_new_minute() && tick >= crate::FRAMES_PER_SECOND * 2 / 10)
        || (npl.get_new_minute() && tick >= crate::FRAMES_PER_SECOND * 8 / 10)
    {
        led_time.set_low().unwrap();
    }
}

/// Turn the bit LEDs (is-one and error) on or off.
pub fn update_bit_leds(
    tick: u8,
    npl: &NPLUtils,
    led_bit_a: &mut Pin<bank0::Gpio3, PushPullOutput>,
    led_bit_b: &mut Pin<bank0::Gpio4, PushPullOutput>,
    led_error: &mut Pin<bank0::Gpio5, PushPullOutput>,
) {
    if tick == 0 {
        led_bit_a.set_low().unwrap();
        led_bit_b.set_low().unwrap();
        led_error.set_low().unwrap();
    } else if npl.get_current_bit_a().is_none() || npl.get_current_bit_b().is_none() {
        led_bit_a.set_low().unwrap();
        led_bit_b.set_low().unwrap();
        led_error.set_high().unwrap();
    } else {
        if npl.get_current_bit_a() == Some(true) {
            led_bit_a.set_high().unwrap();
        } else {
            led_bit_a.set_low().unwrap();
        }
        if npl.get_current_bit_b() == Some(true) {
            led_bit_b.set_high().unwrap();
        } else {
            led_bit_b.set_low().unwrap();
        }
        led_error.set_low().unwrap();
    }
}

/// Return the status overview as a compact string
pub fn str_status(npl: &NPLUtils) -> String<12> {
    let mut str_buf = String::<12>::from("");
    write!(
        str_buf,
        "{}{}{}{}{}{}",
        frontend::str_jumps(npl.get_radio_datetime()),
        frontend::str_parity(npl.get_parity_4(), true, 'd'),
        frontend::str_parity(npl.get_parity_3(), true, 'c'),
        frontend::str_parity(npl.get_parity_2(), true, 'b'),
        frontend::str_parity(npl.get_parity_1(), true, 'a'),
        str_minute_length(npl),
    )
    .unwrap();
    str_buf
}

/// Return a compact string with miscellaneous information
pub fn str_misc(npl: &NPLUtils) -> String<1> {
    let mut str_buf = String::<1>::from("");
    write!(str_buf, "{}", frontend::str_dst(npl.get_radio_datetime())).unwrap();
    str_buf
}

/// Return a character representation of the minute length status.
fn str_minute_length(npl: &NPLUtils) -> char {
    match npl.get_second().cmp(&npl.get_minute_length()) {
        Ordering::Less => '<',
        Ordering::Equal => ' ',
        Ordering::Greater => '>',
    }
}

use crate::{frontend, set_time_led};
use core::cmp::Ordering;
use core::fmt::Write;
use dcf77_utils::DCF77Utils;
use heapless::String;

/// Put the LEDs in their initial state.
pub fn init_leds() -> [bool; 3] {
    [true, false, true]
}

/// Calculate the state of the time LED.
pub fn update_time_led(old: [bool; 3], tick: u8, dcf77: &DCF77Utils) -> [bool; 3] {
    [set_time_led!(tick, dcf77), old[1], old[2]]
}

/// Calculate the state of the bit LEDs (is-one and error).
pub fn update_bit_leds(old: [bool; 3], tick: u8, dcf77: &DCF77Utils) -> [bool; 3] {
    if tick == 0 {
        [old[0], false, false]
    } else if dcf77.get_current_bit().is_none() {
        [old[0], false, true]
    } else {
        [old[0], dcf77.get_current_bit().unwrap(), false]
    }
}

/// Return the status overview as a compact string
pub fn str_status(dcf77: &DCF77Utils) -> String<14> {
    let mut str_buf: String<14> = String::new();
    write!(
        str_buf,
        "{}{}{}{} {}{}{}",
        frontend::str_jumps(dcf77.get_radio_datetime()),
        frontend::str_parity(dcf77.get_parity_3(), false, 'c'),
        frontend::str_parity(dcf77.get_parity_2(), false, 'b'),
        frontend::str_parity(dcf77.get_parity_1(), false, 'a'),
        str_minute_length(dcf77),
        str_bit_0(dcf77),
        str_bit_20(dcf77),
    )
    .unwrap();
    str_buf
}

/// Return a compact string with miscellaneous information
pub fn str_misc(dcf77: &DCF77Utils) -> String<3> {
    let mut str_buf: String<3> = String::new();
    write!(
        str_buf,
        "{}{}{}",
        str_call_bit(dcf77),
        frontend::str_dst(dcf77.get_radio_datetime()),
        str_leap_second(dcf77)
    )
    .unwrap();
    str_buf
}

/// Return a character representation of the minute length status.
fn str_minute_length(dcf77: &DCF77Utils) -> char {
    match dcf77.get_second().cmp(&dcf77.get_this_minute_length()) {
        Ordering::Less => '<',
        Ordering::Equal => ' ',
        Ordering::Greater => '>',
    }
}

/// Return a character representation of the bit 0 status or 'm' for unknown.
fn str_bit_0(dcf77: &DCF77Utils) -> char {
    let value = dcf77.get_bit_0();
    if value == Some(false) {
        ' '
    } else if value == Some(true) {
        'M'
    } else {
        'm'
    }
}

/// Return a character representation of the call bit status or 'c' for unknown.
fn str_call_bit(dcf77: &DCF77Utils) -> char {
    let value = dcf77.get_call_bit();
    if value == Some(true) {
        'C'
    } else if value == Some(false) {
        ' '
    } else {
        'c'
    }
}

/// Return a character representation of the bit 20 status or 's' for unknown.
fn str_bit_20(dcf77: &DCF77Utils) -> char {
    let value = dcf77.get_bit_20();
    if value == Some(true) {
        ' '
    } else if value == Some(false) {
        'S'
    } else {
        's'
    }
}

/// Returns a character representation of the current leap second status.
fn str_leap_second(dcf77: &DCF77Utils) -> char {
    if let Some(leap_second) = dcf77.get_radio_datetime().get_leap_second() {
        if (leap_second & radio_datetime_utils::LEAP_PROCESSED) != 0 {
            'L'
        } else if dcf77.get_leap_second_is_one() == Some(true) {
            '1'
        } else if (leap_second & radio_datetime_utils::LEAP_MISSING) != 0 {
            'm'
        } else if (leap_second & radio_datetime_utils::LEAP_ANNOUNCED) != 0 {
            'l'
        } else {
            ' '
        }
    } else {
        '*'
    }
}

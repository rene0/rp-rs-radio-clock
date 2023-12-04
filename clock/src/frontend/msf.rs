use crate::{frontend, set_time_led};
use core::cmp::Ordering;
use core::fmt::Write;
use heapless::String;
use msf60_utils::MSFUtils;

/// Put the LEDs in their initial state.
pub fn init_leds() -> [bool; 4] {
    [true, false, false, true]
}

/// Calculate the state of the time LED.
pub fn update_time_led(old: [bool; 4], tick: u8, msf: &MSFUtils) -> [bool; 4] {
    [set_time_led!(tick, msf), old[1], old[2], old[3]]
}

/// Calculate the state of the bit LEDs (is-one and error).
pub fn update_bit_leds(old: [bool; 4], tick: u8, msf: &MSFUtils) -> [bool; 4] {
    if tick == 0 {
        [old[0], false, false, false]
    } else if msf.get_current_bit_a().is_none() || msf.get_current_bit_b().is_none() {
        [old[0], false, false, true]
    } else {
        [
            old[0],
            msf.get_current_bit_a().unwrap(),
            msf.get_current_bit_b().unwrap(),
            false,
        ]
    }
}

/// Return the status overview as a compact string
pub fn str_status(msf: &MSFUtils) -> String<14> {
    let mut str_buf: String<14> = String::new();
    write!(
        str_buf,
        "{}{}{}{}{} {}{}",
        frontend::str_jumps(msf.get_radio_datetime()),
        frontend::str_parity(msf.get_parity_4(), true, 'd'),
        frontend::str_parity(msf.get_parity_3(), true, 'c'),
        frontend::str_parity(msf.get_parity_2(), true, 'b'),
        frontend::str_parity(msf.get_parity_1(), true, 'a'),
        str_minute_length(msf),
        if msf.end_of_minute_marker_present() {
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
    let mut s: String<2> = String::new();
    if value.is_some() {
        write!(s, "{:>-2}", value.unwrap()).unwrap();
    } else {
        write!(s, "**").unwrap();
    }
    s
}

/// Return a compact string with miscellaneous information
pub fn str_misc(msf: &MSFUtils) -> String<3> {
    let mut str_buf: String<3> = String::new();
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
    match (msf.get_second() + 1).cmp(&msf.get_minute_length()) {
        Ordering::Less => '<',
        Ordering::Equal => ' ',
        Ordering::Greater => '>',
    }
}

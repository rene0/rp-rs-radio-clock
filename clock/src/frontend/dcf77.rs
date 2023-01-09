use crate::frontend;
use core::cmp::Ordering;
use core::fmt::Write;
use dcf77_utils::DCF77Utils;
use embedded_hal::digital::v2::OutputPin;
use heapless::String;
use rp_pico::hal::gpio::{bank0, Pin, PushPullOutput};

/// Put the LEDs in their initial state.
pub fn init_leds(
    led_time: &mut Pin<bank0::Gpio12, PushPullOutput>,
    led_bit: &mut Pin<bank0::Gpio13, PushPullOutput>,
    led_error: &mut Pin<bank0::Gpio14, PushPullOutput>,
) {
    led_time.set_high().unwrap();
    led_bit.set_low().unwrap();
    led_error.set_high().unwrap();
}

/// Turn the time LED on or off depending on whether a new second or minute arrived.
pub fn update_time_led(
    tick: u8,
    dcf77: &DCF77Utils,
    led_time: &mut Pin<bank0::Gpio12, PushPullOutput>,
) {
    if tick == 0 {
        led_time.set_high().unwrap();
    } else if (!dcf77.get_new_minute() && tick >= crate::FRAMES_PER_SECOND * 2 / 10)
        || (dcf77.get_new_minute() && tick >= crate::FRAMES_PER_SECOND * 8 / 10)
    {
        led_time.set_low().unwrap();
    }
}

/// Turn the bit LEDs (is-one and error) on or off.
pub fn update_bit_leds(
    tick: u8,
    dcf77: &DCF77Utils,
    led_bit: &mut Pin<bank0::Gpio13, PushPullOutput>,
    led_error: &mut Pin<bank0::Gpio14, PushPullOutput>,
) {
    if tick == 0 {
        led_bit.set_low().unwrap();
        led_error.set_low().unwrap();
    } else if dcf77.get_current_bit().is_none() {
        led_bit.set_low().unwrap();
        led_error.set_high().unwrap();
    } else {
        if dcf77.get_current_bit() == Some(true) {
            led_bit.set_high().unwrap();
        } else {
            led_bit.set_low().unwrap();
        }
        led_error.set_low().unwrap();
    }
}

/// Return the status overview as a compact string
pub fn str_status(dcf77: &DCF77Utils) -> String<14> {
    let mut str_buf = String::<14>::from("");
    write!(
        str_buf,
        "{}{}{}{}{}{}{}{}{}{} {}{}{}",
        str_jump_year(dcf77),
        str_jump_month(dcf77),
        str_jump_day(dcf77),
        str_jump_weekday(dcf77),
        str_jump_hour(dcf77),
        str_jump_minute(dcf77),
        str_jump_dst(dcf77),
        frontend::str_parity(dcf77.get_parity_3(), false, 'c'),
        frontend::str_parity(dcf77.get_parity_2(), false, 'b'),
        frontend::str_parity(dcf77.get_parity_1(), false, 'a'),
        str_bit_0(dcf77),
        str_bit_20(dcf77),
        str_minute_length(dcf77),
    )
    .unwrap();
    str_buf
}

/// Return a compact string with miscellaneous information
pub fn str_misc(dcf77: &DCF77Utils) -> String<3> {
    let mut str_buf = String::<3>::from("");
    write!(
        str_buf,
        "{}{}{}",
        str_call_bit(dcf77),
        str_dst(dcf77),
        str_leap_second(dcf77)
    )
    .unwrap();
    str_buf
}

/// Return if the year has jumped unexpectedly, 'y' or ' '
fn str_jump_year(dcf77: &DCF77Utils) -> char {
    if dcf77.get_radio_datetime().get_jump_year() {
        'y'
    } else {
        ' '
    }
}

/// Return if the month has jumped unexpectedly, 'm' or ' '
fn str_jump_month(dcf77: &DCF77Utils) -> char {
    if dcf77.get_radio_datetime().get_jump_month() {
        'm'
    } else {
        ' '
    }
}

/// Return if the day-of-month has jumped unexpectedly, 'd' or ' '.
fn str_jump_day(dcf77: &DCF77Utils) -> char {
    if dcf77.get_radio_datetime().get_jump_day() {
        'd'
    } else {
        ' '
    }
}

/// Return if the day-of-week has jumped unexpectedly, 'w' or ' '.
fn str_jump_weekday(dcf77: &DCF77Utils) -> char {
    if dcf77.get_radio_datetime().get_jump_weekday() {
        'w'
    } else {
        ' '
    }
}

/// Return if the hour has jumped unexpectedly, 'h' or ' '.
fn str_jump_hour(dcf77: &DCF77Utils) -> char {
    if dcf77.get_radio_datetime().get_jump_hour() {
        'h'
    } else {
        ' '
    }
}

/// Return if the minute has jumped unexpectedly, 'm' or ' '.
fn str_jump_minute(dcf77: &DCF77Utils) -> char {
    if dcf77.get_radio_datetime().get_jump_minute() {
        'm'
    } else {
        ' '
    }
}

/// Return if the daylight saving time status jumped unexpectedly, 't' or ' '.
fn str_jump_dst(dcf77: &DCF77Utils) -> char {
    if dcf77.get_radio_datetime().get_dst().is_some()
        && (dcf77.get_radio_datetime().get_dst().unwrap() & radio_datetime_utils::DST_JUMP) != 0
    {
        't'
    } else {
        ' '
    }
}

/// Returns a character representation of the current DST status.
fn str_dst(dcf77: &DCF77Utils) -> char {
    if let Some(dst) = dcf77.get_radio_datetime().get_dst() {
        frontend::dst_str(dst)
    } else {
        '*'
    }
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

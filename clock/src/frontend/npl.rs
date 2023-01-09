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
        "{}{}{}{}{}{}{}{}{}{}{}{}",
        str_jump_year(npl),
        str_jump_month(npl),
        str_jump_day(npl),
        str_jump_weekday(npl),
        str_jump_hour(npl),
        str_jump_minute(npl),
        str_jump_dst(npl),
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
    write!(str_buf, "{}", str_dst(npl)).unwrap();
    str_buf
}

/// Return if the year has jumped unexpectedly, 'y' or ' '
fn str_jump_year(npl: &NPLUtils) -> char {
    if npl.get_radio_datetime().get_jump_year() {
        'y'
    } else {
        ' '
    }
}

/// Return if the month has jumped unexpectedly, 'm' or ' '
fn str_jump_month(npl: &NPLUtils) -> char {
    if npl.get_radio_datetime().get_jump_month() {
        'm'
    } else {
        ' '
    }
}

/// Return if the day-of-month has jumped unexpectedly, 'd' or ' '.
fn str_jump_day(npl: &NPLUtils) -> char {
    if npl.get_radio_datetime().get_jump_day() {
        'd'
    } else {
        ' '
    }
}

/// Return if the day-of-week has jumped unexpectedly, 'w' or ' '.
fn str_jump_weekday(npl: &NPLUtils) -> char {
    if npl.get_radio_datetime().get_jump_weekday() {
        'w'
    } else {
        ' '
    }
}

/// Return if the hour has jumped unexpectedly, 'h' or ' '.
fn str_jump_hour(npl: &NPLUtils) -> char {
    if npl.get_radio_datetime().get_jump_hour() {
        'h'
    } else {
        ' '
    }
}

/// Return if the minute has jumped unexpectedly, 'm' or ' '.
fn str_jump_minute(npl: &NPLUtils) -> char {
    if npl.get_radio_datetime().get_jump_minute() {
        'm'
    } else {
        ' '
    }
}

/// Return if the daylight saving time status jumped unexpectedly, 't' or ' '.
fn str_jump_dst(npl: &NPLUtils) -> char {
    if npl.get_radio_datetime().get_dst().is_some()
        && (npl.get_radio_datetime().get_dst().unwrap() & radio_datetime_utils::DST_JUMP) != 0
    {
        't'
    } else {
        ' '
    }
}

/// Returns a character representation of the current DST status.
fn str_dst(npl: &NPLUtils) -> char {
    if let Some(dst) = npl.get_radio_datetime().get_dst() {
        frontend::dst_str(dst)
    } else {
        '*'
    }
}

/// Return a character representation of the minute length status.
fn str_minute_length(npl: &NPLUtils) -> char {
    match npl.get_second().cmp(&npl.get_minute_length()) {
        Ordering::Less => '<',
        Ordering::Equal => ' ',
        Ordering::Greater => '>',
    }
}

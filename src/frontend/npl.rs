use crate::frontend::{dst_str, str_02, str_weekday};
use crate::FRAMES_PER_SECOND;
use core::cmp::Ordering as spaceship;
use core::fmt::Write;
use embedded_hal::digital::v2::OutputPin;
use heapless::String;
use npl_utils::NPLUtils;
use rp_pico::hal::gpio;

/// Put the LEDs in their initial state.
pub fn init_leds(
    led_time: &mut gpio::Pin<gpio::bank0::Gpio2, gpio::PushPullOutput>,
    led_bit_a: &mut gpio::Pin<gpio::bank0::Gpio3, gpio::PushPullOutput>,
    led_bit_b: &mut gpio::Pin<gpio::bank0::Gpio4, gpio::PushPullOutput>,
    led_error: &mut gpio::Pin<gpio::bank0::Gpio5, gpio::PushPullOutput>,
) {
    led_time.set_high().unwrap();
    led_bit_a.set_low().unwrap();
    led_bit_b.set_low().unwrap();
    led_error.set_high().unwrap();
}

/// Turn the time led on or off depending on whether a new second or minute arrived.
pub fn update_time_led(
    tick: u8,
    npl: &NPLUtils,
    led_time: &mut gpio::Pin<gpio::bank0::Gpio2, gpio::PushPullOutput>,
) {
    if tick == 0 {
        led_time.set_high().unwrap();
    } else if (!npl.get_new_minute() && tick >= FRAMES_PER_SECOND * 2 / 10)
        || (npl.get_new_minute() && tick >= FRAMES_PER_SECOND * 8 / 10)
    {
        led_time.set_low().unwrap();
    }
}

/// Turn the bit LEDs (is-one and error) on or off.
pub fn update_bit_leds(
    tick: u8,
    npl: &NPLUtils,
    led_bit_a: &mut gpio::Pin<gpio::bank0::Gpio3, gpio::PushPullOutput>,
    led_bit_b: &mut gpio::Pin<gpio::bank0::Gpio4, gpio::PushPullOutput>,
    led_error: &mut gpio::Pin<gpio::bank0::Gpio5, gpio::PushPullOutput>,
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
        str_parity_4(npl),
        str_parity_3(npl),
        str_parity_2(npl),
        str_parity_1(npl),
        str_minute_length(npl),
    )
    .unwrap();
    str_buf
}

/// Return the current date and time as a string
pub fn str_datetime(npl: &NPLUtils) -> String<14> {
    let mut str_buf = String::<14>::from("");
    write!(
        str_buf,
        "{}{}{} {} {}{}",
        str_02(npl.get_radio_datetime().get_year()),
        str_02(npl.get_radio_datetime().get_month()),
        str_02(npl.get_radio_datetime().get_day()),
        str_weekday(npl.get_radio_datetime().get_weekday()),
        str_02(npl.get_radio_datetime().get_hour()),
        str_02(npl.get_radio_datetime().get_minute()),
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
        dst_str(dst)
    } else {
        '*'
    }
}

/// Return a character representation of the minute length status.
fn str_minute_length(npl: &NPLUtils) -> char {
    match npl.get_second().cmp(&npl.get_minute_length()) {
        spaceship::Less => '<',
        spaceship::Equal => ' ',
        spaceship::Greater => '>',
    }
}

/// Get a textual version of the year parity bit, ' ' for OK, 'A' for error, or 'a' for unknown.
fn str_parity_1(npl: &NPLUtils) -> char {
    let value = npl.get_parity_1();
    if value == Some(false) {
        'A'
    } else if value == Some(true) {
        ' '
    } else {
        'a'
    }
}

/// Get a textual version of the month/day parity bit, ' ' for OK, 'B' for error, or 'b' for unknown.
fn str_parity_2(npl: &NPLUtils) -> char {
    let value = npl.get_parity_2();
    if value == Some(false) {
        'B'
    } else if value == Some(true) {
        ' '
    } else {
        'b'
    }
}

/// Get a textual version of the weekday parity bit, ' ' for OK, 'C' for error, or 'c' for unknown.
fn str_parity_3(npl: &NPLUtils) -> char {
    let value = npl.get_parity_3();
    if value == Some(false) {
        'C'
    } else if value == Some(true) {
        ' '
    } else {
        'c'
    }
}

/// Get a textual version of the hour/minute parity bit, ' ' for OK, 'D' for  error, or 'd' for unknown.
fn str_parity_4(npl: &NPLUtils) -> char {
    let value = npl.get_parity_4();
    if value == Some(false) {
        'D'
    } else if value == Some(true) {
        ' '
    } else {
        'm'
    }
}

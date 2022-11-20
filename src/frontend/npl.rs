use crate::FRAMES_PER_SECOND;
use core::cmp::Ordering as spaceship;
use embedded_hal::digital::v2::OutputPin;
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

/// Return if the year has jumped unexpectedly, 'y' or ' '
pub fn str_jump_year(npl: &NPLUtils) -> char {
    if npl.get_radio_datetime().get_jump_year() {
        'y'
    } else {
        ' '
    }
}

/// Return if the month has jumped unexpectedly, 'm' or ' '
pub fn str_jump_month(npl: &NPLUtils) -> char {
    if npl.get_radio_datetime().get_jump_month() {
        'm'
    } else {
        ' '
    }
}

/// Return if the day-of-month has jumped unexpectedly, 'd' or ' '.
pub fn str_jump_day(npl: &NPLUtils) -> char {
    if npl.get_radio_datetime().get_jump_day() {
        'd'
    } else {
        ' '
    }
}

/// Return if the day-of-week has jumped unexpectedly, 'w' or ' '.
pub fn str_jump_weekday(npl: &NPLUtils) -> char {
    if npl.get_radio_datetime().get_jump_weekday() {
        'w'
    } else {
        ' '
    }
}

/// Return if the hour has jumped unexpectedly, 'h' or ' '.
pub fn str_jump_hour(npl: &NPLUtils) -> char {
    if npl.get_radio_datetime().get_jump_hour() {
        'h'
    } else {
        ' '
    }
}

/// Return if the minute has jumped unexpectedly, 'm' or ' '.
pub fn str_jump_minute(npl: &NPLUtils) -> char {
    if npl.get_radio_datetime().get_jump_minute() {
        'm'
    } else {
        ' '
    }
}

/// Return if the daylight saving time status jumped unexpectedly, 't' or ' '.
pub fn str_jump_dst(npl: &NPLUtils) -> char {
    if npl.get_radio_datetime().get_dst().is_some()
        && (npl.get_radio_datetime().get_dst().unwrap() & radio_datetime_utils::DST_JUMP) != 0
    {
        't'
    } else {
        ' '
    }
}

/// Returns a character representation of the current DST status.
pub fn str_dst(npl: &NPLUtils) -> char {
    if let Some(dst) = npl.get_radio_datetime().get_dst() {
        let mut res_dst = if (dst & radio_datetime_utils::DST_SUMMER) != 0 {
            's'
        } else {
            'w'
        };
        if (dst & (radio_datetime_utils::DST_ANNOUNCED | radio_datetime_utils::DST_PROCESSED)) != 0
        {
            res_dst = res_dst.to_ascii_uppercase();
        }
        res_dst
    } else {
        '*'
    }
}

/// Return a character representation of the minute length status.
pub fn str_minute_length(npl: &NPLUtils) -> char {
    match npl.get_second().cmp(&npl.get_minute_length()) {
        spaceship::Less => '<',
        spaceship::Equal => ' ',
        spaceship::Greater => '>',
    }
}

/// Get a textual version of the year parity bit, ' ' for OK, 'A' for error, or 'a' for unknown.
pub fn str_parity_1(npl: &NPLUtils) -> char {
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
pub fn str_parity_2(npl: &NPLUtils) -> char {
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
pub fn str_parity_3(npl: &NPLUtils) -> char {
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
pub fn str_parity_4(npl: &NPLUtils) -> char {
    let value = npl.get_parity_4();
    if value == Some(false) {
        'D'
    } else if value == Some(true) {
        ' '
    } else {
        'm'
    }
}
use dcf77_utils::DCF77Utils;

use core::cmp::Ordering as spaceship;
use embedded_hal::digital::v2::OutputPin;
use rp_pico::hal::gpio;

pub fn update_leds(
    dcf77: &DCF77Utils,
    led_time: &mut gpio::Pin<gpio::pin::bank0::Gpio12, gpio::PushPullOutput>,
    led_bit: &mut gpio::Pin<gpio::pin::bank0::Gpio13, gpio::PushPullOutput>,
    led_error: &mut gpio::Pin<gpio::pin::bank0::Gpio14, gpio::PushPullOutput>,
) {
    if dcf77.get_ind_time() {
        led_time.set_high().unwrap();
    } else {
        led_time.set_low().unwrap();
    }
    if dcf77.get_ind_bit() {
        led_bit.set_high().unwrap();
    } else {
        led_bit.set_low().unwrap();
    }
    if dcf77.get_ind_error() {
        led_error.set_high().unwrap();
    } else {
        led_error.set_low().unwrap();
    }
}

/// Return if the year has jumped unexpectedly, 'y' or ' '
pub fn str_jump_year(dcf77: &DCF77Utils) -> char {
    if dcf77.get_radio_datetime().get_jump_year() {
        'y'
    } else {
        ' '
    }
}

/// Return if the month has jumped unexpectedly, 'm' or ' '
pub fn str_jump_month(dcf77: &DCF77Utils) -> char {
    if dcf77.get_radio_datetime().get_jump_month() {
        'm'
    } else {
        ' '
    }
}

/// Return if the day-of-month has jumped unexpectedly, 'd' or ' '.
pub fn str_jump_day(dcf77: &DCF77Utils) -> char {
    if dcf77.get_radio_datetime().get_jump_day() {
        'd'
    } else {
        ' '
    }
}

/// Return if the day-of-week has jumped unexpectedly, 'w' or ' '.
pub fn str_jump_weekday(dcf77: &DCF77Utils) -> char {
    if dcf77.get_radio_datetime().get_jump_weekday() {
        'w'
    } else {
        ' '
    }
}

/// Return if the hour has jumped unexpectedly, 'h' or ' '.
pub fn str_jump_hour(dcf77: &DCF77Utils) -> char {
    if dcf77.get_radio_datetime().get_jump_hour() {
        'h'
    } else {
        ' '
    }
}

/// Return if the minute has jumped unexpectedly, 'm' or ' '.
pub fn str_jump_minute(dcf77: &DCF77Utils) -> char {
    if dcf77.get_radio_datetime().get_jump_minute() {
        'm'
    } else {
        ' '
    }
}

/// Return if the daylight saving time status jumped unexpectedly, 't' or ' '.
pub fn str_jump_dst(dcf77: &DCF77Utils) -> char {
    if dcf77.get_radio_datetime().get_dst().is_some()
        && (dcf77.get_radio_datetime().get_dst().unwrap() & radio_datetime_utils::DST_JUMP) != 0
    {
        't'
    } else {
        ' '
    }
}

/// Returns a character representation of the current DST status.
pub fn str_dst(dcf77: &DCF77Utils) -> char {
    if let Some(dst) = dcf77.get_radio_datetime().get_dst() {
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
pub fn str_minute_length(dcf77: &DCF77Utils) -> char {
    match dcf77.get_second().cmp(&dcf77.get_minute_length()) {
        spaceship::Less => '<',
        spaceship::Equal => ' ',
        spaceship::Greater => '>',
    }
}

/// Get a textual version of the minute parity bit, ' ' for OK or '1' for error.
pub fn str_parity_1(dcf77: &DCF77Utils) -> char {
    if dcf77.get_parity_1() == Some(false) {
        ' '
    } else {
        '1'
    }
}

/// Get a textual version of the hour parity bit, ' ' for OK or '2' for error.
pub fn str_parity_2(dcf77: &DCF77Utils) -> char {
    if dcf77.get_parity_2() == Some(false) {
        ' '
    } else {
        '2'
    }
}

/// Get a textual version of the date parity bit, ' ' for OK or '3' for error.
pub fn str_parity_3(dcf77: &DCF77Utils) -> char {
    if dcf77.get_parity_3() == Some(false) {
        ' '
    } else {
        '3'
    }
}

/// Return a character representation of the bit 0 status
pub fn str_bit_0(dcf77: &DCF77Utils) -> char {
    if dcf77.get_bit_0() == Some(false) {
        ' '
    } else {
        'M'
    }
}

/// Return a character representation of the call bit status
pub fn str_call_bit(dcf77: &DCF77Utils) -> char {
    if dcf77.get_call_bit() == Some(true) {
        'C'
    } else {
        ' '
    }
}

/// Return a character representation of the bit 20 status
pub fn str_bit_20(dcf77: &DCF77Utils) -> char {
    if dcf77.get_bit_20() == Some(true) {
        ' '
    } else {
        'S'
    }
}

/// Returns a character representation of the current leap second status.
pub fn str_leap_second(dcf77: &DCF77Utils) -> char {
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

use core::fmt::Write;
use heapless::String;
use radio_datetime_utils::RadioDateTimeUtils;

pub mod dcf77;
pub mod msf;

#[macro_export]
macro_rules! set_time_led {
    ($tick:expr, $station:ident) => {
        (!$station.get_new_minute() && $tick < $crate::FRAMES_PER_SECOND * 2 / 10)
            || ($station.get_new_minute() && $tick < $crate::FRAMES_PER_SECOND * 6 / 10)
    };
}

/// Return a string version of the given value with leading 0, truncated to two digits or ** for None.
pub fn str_02(value: Option<u8>) -> String<2> {
    let mut s: String<2> = String::new();
    if value.is_some() {
        write!(s, "{:>02}", value.unwrap()).unwrap();
    } else {
        write!(s, "**").unwrap();
    }
    s
}

/// Return a textual representation of the weekday, Mo-Su or ** for None.
fn str_weekday(weekday: Option<u8>, sunday: u8) -> String<2> {
    let mut s: String<2> = String::new();
    write!(
        s,
        "{}",
        match weekday {
            Some(0) | Some(7) if weekday == Some(sunday) => "Su",
            Some(1) => "Mo",
            Some(2) => "Tu",
            Some(3) => "We",
            Some(4) => "Th",
            Some(5) => "Fr",
            Some(6) => "Sa",
            _ => "**",
        }
    )
    .unwrap();
    s
}

/// Return the given date and time as a string
pub fn str_datetime(rdt: RadioDateTimeUtils, sunday: u8) -> String<14> {
    let mut str_buf: String<14> = String::new();
    write!(
        str_buf,
        "{}{}{} {} {}{}",
        str_02(rdt.get_year()),
        str_02(rdt.get_month()),
        str_02(rdt.get_day()),
        str_weekday(rdt.get_weekday(), sunday),
        str_02(rdt.get_hour()),
        str_02(rdt.get_minute()),
    )
    .unwrap();
    str_buf
}

/// Get a textual version of a parity bit, ' ' for OK, 'X' for error, or 'x' for unknown for some 'x'.
fn str_parity(parity: Option<bool>, ok: bool, name: char) -> char {
    if parity == Some(ok) {
        ' '
    } else if parity == Some(!ok) {
        name.to_ascii_uppercase()
    } else {
        name
    }
}

/// Get a textual version of any unexpected jumps in the current date and time.
fn str_jumps(rdt: RadioDateTimeUtils) -> String<7> {
    let mut str_buf: String<7> = String::new();
    write!(
        str_buf,
        "{}{}{}{}{}{}{}",
        str_jump_year(rdt),
        str_jump_month(rdt),
        str_jump_day(rdt),
        str_jump_weekday(rdt),
        str_jump_hour(rdt),
        str_jump_minute(rdt),
        str_jump_dst(rdt)
    )
    .unwrap();
    str_buf
}

/// Returns a character representation of the current DST status.
fn str_dst(rdt: RadioDateTimeUtils) -> char {
    if let Some(dst) = rdt.get_dst() {
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

/// Return if the year has jumped unexpectedly, 'y' or ' '
fn str_jump_year(rdt: RadioDateTimeUtils) -> char {
    if rdt.get_jump_year() {
        'y'
    } else {
        ' '
    }
}

/// Return if the month has jumped unexpectedly, 'm' or ' '
fn str_jump_month(rdt: RadioDateTimeUtils) -> char {
    if rdt.get_jump_month() {
        'm'
    } else {
        ' '
    }
}

/// Return if the day-of-month has jumped unexpectedly, 'd' or ' '.
fn str_jump_day(rdt: RadioDateTimeUtils) -> char {
    if rdt.get_jump_day() {
        'd'
    } else {
        ' '
    }
}

/// Return if the day-of-week has jumped unexpectedly, 'w' or ' '.
fn str_jump_weekday(rdt: RadioDateTimeUtils) -> char {
    if rdt.get_jump_weekday() {
        'w'
    } else {
        ' '
    }
}

/// Return if the hour has jumped unexpectedly, 'h' or ' '.
fn str_jump_hour(rdt: RadioDateTimeUtils) -> char {
    if rdt.get_jump_hour() {
        'h'
    } else {
        ' '
    }
}

/// Return if the minute has jumped unexpectedly, 'm' or ' '.
fn str_jump_minute(rdt: RadioDateTimeUtils) -> char {
    if rdt.get_jump_minute() {
        'm'
    } else {
        ' '
    }
}

/// Return if the daylight saving time status jumped unexpectedly, 't' or ' '.
fn str_jump_dst(rdt: RadioDateTimeUtils) -> char {
    if rdt.get_dst().is_some() && (rdt.get_dst().unwrap() & radio_datetime_utils::DST_JUMP) != 0 {
        't'
    } else {
        ' '
    }
}

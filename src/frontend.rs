use core::fmt::Write;
use heapless::String;

pub mod dcf77;
pub mod npl;

/// Return a character representation of the given DST state.
pub fn dst_str(dst: u8) -> char {
    let mut res_dst = if (dst & radio_datetime_utils::DST_SUMMER) != 0 {
        's'
    } else {
        'w'
    };
    if (dst & (radio_datetime_utils::DST_ANNOUNCED | radio_datetime_utils::DST_PROCESSED)) != 0 {
        res_dst = res_dst.to_ascii_uppercase();
    }
    res_dst
}

/// Return a string version of the given value with leading 0, truncated to two digits or ** for None.
pub fn str_02(value: Option<u8>) -> String<2> {
    let mut s = String::<2>::from("");
    if value.is_some() {
        write!(s, "{:>02}", value.unwrap()).unwrap();
    } else {
        write!(s, "**").unwrap();
    }
    s
}

/// Return a textual representation of the weekday, Mo-Su or ** for None.
pub fn str_weekday(weekday: Option<u8>) -> String<2> {
    String::<2>::from(match weekday {
        Some(0) | Some(7) => "Su",
        Some(1) => "Mo",
        Some(2) => "Tu",
        Some(3) => "We",
        Some(4) => "Th",
        Some(5) => "Fr",
        Some(6) => "Sa",
        _ => "**",
    })
}


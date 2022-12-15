use core::fmt::Write;
use heapless::String;

pub mod dcf77;
pub mod npl;

/// Number of rows on the display, change as needed
const DISPLAY_ROWS: u8 = 4;
/// Number of columns on the display, change as needed
const DISPLAY_COLUMNS: u8 = 20;

/// Gets the one-dimensional HD44780 coordinate for position (x, y) (zero-based)
///
/// See <https://web.alfredstate.edu/faculty/weimandn/lcd/lcd_addressing/lcd_addressing_index.html>
///
/// Assumes type-2 addressing for 16x1 displays
///
/// Temporary until HD44780 releases an update.
pub fn get_xy(x: u8, y: u8) -> Option<u8> {
    if (x >= DISPLAY_COLUMNS) || (y >= DISPLAY_ROWS) {
        panic!(
            "Coordinates ({},{}) out of bounds ({},{})",
            x, y, DISPLAY_COLUMNS, DISPLAY_ROWS
        );
    }
    let mut addr = x & 0x3f;
    if (y & 1) == 1 {
        addr += 0x40;
    }
    if (y & 2) == 2 {
        addr += DISPLAY_COLUMNS;
    }
    Some(addr)
}

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

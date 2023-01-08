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

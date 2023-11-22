use embedded_hal::blocking::delay::{DelayMs, DelayUs};
use hd44780_driver::bus::I2CBus;
use hd44780_driver::HD44780;
use rp_pico::hal::{
    gpio::{bank0, FunctionI2C, Pin, PullDown},
    I2C,
};
use rp_pico::pac::I2C1;

/// Number of rows on the display, change as needed
const DISPLAY_ROWS: u8 = 4;
/// Number of columns on the display, change as needed
const DISPLAY_COLUMNS: u8 = 20;

type I2CDisplay = I2CBus<
    I2C<
        I2C1,
        (
            Pin<bank0::Gpio26, FunctionI2C, PullDown>,
            Pin<bank0::Gpio27, FunctionI2C, PullDown>,
        ),
    >,
>;

/// Gets the one-dimensional HD44780 coordinate for position (x, y) (zero-based)
///
/// See <https://web.alfredstate.edu/faculty/weimandn/lcd/lcd_addressing/lcd_addressing_index.html>
///
/// Assumes type-2 addressing for 16x1 displays
///
/// Temporary until HD44780 releases an update.
fn get_xy(x: u8, y: u8) -> Option<u8> {
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

#[inline] // spend 2kB in the uf2 file on speed
pub fn write_at<D: DelayUs<u16> + DelayMs<u8>>(
    pos: (u8, u8),
    m: &str,
    lcd: &mut HD44780<I2CDisplay>,
    delay: &mut D,
) {
    lcd.set_cursor_pos(get_xy(pos.0, pos.1).unwrap(), delay)
        .unwrap();
    lcd.write_str(m, delay).unwrap();
}

#![no_std]

use core::fmt::Error;

pub struct Hd44780Wrapper {
    number_columns: u8,
    number_rows: u8,
}
impl Hd44780Wrapper {
    pub fn new(columns:u8, rows:u8) -> Self {
        Self { number_columns: columns, number_rows: rows }
    }

    /// Gets the one-dimensional HD44780 coordinate for position (x, y) (zero-based)
    ///
    /// https://web.alfredstate.edu/faculty/weimandn/lcd/lcd_addressing/lcd_addressing_index.html
    /// Assumes type-2 addressing for 16x1 displays
    pub fn get_xy(&self, x: u8, y: u8) -> Result<u8, Error> {
        if (x > self.number_columns) || (y > self.number_rows) {
            panic!("Coordinates out of bounds")
        }
        let mut addr = x & 0x3f;
        if (y & 1) == 1 {
            addr += 0x40;
        }
        if (y & 2) == 2 {
            addr += self.number_columns;
        }
        Ok(addr)
    }
}

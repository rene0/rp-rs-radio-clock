#![no_std]
#![no_main]

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};
use bsp::XOSC_CRYSTAL_FREQ;
use core::fmt::Error;
use core::fmt::Write;
use cortex_m_rt::entry;
use defmt_rtt as _;
use embedded_time::fixed_point::FixedPoint;
use embedded_time::rate::Extensions;
use hd44780_driver::{Cursor, CursorBlink, HD44780};
use heapless::String;
use panic_halt as _;
use rp_pico as bsp;

/// I²C address of the PCF8574 adapter, change as needed
const I2C_ADDRESS: u8 = 0x27;
/// Number of rows on the display, change as needed
const DISPLAY_ROWS: u8 = 4;
/// Number of columns on the display, change as needed
const DISPLAY_COLUMNS: u8 = 20;

/// Gets the one-dimensional HD44780 coordinate for position (x, y) (zero-based)
///
/// https://web.alfredstate.edu/faculty/weimandn/lcd/lcd_addressing/lcd_addressing_index.html
/// Assumes type-2 addressing for 16x1 displays
fn get_xy(x: u8, y: u8) -> Result<u8, Error> {
    if (x > DISPLAY_COLUMNS) || (y > DISPLAY_ROWS) {
        panic!("Coordinates out of bounds")
    }
    let mut addr = x & 0x3f;
    if (y & 1) == 1 {
        addr += 0x40;
    }
    if (y & 2) == 2 {
        addr += DISPLAY_COLUMNS;
    }
    Ok(addr)
}

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);
    // boilerplate from the rp2040 template:
    let clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());
    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Create the I²C drive, using the two pre-configured pins. This will fail
    // at compile time if the pins are in the wrong mode, or if this I²C
    // peripheral isn't available on these pins!
    let sda_pin = pins.gpio0.into_mode::<bsp::hal::gpio::FunctionI2C>();
    let scl_pin = pins.gpio1.into_mode::<bsp::hal::gpio::FunctionI2C>();

    let i2c = bsp::hal::I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        clocks.peripheral_clock,
    );
    let mut lcd = HD44780::new_i2c(i2c, I2C_ADDRESS, &mut delay).unwrap();

    lcd.reset(&mut delay).unwrap();
    lcd.clear(&mut delay).unwrap();
    // Turn the cursor off, in two steps:
    lcd.set_cursor_blink(CursorBlink::Off, &mut delay).unwrap(); // small static cursor
    lcd.set_cursor_visibility(Cursor::Invisible, &mut delay)
        .unwrap(); // turn off completely

    // Write to the top line
    lcd.write_str("rp-hal on", &mut delay).unwrap();

    // Move the cursor
    lcd.set_cursor_pos(get_xy(8, 2).unwrap(), &mut delay)
        .unwrap();

    // Write more more text
    lcd.write_str("HD44780!", &mut delay).unwrap();

    lcd.set_cursor_pos(get_xy(11, 3).unwrap(), &mut delay)
        .unwrap();
    lcd.write_str("at (11,3)", &mut delay).unwrap();

    let timer = bsp::hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut old_value = timer.get_counter();
    loop {
        delay.delay_ms(960);
        let new_value = timer.get_counter();
        let dist = new_value - old_value;
        let mut data = String::<20>::from("");

        // `write` for `heapless::String` returns an error if the buffer is full,
        // but because the buffer here is 20 bytes large, the u64 will fit.
        let _ = write!(data, "{}", dist);
        lcd.set_cursor_pos(get_xy(0, 1).unwrap(), &mut delay)
            .unwrap();
        lcd.write_str(data.as_str(), &mut delay).unwrap();
        old_value = new_value;
    }
}

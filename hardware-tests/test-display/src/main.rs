#![no_std]
#![no_main]

use core::fmt::Write;
use cortex_m::delay::Delay;
use fugit::RateExtU32;
use hd44780_driver::{Cursor, CursorBlink, HD44780};
use heapless::String;
use rp_pico::hal::{
    clocks, clocks::Clock, gpio::FunctionI2C, sio::Sio, watchdog::Watchdog, Timer, I2C,
};
use rp_pico::pac::{CorePeripherals, Peripherals};
use rp_pico::Pins;

extern crate panic_halt;
mod hd44780_helper;

/// I²C address of the PCF8574 adapter, change as needed
const I2C_ADDRESS: u8 = 0x27;

#[rp_pico::entry]
fn main() -> ! {
    let mut pac = Peripherals::take().unwrap();
    let core = CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);
    // boilerplate from the rp2040 template:
    let clocks = clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Create the I²C drive, using the two pre-configured pins. This will fail
    // at compile time if the pins are in the wrong mode, or if this I²C
    // peripheral isn't available on these pins!
    let sda_pin = pins.gpio26.into_function::<FunctionI2C>();
    let scl_pin = pins.gpio27.into_function::<FunctionI2C>();

    let i2c = I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.peripheral_clock,
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
    lcd.set_cursor_pos(hd44780_helper::get_xy(8, 2).unwrap(), &mut delay)
        .unwrap();

    // Write more more text
    lcd.write_str("HD44780!", &mut delay).unwrap();

    lcd.set_cursor_pos(hd44780_helper::get_xy(11, 3).unwrap(), &mut delay)
        .unwrap();
    lcd.write_str("at (11,3)", &mut delay).unwrap();

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let mut old_value = timer.get_counter();
    loop {
        delay.delay_ms(960);
        let new_value = timer.get_counter();
        let dist = new_value - old_value;
        let mut data: String<20> = String::new();

        // `write` for `heapless::String` returns an error if the buffer is full,
        // but because the buffer here is 20 bytes large, the u64 will fit.
        let _ = write!(data, "{dist}");
        lcd.set_cursor_pos(hd44780_helper::get_xy(0, 1).unwrap(), &mut delay)
            .unwrap();
        lcd.write_str(data.as_str(), &mut delay).unwrap();
        old_value = new_value;
    }
}

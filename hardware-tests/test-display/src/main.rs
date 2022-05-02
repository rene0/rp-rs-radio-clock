#![no_std]
#![no_main]

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};
use bsp::XOSC_CRYSTAL_FREQ;
use cortex_m_rt::entry;
use defmt_rtt as _;
use embedded_time::fixed_point::FixedPoint;
use embedded_time::rate::Extensions;
use hd44780_driver::HD44780;
use panic_halt as _;
use rp_pico as bsp;

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
    let mut lcd = HD44780::new_i2c(i2c, 0x27, &mut delay).unwrap();

    lcd.reset(&mut delay).unwrap();
    lcd.clear(&mut delay).unwrap();
    // Write to the top line
    lcd.write_str("rp-hal on", &mut delay).unwrap();

    // Move the cursor
    lcd.set_cursor_pos(40, &mut delay).unwrap();

    // Write more more text
    lcd.write_str("HD44780!", &mut delay).unwrap();

    // Do nothing - we're finished
    #[allow(clippy::empty_loop)]
    loop {
        // Empty loop
    }
}

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt_rtt as _;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use panic_halt as _;
use rp_pico as bsp;
use bsp::hal::{
    pac,
    sio::Sio,
};

fn test_ky040(pins: bsp::Pins) -> ! {
    let button_pin = pins.gpio7.into_pull_down_input(); // KY-040 SW push button, perhaps into_floating_input() works too
    let mut led_pin = pins.led.into_push_pull_output();
    // test "write to I/O-port" and pushing a button --> OK
    loop {
        if button_pin.is_low().unwrap() {
            led_pin.set_high().unwrap();
        } else {
            led_pin.set_low().unwrap();
        }
    }
}

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let sio = Sio::new(pac.SIO);

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    test_ky040(pins);
}

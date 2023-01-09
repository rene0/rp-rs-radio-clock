#![no_std]
#![no_main]

use cortex_m::delay::Delay;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use panic_halt as _;
use rp_pico::hal::{clocks, clocks::Clock, sio::Sio, watchdog::Watchdog};
use rp_pico::pac::{CorePeripherals, Peripherals};
use rp_pico::Pins;

fn test_leds(pins: Pins, mut delay: Delay) -> ! {
    // no vec! in no_std, and each pin is a different type so using an array does not work.
    let mut dcf77_led_time = pins.gpio12.into_push_pull_output();
    let mut dcf77_led_bit = pins.gpio13.into_push_pull_output();
    let mut dcf77_led_error = pins.gpio14.into_push_pull_output();
    let mut npl_led_time = pins.gpio2.into_push_pull_output();
    let mut npl_led_bit_a = pins.gpio3.into_push_pull_output();
    let mut npl_led_bit_b = pins.gpio4.into_push_pull_output();
    let mut npl_led_error = pins.gpio5.into_push_pull_output();
    // test "write to I/O-port" --> OK
    loop {
        dcf77_led_time.set_low().unwrap();
        dcf77_led_bit.set_low().unwrap();
        dcf77_led_error.set_low().unwrap();
        npl_led_time.set_high().unwrap();
        npl_led_bit_a.set_high().unwrap();
        npl_led_bit_b.set_high().unwrap();
        npl_led_error.set_high().unwrap();
        delay.delay_ms(300);
        dcf77_led_time.set_high().unwrap();
        dcf77_led_bit.set_high().unwrap();
        dcf77_led_error.set_high().unwrap();
        npl_led_time.set_low().unwrap();
        npl_led_bit_a.set_low().unwrap();
        npl_led_bit_b.set_low().unwrap();
        npl_led_error.set_low().unwrap();
        delay.delay_ms(300);
    }
}

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

    let delay = Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    test_leds(pins, delay);
}

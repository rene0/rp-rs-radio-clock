use crate::FRAMES_PER_SECOND;
use embedded_hal::digital::v2::OutputPin;
use npl_utils::NPLUtils;
use rp_pico::hal::gpio;

/// Put the LEDs in their initial state.
pub fn init_leds(
    led_time: &mut gpio::Pin<gpio::bank0::Gpio2, gpio::PushPullOutput>,
    led_bit_a: &mut gpio::Pin<gpio::bank0::Gpio3, gpio::PushPullOutput>,
    led_bit_b: &mut gpio::Pin<gpio::bank0::Gpio4, gpio::PushPullOutput>,
    led_error: &mut gpio::Pin<gpio::bank0::Gpio5, gpio::PushPullOutput>,
) {
    led_time.set_high().unwrap();
    led_bit_a.set_low().unwrap();
    led_bit_b.set_low().unwrap();
    led_error.set_high().unwrap();
}

/// Turn the time led on or off depending on whether a new second or minute arrived.
pub fn update_time_led(
    tick: u8,
    npl: &NPLUtils,
    led_time: &mut gpio::Pin<gpio::bank0::Gpio2, gpio::PushPullOutput>,
) {
    if tick == 0 {
        led_time.set_high().unwrap();
    } else if (!npl.get_new_minute() && tick >= FRAMES_PER_SECOND * 2 / 10)
        || (npl.get_new_minute() && tick >= FRAMES_PER_SECOND * 8 / 10)
    {
        led_time.set_low().unwrap();
    }
}

/// Turn the bit LEDs (is-one and error) on or off.
pub fn update_bit_leds(
    tick: u8,
    npl: &NPLUtils,
    led_bit_a: &mut gpio::Pin<gpio::bank0::Gpio3, gpio::PushPullOutput>,
    led_bit_b: &mut gpio::Pin<gpio::bank0::Gpio4, gpio::PushPullOutput>,
    led_error: &mut gpio::Pin<gpio::bank0::Gpio5, gpio::PushPullOutput>,
) {
    if tick == 0 {
        led_bit_a.set_low().unwrap();
        led_bit_b.set_low().unwrap();
        led_error.set_low().unwrap();
    } else if npl.get_current_bit_a().is_none() || npl.get_current_bit_b().is_none() {
        led_bit_a.set_low().unwrap();
        led_bit_b.set_low().unwrap();
        led_error.set_high().unwrap();
    } else {
        if npl.get_current_bit_a() == Some(true) {
            led_bit_a.set_high().unwrap();
        } else {
            led_bit_a.set_low().unwrap();
        }
        if npl.get_current_bit_b() == Some(true) {
            led_bit_b.set_high().unwrap();
        } else {
            led_bit_b.set_low().unwrap();
        }
        led_error.set_low().unwrap();
    }
}

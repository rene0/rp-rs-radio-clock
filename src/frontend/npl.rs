use embedded_hal::digital::v2::OutputPin;
use npl_utils::NPLUtils;
use rp_pico::hal::gpio;

pub fn update_leds(
    npl: &NPLUtils,
    led_time: &mut gpio::Pin<gpio::pin::bank0::Gpio2, gpio::PushPullOutput>,
    led_bit_a: &mut gpio::Pin<gpio::pin::bank0::Gpio3, gpio::PushPullOutput>,
    led_bit_b: &mut gpio::Pin<gpio::pin::bank0::Gpio4, gpio::PushPullOutput>,
    led_error: &mut gpio::Pin<gpio::pin::bank0::Gpio5, gpio::PushPullOutput>,
) {
    if npl.get_ind_time() {
        led_time.set_high().unwrap();
    } else {
        led_time.set_low().unwrap();
    }
    if npl.get_ind_bit_a() {
        led_bit_a.set_high().unwrap();
    } else {
        led_bit_a.set_low().unwrap();
    }
    if npl.get_ind_bit_b() {
        led_bit_b.set_high().unwrap();
    } else {
        led_bit_b.set_low().unwrap();
    }
    if npl.get_ind_error() {
        led_error.set_high().unwrap();
    } else {
        led_error.set_low().unwrap();
    }
}

use core::sync::atomic::{AtomicBool, AtomicU32};
use embedded_hal::digital::v2::OutputPin;
use hd44780_driver::bus::I2CBus;
use rp_pico::hal::{
    gpio::{bank0, FunctionI2C, FunctionSioOutput, Pin, PullDown},
    I2C,
};
use rp_pico::pac::I2C1;

// was_low is to prevent ghost edges
pub struct HardwareEdge {
    pub(crate) is_new: AtomicBool,
    pub(crate) is_low: AtomicBool,
    pub(crate) was_low: AtomicBool,
    pub(crate) when: AtomicU32,
}

impl HardwareEdge {
    pub const fn new() -> Self {
        Self {
            is_new: AtomicBool::new(false),
            is_low: AtomicBool::new(false),
            was_low: AtomicBool::new(false),
            when: AtomicU32::new(0),
        }
    }
}

pub type I2CDisplay = I2CBus<
    I2C<
        I2C1,
        (
            Pin<bank0::Gpio26, FunctionI2C, PullDown>,
            Pin<bank0::Gpio27, FunctionI2C, PullDown>,
        ),
    >,
>;

#[macro_export]
macro_rules! init_pin {
    ($pin_type:expr, $mgv:ident) => {
        let pin_inst = $pin_type;
        pin_inst.set_interrupt_enabled(gpio::Interrupt::EdgeHigh, true);
        pin_inst.set_interrupt_enabled(gpio::Interrupt::EdgeLow, true);
        unsafe {
            $mgv = Some(pin_inst);
        }
    };
}

pub fn set_leds_dcf77(
    led_time: &mut Pin<bank0::Gpio12, FunctionSioOutput, PullDown>,
    led_bit: &mut Pin<bank0::Gpio13, FunctionSioOutput, PullDown>,
    led_error: &mut Pin<bank0::Gpio14, FunctionSioOutput, PullDown>,
    state: [bool; 3],
) {
    if state[0] {
        led_time.set_high().unwrap();
    } else {
        led_time.set_low().unwrap();
    }
    if state[1] {
        led_bit.set_high().unwrap();
    } else {
        led_bit.set_low().unwrap();
    }
    if state[2] {
        led_error.set_high().unwrap();
    } else {
        led_error.set_low().unwrap();
    }
}

pub fn set_leds_msf(
    led_time: &mut Pin<bank0::Gpio2, FunctionSioOutput, PullDown>,
    led_bit_a: &mut Pin<bank0::Gpio3, FunctionSioOutput, PullDown>,
    led_bit_b: &mut Pin<bank0::Gpio4, FunctionSioOutput, PullDown>,
    led_error: &mut Pin<bank0::Gpio5, FunctionSioOutput, PullDown>,
    state: [bool; 4],
) {
    if state[0] {
        led_time.set_high().unwrap();
    } else {
        led_time.set_low().unwrap();
    }
    if state[1] {
        led_bit_a.set_high().unwrap();
    } else {
        led_bit_a.set_low().unwrap();
    }
    if state[2] {
        led_bit_b.set_high().unwrap();
    } else {
        led_bit_b.set_low().unwrap();
    }
    if state[3] {
        led_error.set_high().unwrap();
    } else {
        led_error.set_low().unwrap();
    }
}

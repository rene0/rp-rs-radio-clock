use core::sync::atomic::{AtomicBool, AtomicU32};
use hd44780_driver::bus::I2CBus;
use rp_pico::hal::{
    gpio::{bank0, FunctionI2C, Pin, PullDown},
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

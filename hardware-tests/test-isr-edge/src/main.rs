#![no_std]
#![no_main]

use core::sync::atomic::{AtomicBool, Ordering};
use defmt_rtt as _; // otherwise "linking with `flip-link`" fails
use embedded_hal::digital::v2::{InputPin, OutputPin};
use panic_halt as _;
use rp_pico::hal::{
    gpio,
    gpio::{bank0, Pin, PullDownInput},
    sio::Sio,
};
use rp_pico::pac;
use rp_pico::pac::{interrupt, Peripherals, NVIC};
use rp_pico::Pins;

type DCF77SignalPin = Pin<bank0::Gpio11, PullDownInput>;
type NPLSignalPin = Pin<bank0::Gpio6, PullDownInput>;

// Needed to transfer our pins into the ISR:
static mut GLOBAL_PIN_DCF77: Option<DCF77SignalPin> = None;
static mut GLOBAL_PIN_NPL: Option<NPLSignalPin> = None;

struct ClockHardware {
    high_edge_received: AtomicBool,
    low_edge_received: AtomicBool,
}

impl ClockHardware {
    pub const fn new() -> Self {
        Self {
            high_edge_received: AtomicBool::new(false),
            low_edge_received: AtomicBool::new(false),
        }
    }
}

static HW_DCF77: ClockHardware = ClockHardware::new();
static HW_NPL: ClockHardware = ClockHardware::new();

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
#[rp_pico::entry]
fn main() -> ! {
    let mut pac = Peripherals::take().unwrap();
    let sio = Sio::new(pac.SIO);
    // boilerplate from the rp2040 template:
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Set power-down pins to LOW, i.e. receiver ON:
    let mut dcf77_pdn = pins.gpio15.into_push_pull_output();
    dcf77_pdn.set_low().unwrap();
    let mut npl_pdn = pins.gpio10.into_push_pull_output();
    npl_pdn.set_low().unwrap();

    // set AGC pins to HIGH, i.e. AGC ON:
    let mut dcf77_aon = pins.gpio26.into_push_pull_output();
    dcf77_aon.set_high().unwrap();
    let mut npl_aon = pins.gpio27.into_push_pull_output();
    npl_aon.set_high().unwrap();

    let mut dcf77_led_bit = pins.gpio13.into_push_pull_output();
    dcf77_led_bit.set_low().unwrap();
    let dcf77_signal_pin = pins.gpio11.into_mode();
    dcf77_signal_pin.set_interrupt_enabled(gpio::Interrupt::EdgeHigh, true);
    dcf77_signal_pin.set_interrupt_enabled(gpio::Interrupt::EdgeLow, true);

    let mut npl_led_bit_a = pins.gpio3.into_push_pull_output();
    npl_led_bit_a.set_low().unwrap();
    let npl_signal_pin = pins.gpio6.into_mode();
    npl_signal_pin.set_interrupt_enabled(gpio::Interrupt::EdgeHigh, true);
    npl_signal_pin.set_interrupt_enabled(gpio::Interrupt::EdgeLow, true);

    // Give our pins away to the ISR and enable the ISR
    unsafe {
        GLOBAL_PIN_DCF77 = Some(dcf77_signal_pin);
        GLOBAL_PIN_NPL = Some(npl_signal_pin);
        NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
    }

    loop {
        if HW_DCF77.low_edge_received.load(Ordering::Acquire) {
            dcf77_led_bit.set_low().unwrap();
            HW_DCF77.low_edge_received.store(false, Ordering::Release);
        }
        if HW_DCF77.high_edge_received.load(Ordering::Acquire) {
            dcf77_led_bit.set_high().unwrap();
            HW_DCF77.high_edge_received.store(false, Ordering::Release);
        }

        if HW_NPL.low_edge_received.load(Ordering::Acquire) {
            npl_led_bit_a.set_low().unwrap();
            HW_NPL.low_edge_received.store(false, Ordering::Release);
        }
        if HW_NPL.high_edge_received.load(Ordering::Acquire) {
            npl_led_bit_a.set_high().unwrap();
            HW_NPL.high_edge_received.store(false, Ordering::Release);
        }
    }
}

macro_rules! handle_tick {
    // Our interrupts don't clear themselves.
    // Do that at the end of each condition, so we don't immediately jump back to this interrupt handler.
    ($pin:ident, $hw:ident) => {
        let s_pin = $pin.as_mut().unwrap();
        if s_pin.is_low().unwrap() {
            $hw.low_edge_received.store(true, Ordering::Release);
            s_pin.clear_interrupt(gpio::Interrupt::EdgeLow);
        } else {
            $hw.high_edge_received.store(true, Ordering::Release);
            s_pin.clear_interrupt(gpio::Interrupt::EdgeHigh);
        }
    };
}

#[allow(non_snake_case)]
#[interrupt]
unsafe fn IO_IRQ_BANK0() {
    handle_tick!(GLOBAL_PIN_DCF77, HW_DCF77);
    handle_tick!(GLOBAL_PIN_NPL, HW_NPL);
}

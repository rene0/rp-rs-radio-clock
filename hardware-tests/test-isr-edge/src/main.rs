#![no_std]
#![no_main]

use bsp::hal::{pac, sio::Sio};
use bsp::pac::interrupt;
use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};
use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry; // the macro for our start-up function
use defmt_rtt as _; // otherwise "linking with `flip-link`" fails
use embedded_hal::digital::v2::{InputPin, OutputPin};
use panic_halt as _;
use rp_pico as bsp;
use rp_pico::hal::gpio;
use rp_pico::hal::gpio::Interrupt::{EdgeHigh, EdgeLow};

type DCF77SignalPin = gpio::Pin<gpio::bank0::Gpio11, gpio::PullDownInput>;
type NPLSignalPin = gpio::Pin<gpio::bank0::Gpio6, gpio::PullDownInput>;

// Needed to transfer our pins into the ISR:
static GLOBAL_PINS_DCF77: Mutex<RefCell<Option<DCF77SignalPin>>> = Mutex::new(RefCell::new(None));
static GLOBAL_PINS_NPL: Mutex<RefCell<Option<NPLSignalPin>>> = Mutex::new(RefCell::new(None));

static G_HIGH_EDGE_RECEIVED_DCF77: AtomicBool = AtomicBool::new(false);
static G_LOW_EDGE_RECEIVED_DCF77: AtomicBool = AtomicBool::new(false);
static G_HIGH_EDGE_RECEIVED_NPL: AtomicBool = AtomicBool::new(false);
static G_LOW_EDGE_RECEIVED_NPL: AtomicBool = AtomicBool::new(false);

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let sio = Sio::new(pac.SIO);
    // boilerplate from the rp2040 template:
    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut dcf77_led_bit = pins.gpio13.into_push_pull_output();
    dcf77_led_bit.set_low().unwrap();
    let dcf77_signal_pin = pins.gpio11.into_mode();
    dcf77_signal_pin.set_interrupt_enabled(EdgeHigh, true);
    dcf77_signal_pin.set_interrupt_enabled(EdgeLow, true);

    let mut npl_led_bit_a = pins.gpio3.into_push_pull_output();
    npl_led_bit_a.set_low().unwrap();
    let npl_signal_pin = pins.gpio6.into_mode();
    npl_signal_pin.set_interrupt_enabled(EdgeHigh, true);
    npl_signal_pin.set_interrupt_enabled(EdgeLow, true);

    // Give our pins away to the ISR
    cortex_m::interrupt::free(|cs| GLOBAL_PINS_DCF77.borrow(cs).replace(Some(dcf77_signal_pin)));
    cortex_m::interrupt::free(|cs| GLOBAL_PINS_NPL.borrow(cs).replace(Some(npl_signal_pin)));
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
    }

    loop {
        if G_LOW_EDGE_RECEIVED_DCF77.load(Ordering::Acquire) {
            dcf77_led_bit.set_low().unwrap();
            G_LOW_EDGE_RECEIVED_DCF77.store(false, Ordering::Release);
        }
        if G_HIGH_EDGE_RECEIVED_DCF77.load(Ordering::Acquire) {
            dcf77_led_bit.set_high().unwrap();
            G_HIGH_EDGE_RECEIVED_DCF77.store(false, Ordering::Release);
        }

        if G_LOW_EDGE_RECEIVED_NPL.load(Ordering::Acquire) {
            npl_led_bit_a.set_low().unwrap();
            G_LOW_EDGE_RECEIVED_NPL.store(false, Ordering::Release);
        }
        if G_HIGH_EDGE_RECEIVED_NPL.load(Ordering::Acquire) {
            npl_led_bit_a.set_high().unwrap();
            G_HIGH_EDGE_RECEIVED_NPL.store(false, Ordering::Release);
        }
    }
}

#[interrupt]
fn IO_IRQ_BANK0() {
    static mut DCF77_SIGNAL_PIN: Option<DCF77SignalPin> = None;
    static mut NPL_SIGNAL_PIN: Option<NPLSignalPin> = None;

    // This is one-time lazy initialisation. We steal the variables given to us via `GLOBAL_PINS`.
    if DCF77_SIGNAL_PIN.is_none() {
        cortex_m::interrupt::free(|cs| {
            *DCF77_SIGNAL_PIN = GLOBAL_PINS_DCF77.borrow(cs).take();
        });
    }
    if NPL_SIGNAL_PIN.is_none() {
        cortex_m::interrupt::free(|cs| {
            *NPL_SIGNAL_PIN = GLOBAL_PINS_NPL.borrow(cs).take();
        });
    }

    // Our interrupts don't clear themselves.
    // Do that here at the end of each condition so we don't immediately jump back to this interrupt handler.

    if let Some(gpio_pin) = DCF77_SIGNAL_PIN {
        let dcf77_signal = gpio_pin;
        if dcf77_signal.is_low().unwrap() {
            G_LOW_EDGE_RECEIVED_DCF77.store(true, Ordering::Release);
            dcf77_signal.clear_interrupt(EdgeLow);
        } else {
            G_HIGH_EDGE_RECEIVED_DCF77.store(true, Ordering::Release);
            dcf77_signal.clear_interrupt(EdgeHigh);
        }
    }
    if let Some(gpio_pin) = NPL_SIGNAL_PIN {
        let npl_signal = gpio_pin;
        if npl_signal.is_low().unwrap() {
            G_LOW_EDGE_RECEIVED_NPL.store(true, Ordering::Release);
            npl_signal.clear_interrupt(EdgeLow);
        } else {
            G_HIGH_EDGE_RECEIVED_NPL.store(true, Ordering::Release);
            npl_signal.clear_interrupt(EdgeHigh);
        }
    }
}

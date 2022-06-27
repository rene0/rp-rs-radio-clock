#![no_std]
#![no_main]

use bsp::entry;
use bsp::hal::timer::{Alarm, Alarm0};
use bsp::hal::{pac, sio::Sio};
use bsp::pac::interrupt;
use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};
use cortex_m::interrupt::Mutex;
use defmt_rtt as _; // otherwise "linking with `flip-link`" fails
use embedded_hal::digital::v2::OutputPin;
use embedded_time::duration::Extensions;
use panic_halt as _;
use rp_pico as bsp;

static GLOBAL_PINS: Mutex<RefCell<Option<Alarm0>>> = Mutex::new(RefCell::new(None));
static G_TOGGLE_LED: AtomicBool = AtomicBool::new(false);
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
    let mut led_pin = pins.led.into_push_pull_output();
    led_pin.set_low().unwrap();
    let mut timer = bsp::hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut alarm0 = timer.alarm_0().unwrap();
    alarm0.enable_interrupt();
    alarm0.schedule(250_000.microseconds()).unwrap();
    cortex_m::interrupt::free(|cs| GLOBAL_PINS.borrow(cs).replace(Some(alarm0)));
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
    }
    let mut led_active = false;
    loop {
        if G_TOGGLE_LED.load(Ordering::Acquire) {
            G_TOGGLE_LED.store(false, Ordering::Release);
            if led_active {
                led_pin.set_high().unwrap();
            } else {
                led_pin.set_low().unwrap();
            }
            led_active = !led_active;
        }
    }
}

#[interrupt]
fn TIMER_IRQ_0() {
    static mut ALARM: Option<Alarm0> = None;

    if ALARM.is_none() {
        // one-time lazy init
        cortex_m::interrupt::free(|cs| {
            *ALARM = GLOBAL_PINS.borrow(cs).take();
        });
    }
    if let Some(alarm) = ALARM {
        G_TOGGLE_LED.store(true, Ordering::Release);
        alarm.clear_interrupt();
        // alarm is oneshot, so re-arm it here:
        alarm.schedule(250_000.microseconds()).unwrap();
    }
}

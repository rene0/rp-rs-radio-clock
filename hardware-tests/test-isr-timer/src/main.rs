#![no_std]
#![no_main]

use core::sync::atomic::{AtomicBool, Ordering};
use defmt_rtt as _; // otherwise "linking with `flip-link`" fails
use embedded_hal::digital::v2::OutputPin;
use fugit::MicrosDurationU32;
use panic_halt as _;
use rp_pico::hal::{
    sio::Sio,
    timer::{Alarm, Alarm0, Timer},
};
use rp_pico::pac::{interrupt, Interrupt, Peripherals, NVIC};
use rp_pico::Pins;

static mut GLOBAL_ALARM: Option<Alarm0> = None;
static G_TOGGLE_LED: AtomicBool = AtomicBool::new(false);

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
    let mut led_pin = pins.led.into_push_pull_output();
    led_pin.set_low().unwrap();

    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut alarm0 = timer.alarm_0().unwrap();
    alarm0.schedule(MicrosDurationU32::micros(250_000)).unwrap();
    alarm0.enable_interrupt();
    unsafe {
        GLOBAL_ALARM = Some(alarm0);
        NVIC::unmask(Interrupt::TIMER_IRQ_0);
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

#[allow(non_snake_case)]
#[interrupt]
unsafe fn TIMER_IRQ_0() {
    G_TOGGLE_LED.store(true, Ordering::Release);

    let alarm = GLOBAL_ALARM.as_mut().unwrap();
    alarm.clear_interrupt();
    // alarm is oneshot, so re-arm it here:
    alarm.schedule(MicrosDurationU32::micros(250_000)).unwrap();
}

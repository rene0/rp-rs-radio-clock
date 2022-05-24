//! DCF77/NPL radio clock on a Pico board
#![no_std]
#![no_main]

use crate::gpio::Pin;
use crate::pac::I2C0;
use bsp::hal::gpio;
use bsp::hal::gpio::Interrupt::{EdgeHigh, EdgeLow};
use bsp::hal::timer::Alarm0;
use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
    I2C,
};
use bsp::pac::interrupt;
use bsp::XOSC_CRYSTAL_FREQ;
use core::cell::RefCell;
use core::fmt::Write;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry; // the macro for our start-up function
use defmt_rtt as _; // otherwise "linking with `flip-link`" fails
use embedded_hal::blocking::delay::{DelayMs, DelayUs};
use embedded_hal::digital::v2::{InputPin, OutputPin, ToggleableOutputPin};
use embedded_time::duration::Extensions;
use embedded_time::fixed_point::FixedPoint; // for .integer()
use embedded_time::rate::Extensions as rate_extensions; // allows for plain "400" in "400.kHz()"
use hd44780_driver::bus::I2CBus;
use hd44780_driver::{Cursor, CursorBlink, HD44780};
use hd44780_helpers::Hd44780Wrapper;
use heapless::String;
use panic_halt as _;
use rp_pico as bsp;

/// I²C address of the PCF8574 adapter, change as needed
const I2C_ADDRESS: u8 = 0x27;
/// Number of rows on the display, change as needed
const DISPLAY_ROWS: u8 = 4;
/// Number of columns on the display, change as needed
const DISPLAY_COLUMNS: u8 = 20;
static G_EDGE_RECEIVED_DCF77: AtomicBool = AtomicBool::new(false);
static G_EDGE_LOW_DCF77: AtomicBool = AtomicBool::new(false);
static G_EDGE_RECEIVED_NPL: AtomicBool = AtomicBool::new(false);
static G_EDGE_LOW_NPL: AtomicBool = AtomicBool::new(false);
// Just use the lower 32 bits of the timer values, we will deal with the 1h11m35s wrap ourselves.
// This saves hardware access to registers, atomic operations, and logic inside get_counter() which are all more expensive.
static G_TIMER_TICK_DCF77: AtomicU32 = AtomicU32::new(0);
static G_TIMER_TICK_NPL: AtomicU32 = AtomicU32::new(0);
// tick-tock
static G_TOGGLE_LED: AtomicBool = AtomicBool::new(false);

type DCF77SignalPin = gpio::Pin<gpio::bank0::Gpio11, gpio::PullDownInput>;
type NPLSignalPin = gpio::Pin<gpio::bank0::Gpio6, gpio::PullDownInput>;
// needed to transfer our pin(s) into the ISR:
static GLOBAL_PIN_DCF77: Mutex<RefCell<Option<DCF77SignalPin>>> = Mutex::new(RefCell::new(None));
static GLOBAL_PIN_NPL: Mutex<RefCell<Option<NPLSignalPin>>> = Mutex::new(RefCell::new(None));
static GLOBAL_TIMER: Mutex<RefCell<Option<bsp::hal::Timer>>> = Mutex::new(RefCell::new(None));
// and one for the timer alarm:
static GLOBAL_ALARM: Mutex<RefCell<Option<Alarm0>>> = Mutex::new(RefCell::new(None));

type I2cDisplay = I2C<
    I2C0,
    (
        Pin<bsp::hal::gpio::pin::bank0::Gpio0, bsp::hal::gpio::FunctionI2C>,
        Pin<bsp::hal::gpio::pin::bank0::Gpio1, bsp::hal::gpio::FunctionI2C>,
    ),
>;
enum DisplayMode {
    Time,
    #[allow(dead_code)]
    Pulses,
}

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);
    // boilerplate from the rp2040 template:
    let clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());
    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Create the I²C drive, using the two pre-configured pins. This will fail
    // at compile time if the pins are in the wrong mode, or if this I²C
    // peripheral isn't available on these pins!
    let sda_pin = pins.gpio0.into_mode::<bsp::hal::gpio::FunctionI2C>();
    let scl_pin = pins.gpio1.into_mode::<bsp::hal::gpio::FunctionI2C>();
    let i2c = bsp::hal::I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        clocks.peripheral_clock,
    );
    // Initialize the display:
    let mut lcd = HD44780::new_i2c(i2c, I2C_ADDRESS, &mut delay).unwrap();
    let lcd_helper = Hd44780Wrapper::new(DISPLAY_COLUMNS, DISPLAY_ROWS);
    lcd.reset(&mut delay).unwrap();
    lcd.clear(&mut delay).unwrap();
    lcd.set_cursor_blink(CursorBlink::Off, &mut delay).unwrap(); // small static cursor
    lcd.set_cursor_visibility(Cursor::Invisible, &mut delay)
        .unwrap(); // turn off completely
    lcd.write_str("DCF77", &mut delay).unwrap();
    lcd.set_cursor_pos(lcd_helper.get_xy(0, 2).unwrap(), &mut delay)
        .unwrap();
    lcd.write_str("NPL", &mut delay).unwrap();

    // Set up the LEDs and signal the "looking for signal" state (time and error LEDs on):
    let mut dcf77_led_time = pins.gpio12.into_push_pull_output();
    let mut dcf77_led_bit = pins.gpio13.into_push_pull_output();
    let mut dcf77_led_error = pins.gpio14.into_push_pull_output();
    dcf77_led_time.set_high().unwrap();
    dcf77_led_bit.set_low().unwrap();
    dcf77_led_error.set_high().unwrap();
    let mut npl_led_time = pins.gpio2.into_push_pull_output();
    let mut npl_led_bit_a = pins.gpio3.into_push_pull_output();
    let mut npl_led_bit_b = pins.gpio4.into_push_pull_output();
    let mut npl_led_error = pins.gpio5.into_push_pull_output();
    npl_led_time.set_high().unwrap();
    npl_led_bit_a.set_low().unwrap();
    npl_led_bit_b.set_low().unwrap();
    npl_led_error.set_high().unwrap();

    let mut led_pin = pins.led.into_push_pull_output();
    led_pin.set_low().unwrap();

    let dcf77_signal_pin = pins.gpio11.into_mode();
    dcf77_signal_pin.set_interrupt_enabled(EdgeHigh, true);
    dcf77_signal_pin.set_interrupt_enabled(EdgeLow, true);
    let npl_signal_pin = pins.gpio6.into_mode();
    npl_signal_pin.set_interrupt_enabled(EdgeHigh, true);
    npl_signal_pin.set_interrupt_enabled(EdgeLow, true);
    let mut timer = bsp::hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut alarm0 = timer.alarm_0().unwrap();

    // Give our pin and timer away to the ISR
    cortex_m::interrupt::free(|cs| GLOBAL_PIN_DCF77.borrow(cs).replace(Some(dcf77_signal_pin)));
    cortex_m::interrupt::free(|cs| GLOBAL_PIN_NPL.borrow(cs).replace(Some(npl_signal_pin)));
    cortex_m::interrupt::free(|cs| GLOBAL_TIMER.borrow(cs).replace(Some(timer)));
    alarm0.enable_interrupt();
    alarm0.schedule(250_000.microseconds()).unwrap();
    cortex_m::interrupt::free(|cs| GLOBAL_ALARM.borrow(cs).replace(Some(alarm0)));
    // Ready, set, go!
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
        pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
    }
    let mut t0_dcf77 = 0;
    let mut t1_dcf77;
    let mut first_dcf77 = true;
    let mut t0_npl = 0;
    let mut t1_npl;
    let mut first_npl = true;
    let display_mode = DisplayMode::Time; // This should become something to loop through with the KY-040
    loop {
        if G_EDGE_RECEIVED_DCF77.load(Ordering::Acquire) {
            t1_dcf77 = G_TIMER_TICK_DCF77.load(Ordering::Acquire);
            if !first_dcf77 {
                if matches!(display_mode, DisplayMode::Pulses) {
                    show_pulses(
                        &mut lcd,
                        &mut delay,
                        &lcd_helper,
                        0,
                        G_EDGE_LOW_DCF77.load(Ordering::Acquire),
                        t0_dcf77,
                        t1_dcf77,
                    );
                }
            } else {
                dcf77_led_time.set_low().unwrap();
                dcf77_led_error.set_low().unwrap();
            }
            t0_dcf77 = t1_dcf77;
            first_dcf77 = false;
            G_EDGE_RECEIVED_DCF77.store(false, Ordering::Release);
        }
        if G_EDGE_RECEIVED_NPL.load(Ordering::Acquire) {
            t1_npl = G_TIMER_TICK_NPL.load(Ordering::Acquire);
            if !first_npl {
                if matches!(display_mode, DisplayMode::Pulses) {
                    show_pulses(
                        &mut lcd,
                        &mut delay,
                        &lcd_helper,
                        2,
                        G_EDGE_LOW_NPL.load(Ordering::Acquire),
                        t0_npl,
                        t1_npl,
                    );
                }
            } else {
                npl_led_time.set_low().unwrap();
                npl_led_error.set_low().unwrap();
            }
            t0_npl = t1_npl;
            first_npl = false;
            G_EDGE_RECEIVED_NPL.store(false, Ordering::Release);
        }
        if G_TOGGLE_LED.load(Ordering::Acquire) {
            led_pin.toggle().unwrap();
            G_TOGGLE_LED.store(false, Ordering::Release);
        }
    }
}

// FIXME skips pulses, updating too slow? HD44780 driver perhaps uses too many delays.
fn show_pulses<D: DelayUs<u16> + DelayMs<u8>>(
    lcd: &mut HD44780<I2CBus<I2cDisplay>>,
    delay: &mut D,
    lcd_helper: &Hd44780Wrapper,
    base_row: u8,
    is_low_edge: bool,
    t0: u32,
    t1: u32,
) {
    let mut str_buf = String::<12>::from("");
    lcd.set_cursor_pos(lcd_helper.get_xy(7, base_row).unwrap(), delay)
        .unwrap();
    str_buf.clear();
    let _ = write!(
        str_buf,
        "{} {:<10}",
        if is_low_edge { 'L' } else { 'H' },
        time_diff(t0, t1)
    );
    lcd.write_str(str_buf.as_str(), delay).unwrap();
    lcd.set_cursor_pos(lcd_helper.get_xy(0, 1).unwrap(), delay)
        .unwrap();
    str_buf.clear();
    let _ = write!(str_buf, "{:<10}  ", t1);
    lcd.write_str(str_buf.as_str(), delay).unwrap();
}

#[inline]
fn time_diff(t0: u32, t1: u32) -> u32 {
    if t1 > t0 {
        t1 - t0
    } else if t0 > 0 {
        u32::MAX - t0 + t1 + 1 /* wrapped, each 1h11m35s */
    } else {
        0 /* initial */
    }
}

#[interrupt]
fn IO_IRQ_BANK0() {
    static mut TICK_TIMER: Option<bsp::hal::Timer> = None;
    static mut DCF77_PIN: Option<DCF77SignalPin> = None;
    static mut PREVIOUS_DCF77_LOW: bool = false;
    static mut NPL_PIN: Option<NPLSignalPin> = None;
    static mut PREVIOUS_NPL_LOW: bool = false;

    // This is one-time lazy initialisation. We steal the variables given to us via `GLOBAL_*`.
    if TICK_TIMER.is_none() {
        cortex_m::interrupt::free(|cs| *TICK_TIMER = GLOBAL_TIMER.borrow(cs).take());
    }
    if DCF77_PIN.is_none() {
        cortex_m::interrupt::free(|cs| *DCF77_PIN = GLOBAL_PIN_DCF77.borrow(cs).take());
    }
    if NPL_PIN.is_none() {
        cortex_m::interrupt::free(|cs| *NPL_PIN = GLOBAL_PIN_NPL.borrow(cs).take());
    }

    // Our edge interrupts don't clear themselves.
    // Do that at the end of the various conditions, so we don't immediately jump back to this interrupt handler.
    if let Some(tick) = TICK_TIMER {
        let now = tick.get_counter_low();

        if let Some(dcf77_signal) = DCF77_PIN {
            let is_low = dcf77_signal.is_low().unwrap();
            if is_low != *PREVIOUS_DCF77_LOW {
                *PREVIOUS_DCF77_LOW = is_low;
                G_TIMER_TICK_DCF77.store(now, Ordering::Release);
                G_EDGE_LOW_DCF77.store(is_low, Ordering::Release);
                G_EDGE_RECEIVED_DCF77.store(true, Ordering::Release);
            }
            dcf77_signal.clear_interrupt(if is_low { EdgeLow } else { EdgeHigh });
        }

        if let Some(npl_signal) = NPL_PIN {
            let is_low = npl_signal.is_low().unwrap();
            if is_low != *PREVIOUS_NPL_LOW {
                *PREVIOUS_NPL_LOW = is_low;
                G_TIMER_TICK_NPL.store(now, Ordering::Release);
                G_EDGE_LOW_NPL.store(is_low, Ordering::Release);
                G_EDGE_RECEIVED_NPL.store(true, Ordering::Release);
            }
            npl_signal.clear_interrupt(if is_low { EdgeLow } else { EdgeHigh });
        }
    }
}

#[interrupt]
fn TIMER_IRQ_0() {
    static mut ALARM: Option<Alarm0> = None;

    if ALARM.is_none() {
        // one-time lazy initialization
        cortex_m::interrupt::free(|cs| *ALARM = GLOBAL_ALARM.borrow(cs).take());
    }
    if let Some(alarm) = ALARM {
        G_TOGGLE_LED.store(true, Ordering::Release);
        alarm.clear_interrupt();
        // alarm is oneshot, so re-arm it here:
        alarm.schedule(250_000.microseconds()).unwrap();
    }
}

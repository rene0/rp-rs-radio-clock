//! DCF77/NPL radio clock on a Pico board
#![no_std]
#![no_main]

use crate::frontend::{dcf77, npl};
use core::{
    cell::RefCell,
    fmt::Write,
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
};
use cortex_m::{delay::Delay, interrupt::Mutex};
use dcf77_utils::DCF77Utils;
use defmt_rtt as _; // otherwise "linking with `flip-link`" fails
use embedded_hal::{
    blocking::delay::{DelayMs, DelayUs},
    digital::v2::{InputPin, OutputPin, ToggleableOutputPin},
};
use fugit::{MicrosDurationU32, RateExtU32};
use hd44780_driver::{bus::I2CBus, Cursor, CursorBlink, HD44780};
use heapless::String;
use npl_utils::NPLUtils;
use panic_halt as _;
use radio_datetime_utils::radio_datetime_helpers;
use rp_pico::hal::{
    clocks,
    clocks::Clock,
    gpio,
    gpio::{bank0, FunctionI2C, Pin, PullDownInput},
    sio::Sio,
    timer::{Alarm, Alarm0},
    watchdog::Watchdog,
    Timer, I2C,
};
use rp_pico::pac;
use rp_pico::pac::{interrupt, CorePeripherals, Peripherals, I2C0, NVIC};
use rp_pico::Pins;

mod frontend;
mod hd44780_helper;

struct ClockHardware {
    edge_received: AtomicBool,
    edge_low: AtomicBool,
    timer_tick: AtomicU32,
}

impl ClockHardware {
    pub const fn new() -> Self {
        Self {
            edge_received: AtomicBool::new(false),
            edge_low: AtomicBool::new(false),
            timer_tick: AtomicU32::new(0),
        }
    }
}

/// I²C address of the PCF8574 adapter, change as needed
const I2C_ADDRESS: u8 = 0x27;
static HW_DCF77: ClockHardware = ClockHardware::new();
static HW_NPL: ClockHardware = ClockHardware::new();
/// Ticks (frames) in each second, to control LEDs and display
const FRAMES_PER_SECOND: u8 = 10;
static G_TIMER_TICK: AtomicBool = AtomicBool::new(false); // tick-tock

type DCF77SignalPin = Pin<bank0::Gpio11, PullDownInput>;
type NPLSignalPin = Pin<bank0::Gpio6, PullDownInput>;
// needed to transfer our pin(s) into the ISR:
static GLOBAL_PIN_DCF77: Mutex<RefCell<Option<DCF77SignalPin>>> = Mutex::new(RefCell::new(None));
static GLOBAL_PIN_NPL: Mutex<RefCell<Option<NPLSignalPin>>> = Mutex::new(RefCell::new(None));
static GLOBAL_TIMER: Mutex<RefCell<Option<Timer>>> = Mutex::new(RefCell::new(None));
// and one for the timer alarm:
static GLOBAL_ALARM: Mutex<RefCell<Option<Alarm0>>> = Mutex::new(RefCell::new(None));

type I2CDisplay = I2C<
    I2C0,
    (
        Pin<bank0::Gpio0, FunctionI2C>,
        Pin<bank0::Gpio1, FunctionI2C>,
    ),
>;
enum DisplayMode {
    Time,
    #[allow(dead_code)]
    Pulses,
    #[allow(dead_code)]
    Times,
}

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
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

    let mut delay = Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure two pins as being I²C, not GPIO
    let sda_pin = pins.gpio0.into_mode::<FunctionI2C>();
    let scl_pin = pins.gpio1.into_mode::<FunctionI2C>();

    // Create the I²C drive, using the two pre-configured pins. This will fail
    // at compile time if the pins are in the wrong mode, or if this I²C
    // peripheral isn't available on these pins!
    let i2c = I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.peripheral_clock,
    );
    // Initialize the display:
    let mut lcd = HD44780::new_i2c(i2c, I2C_ADDRESS, &mut delay).unwrap();
    lcd.reset(&mut delay).unwrap();
    lcd.clear(&mut delay).unwrap();
    lcd.set_cursor_blink(CursorBlink::Off, &mut delay).unwrap(); // small static cursor
    lcd.set_cursor_visibility(Cursor::Invisible, &mut delay)
        .unwrap(); // turn off completely
    lcd.write_str("DCF77", &mut delay).unwrap();
    lcd.set_cursor_pos(hd44780_helper::get_xy(0, 2).unwrap(), &mut delay)
        .unwrap();
    lcd.write_str("NPL", &mut delay).unwrap();

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

    // Set up the LEDs and signal the "looking for signal" state (time and error LEDs on):
    let mut dcf77_led_time = pins.gpio12.into_push_pull_output();
    let mut dcf77_led_bit = pins.gpio13.into_push_pull_output();
    let mut dcf77_led_error = pins.gpio14.into_push_pull_output();
    dcf77::init_leds(
        &mut dcf77_led_time,
        &mut dcf77_led_bit,
        &mut dcf77_led_error,
    );

    let mut npl_led_time = pins.gpio2.into_push_pull_output();
    let mut npl_led_bit_a = pins.gpio3.into_push_pull_output();
    let mut npl_led_bit_b = pins.gpio4.into_push_pull_output();
    let mut npl_led_error = pins.gpio5.into_push_pull_output();
    npl::init_leds(
        &mut npl_led_time,
        &mut npl_led_bit_a,
        &mut npl_led_bit_b,
        &mut npl_led_error,
    );

    // Set up the on-board heartbeat LED:
    let mut led_pin = pins.led.into_push_pull_output();
    led_pin.set_low().unwrap();

    let dcf77_signal_pin = pins.gpio11.into_mode();
    dcf77_signal_pin.set_interrupt_enabled(gpio::Interrupt::EdgeHigh, true);
    dcf77_signal_pin.set_interrupt_enabled(gpio::Interrupt::EdgeLow, true);
    let npl_signal_pin = pins.gpio6.into_mode();
    npl_signal_pin.set_interrupt_enabled(gpio::Interrupt::EdgeHigh, true);
    npl_signal_pin.set_interrupt_enabled(gpio::Interrupt::EdgeLow, true);
    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut alarm0 = timer.alarm_0().unwrap();

    // Give the signal pins and timer away to the ISR
    cortex_m::interrupt::free(|cs| GLOBAL_PIN_DCF77.borrow(cs).replace(Some(dcf77_signal_pin)));
    cortex_m::interrupt::free(|cs| GLOBAL_PIN_NPL.borrow(cs).replace(Some(npl_signal_pin)));
    cortex_m::interrupt::free(|cs| GLOBAL_TIMER.borrow(cs).replace(Some(timer)));
    alarm0.enable_interrupt();
    alarm0
        .schedule(MicrosDurationU32::micros(
            1_000_000 / FRAMES_PER_SECOND as u32,
        ))
        .unwrap();
    cortex_m::interrupt::free(|cs| GLOBAL_ALARM.borrow(cs).replace(Some(alarm0)));
    // Ready, set, go!
    unsafe {
        NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
        NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
    }
    let mut t0_dcf77 = 0;
    let mut dcf77_tick = 0;
    let mut t0_npl = 0;
    let mut npl_tick = 0;
    let display_mode = DisplayMode::Time; // This should become something to loop through with the KY-040
    let mut dcf77 = DCF77Utils::default();
    let mut npl = NPLUtils::default();
    loop {
        if HW_DCF77.edge_received.load(Ordering::Acquire) {
            let t1_dcf77 = HW_DCF77.timer_tick.load(Ordering::Acquire);
            let is_low_edge = HW_DCF77.edge_low.load(Ordering::Acquire);
            if matches!(display_mode, DisplayMode::Pulses) {
                show_pulses(&mut lcd, &mut delay, 0, is_low_edge, t0_dcf77, t1_dcf77);
            } else if matches!(display_mode, DisplayMode::Times) {
                show_times(&mut lcd, &mut delay, 0, t1_dcf77);
            }
            dcf77.handle_new_edge(is_low_edge, t1_dcf77);
            if dcf77.get_new_second() {
                dcf77_tick = 0;
            }
            dcf77::update_bit_leds(dcf77_tick, &dcf77, &mut dcf77_led_bit, &mut dcf77_led_error);
            t0_dcf77 = t1_dcf77;
            HW_DCF77.edge_received.store(false, Ordering::Release);
        }
        if HW_NPL.edge_received.load(Ordering::Acquire) {
            let t1_npl = HW_NPL.timer_tick.load(Ordering::Acquire);
            let is_low_edge = HW_NPL.edge_low.load(Ordering::Acquire);
            if matches!(display_mode, DisplayMode::Pulses) {
                show_pulses(&mut lcd, &mut delay, 2, is_low_edge, t0_npl, t1_npl);
            } else if matches!(display_mode, DisplayMode::Times) {
                show_times(&mut lcd, &mut delay, 0, t1_npl);
            }
            npl.handle_new_edge(is_low_edge, t1_npl);
            if npl.get_new_second() {
                npl_tick = 0;
            }
            npl::update_bit_leds(
                npl_tick,
                &npl,
                &mut npl_led_bit_a,
                &mut npl_led_bit_b,
                &mut npl_led_error,
            );
            t0_npl = t1_npl;
            HW_NPL.edge_received.store(false, Ordering::Release);
        }
        if G_TIMER_TICK.load(Ordering::Acquire) {
            led_pin.toggle().unwrap();
            if t0_dcf77 != 0 {
                dcf77::update_time_led(dcf77_tick, &dcf77, &mut dcf77_led_time);
            }
            if t0_npl != 0 {
                npl::update_time_led(npl_tick, &npl, &mut npl_led_time);
            }
            if dcf77_tick == 0 {
                let mut second = dcf77.get_second() + 1;
                if second == dcf77.get_next_minute_length() {
                    second = 0;
                }
                lcd.set_cursor_pos(hd44780_helper::get_xy(14, 1).unwrap(), &mut delay)
                    .unwrap();
                lcd.write_str(frontend::str_02(Some(second)).as_str(), &mut delay)
                    .unwrap();
            }
            if dcf77_tick == 1 && dcf77.get_new_minute() {
                // print date/time/status
                dcf77.decode_time();
                if !dcf77.get_first_minute() {
                    if matches!(display_mode, DisplayMode::Time) {
                        lcd.set_cursor_pos(hd44780_helper::get_xy(6, 0).unwrap(), &mut delay)
                            .unwrap();
                        lcd.write_str(dcf77::str_status(&dcf77).as_str(), &mut delay)
                            .unwrap();
                    }
                    // Decoded date and time:
                    lcd.set_cursor_pos(hd44780_helper::get_xy(0, 1).unwrap(), &mut delay)
                        .unwrap();
                    lcd.write_str(
                        frontend::str_datetime(dcf77.get_radio_datetime()).as_str(),
                        &mut delay,
                    )
                    .unwrap();
                    // Other things:
                    lcd.set_cursor_pos(hd44780_helper::get_xy(17, 1).unwrap(), &mut delay)
                        .unwrap();
                    lcd.write_str(dcf77::str_misc(&dcf77).as_str(), &mut delay)
                        .unwrap();
                }
            }
            if dcf77_tick == 7 {
                dcf77.increase_second();
            }
            dcf77_tick += 1;
            if dcf77_tick == FRAMES_PER_SECOND {
                dcf77_tick = 0;
            }
            if npl_tick == 0 {
                let mut second = npl.get_second() + 1;
                if second == npl.get_minute_length() {
                    second = 0;
                }
                lcd.set_cursor_pos(hd44780_helper::get_xy(14, 3).unwrap(), &mut delay)
                    .unwrap();
                lcd.write_str(frontend::str_02(Some(second)).as_str(), &mut delay)
                    .unwrap();
            }
            if npl_tick == 1 && npl.get_new_minute() {
                // print date/time/status
                npl.decode_time();
                if !npl.get_first_minute() {
                    if matches!(display_mode, DisplayMode::Time) {
                        lcd.set_cursor_pos(hd44780_helper::get_xy(6, 0).unwrap(), &mut delay)
                            .unwrap();
                        lcd.write_str(npl::str_status(&npl).as_str(), &mut delay)
                            .unwrap();
                    }
                    // Decoded date and time:
                    lcd.set_cursor_pos(hd44780_helper::get_xy(0, 1).unwrap(), &mut delay)
                        .unwrap();
                    lcd.write_str(
                        frontend::str_datetime(npl.get_radio_datetime()).as_str(),
                        &mut delay,
                    )
                    .unwrap();
                    // Other things:
                    lcd.set_cursor_pos(hd44780_helper::get_xy(17, 1).unwrap(), &mut delay)
                        .unwrap();
                    lcd.write_str(npl::str_misc(&npl).as_str(), &mut delay)
                        .unwrap();
                }
            }
            if npl_tick == 7 {
                npl.increase_second();
            }
            npl_tick += 1;
            if npl_tick == FRAMES_PER_SECOND {
                npl_tick = 0;
            }
            G_TIMER_TICK.store(false, Ordering::Release);
        }
    }
}

macro_rules! handle_tick {
    ($pin:ident, $previous_low:ident, $hw:ident, $now:expr) => {
        if let Some(s_pin) = $pin {
            let is_low = s_pin.is_low().unwrap();
            if is_low != *$previous_low {
                *$previous_low = is_low;
                $hw.timer_tick.store($now, Ordering::Release);
                $hw.edge_low.store(is_low, Ordering::Release);
                $hw.edge_received.store(true, Ordering::Release);
            }
            s_pin.clear_interrupt(if is_low {
                gpio::Interrupt::EdgeLow
            } else {
                gpio::Interrupt::EdgeHigh
            });
        }
    };
}

fn show_pulses<D: DelayUs<u16> + DelayMs<u8>>(
    lcd: &mut HD44780<I2CBus<I2CDisplay>>,
    delay: &mut D,
    base_row: u8,
    is_low_edge: bool,
    t0: u32,
    t1: u32,
) {
    lcd.set_cursor_pos(hd44780_helper::get_xy(7, base_row).unwrap(), delay)
        .unwrap();
    let mut str_buf = String::<12>::from("");
    str_buf.clear();
    write!(
        str_buf,
        "{} {:<10}",
        if is_low_edge { 'L' } else { 'H' },
        radio_datetime_helpers::time_diff(t0, t1)
    )
    .unwrap();
    lcd.write_str(str_buf.as_str(), delay).unwrap();
}

fn show_times<D: DelayUs<u16> + DelayMs<u8>>(
    lcd: &mut HD44780<I2CBus<I2CDisplay>>,
    delay: &mut D,
    base_row: u8,
    t1: u32,
) {
    lcd.set_cursor_pos(hd44780_helper::get_xy(7, base_row).unwrap(), delay)
        .unwrap();
    let mut str_buf = String::<12>::from("");
    str_buf.clear();
    write!(str_buf, "T {t1:<10}").unwrap();
    lcd.write_str(str_buf.as_str(), delay).unwrap();
}

#[allow(non_snake_case)]
#[interrupt]
unsafe fn IO_IRQ_BANK0() {
    static mut TICK_TIMER: Option<Timer> = None;
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

        handle_tick!(DCF77_PIN, PREVIOUS_DCF77_LOW, HW_DCF77, now);
        handle_tick!(NPL_PIN, PREVIOUS_NPL_LOW, HW_NPL, now);
    }
}

#[allow(non_snake_case)]
#[interrupt]
unsafe fn TIMER_IRQ_0() {
    static mut ALARM: Option<Alarm0> = None;

    if ALARM.is_none() {
        // one-time lazy initialization
        cortex_m::interrupt::free(|cs| *ALARM = GLOBAL_ALARM.borrow(cs).take());
    }
    if let Some(alarm) = ALARM {
        G_TIMER_TICK.store(true, Ordering::Release);
        alarm.clear_interrupt();
        // alarm is oneshot, so re-arm it here:
        alarm
            .schedule(MicrosDurationU32::micros(
                1_000_000 / FRAMES_PER_SECOND as u32,
            ))
            .unwrap();
    }
}

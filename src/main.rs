//! DCF77/NPL radio clock on a Pico board
#![no_std]
#![no_main]

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio,
    gpio::Interrupt::{EdgeHigh, EdgeLow},
    pac,
    pac::interrupt,
    sio::Sio,
    timer::{Alarm, Alarm0},
    watchdog::Watchdog,
    Timer, I2C,
};
use bsp::{entry, XOSC_CRYSTAL_FREQ};
use core::{
    cell::RefCell,
    cmp::Ordering as spaceship,
    fmt::Write,
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
};
use cortex_m::interrupt::Mutex;
use dcf77_utils::DCF77Utils;
use defmt_rtt as _; // otherwise "linking with `flip-link`" fails
use embedded_hal::{
    blocking::delay::{DelayMs, DelayUs},
    digital::v2::{InputPin, OutputPin, ToggleableOutputPin},
};
use embedded_time::{
    duration::Extensions,
    fixed_point::FixedPoint,             // for .integer()
    rate::Extensions as rate_extensions, // allows for plain "400" in "400.kHz()"
};
use hd44780_driver::{
    bus::I2CBus,
    {Cursor, CursorBlink, HD44780},
};
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
/// Ticks (frames) in each second, to control LEDs and display
const FRAMES_PER_SECOND: u8 = 10;
// tick-tock
static G_TIMER_TICK: AtomicBool = AtomicBool::new(false);

type DCF77SignalPin = gpio::Pin<gpio::bank0::Gpio11, gpio::PullDownInput>;
type NPLSignalPin = gpio::Pin<gpio::bank0::Gpio6, gpio::PullDownInput>;
// needed to transfer our pin(s) into the ISR:
static GLOBAL_PIN_DCF77: Mutex<RefCell<Option<DCF77SignalPin>>> = Mutex::new(RefCell::new(None));
static GLOBAL_PIN_NPL: Mutex<RefCell<Option<NPLSignalPin>>> = Mutex::new(RefCell::new(None));
static GLOBAL_TIMER: Mutex<RefCell<Option<Timer>>> = Mutex::new(RefCell::new(None));
// and one for the timer alarm:
static GLOBAL_ALARM: Mutex<RefCell<Option<Alarm0>>> = Mutex::new(RefCell::new(None));

type I2cDisplay = I2C<
    pac::I2C0,
    (
        gpio::Pin<gpio::pin::bank0::Gpio0, gpio::FunctionI2C>,
        gpio::Pin<gpio::pin::bank0::Gpio1, gpio::FunctionI2C>,
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
    let sda_pin = pins.gpio0.into_mode::<gpio::FunctionI2C>();
    let scl_pin = pins.gpio1.into_mode::<gpio::FunctionI2C>();
    let i2c = I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        clocks.peripheral_clock,
    );
    // Initialize the display:
    let mut lcd = HD44780::new_i2c(i2c, I2C_ADDRESS, &mut delay).unwrap();
    lcd.reset(&mut delay).unwrap();
    lcd.clear(&mut delay).unwrap();
    lcd.set_cursor_blink(CursorBlink::Off, &mut delay).unwrap(); // small static cursor
    lcd.set_cursor_visibility(Cursor::Invisible, &mut delay)
        .unwrap(); // turn off completely
    lcd.write_str("DCF77", &mut delay).unwrap();
    lcd.set_cursor_pos(get_xy(0, 2).unwrap(), &mut delay)
        .unwrap();
    lcd.write_str("NPL", &mut delay).unwrap();

    // Set up the LEDs and signal the "looking for signal" state (time and error LEDs on):
    let mut dcf77_led_time = pins.gpio12.into_push_pull_output();
    let mut dcf77_led_bit = pins.gpio13.into_push_pull_output();
    let mut dcf77_led_error = pins.gpio14.into_push_pull_output();
    let mut dcf77 = DCF77Utils::new(FRAMES_PER_SECOND);
    update_leds_dcf77(
        &dcf77,
        &mut dcf77_led_time,
        &mut dcf77_led_bit,
        &mut dcf77_led_error,
    );
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
    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut alarm0 = timer.alarm_0().unwrap();

    // Give the signal pins and timer away to the ISR
    cortex_m::interrupt::free(|cs| GLOBAL_PIN_DCF77.borrow(cs).replace(Some(dcf77_signal_pin)));
    cortex_m::interrupt::free(|cs| GLOBAL_PIN_NPL.borrow(cs).replace(Some(npl_signal_pin)));
    cortex_m::interrupt::free(|cs| GLOBAL_TIMER.borrow(cs).replace(Some(timer)));
    alarm0.enable_interrupt();
    alarm0
        .schedule((1_000_000 / FRAMES_PER_SECOND as u32).microseconds())
        .unwrap();
    cortex_m::interrupt::free(|cs| GLOBAL_ALARM.borrow(cs).replace(Some(alarm0)));
    // Ready, set, go!
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
        pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
    }
    let mut t0_dcf77 = 0;
    let mut t0_npl = 0;
    let mut first_npl = true;
    let display_mode = DisplayMode::Time; // This should become something to loop through with the KY-040
    loop {
        if G_EDGE_RECEIVED_DCF77.load(Ordering::Acquire) {
            let t1_dcf77 = G_TIMER_TICK_DCF77.load(Ordering::Acquire);
            let is_low_edge = G_EDGE_LOW_DCF77.load(Ordering::Acquire);
            if matches!(display_mode, DisplayMode::Pulses) {
                show_pulses(&mut lcd, &mut delay, 0, is_low_edge, t0_dcf77, t1_dcf77);
            }
            dcf77.handle_new_edge(is_low_edge, t0_dcf77, t1_dcf77);
            update_leds_dcf77(
                &dcf77,
                &mut dcf77_led_time,
                &mut dcf77_led_bit,
                &mut dcf77_led_error,
            );
            t0_dcf77 = t1_dcf77;
            G_EDGE_RECEIVED_DCF77.store(false, Ordering::Release);
        }
        if G_EDGE_RECEIVED_NPL.load(Ordering::Acquire) {
            let t1_npl = G_TIMER_TICK_NPL.load(Ordering::Acquire);
            let is_low_edge = G_EDGE_LOW_NPL.load(Ordering::Acquire);
            if first_npl {
                npl_led_time.set_low().unwrap();
                npl_led_error.set_low().unwrap();
            } else if matches!(display_mode, DisplayMode::Pulses) {
                show_pulses(&mut lcd, &mut delay, 2, is_low_edge, t0_npl, t1_npl);
            }
            t0_npl = t1_npl;
            first_npl = false;
            G_EDGE_RECEIVED_NPL.store(false, Ordering::Release);
        }
        if G_TIMER_TICK.load(Ordering::Acquire) {
            led_pin.toggle().unwrap();
            dcf77.handle_new_timer_tick();
            update_leds_dcf77(
                &dcf77,
                &mut dcf77_led_time,
                &mut dcf77_led_bit,
                &mut dcf77_led_error,
            );
            if dcf77.get_frame_counter() == 1 {
                if dcf77.get_new_minute() && !dcf77.get_first_minute() {
                    // print date/time/status
                    let mut str_buf = String::<14>::from("");
                    let _ = write!(
                        str_buf,
                        "{}{}{}{}{}{}{}{}{}{} {}{}{}",
                        str_jump_year(&dcf77),
                        str_jump_month(&dcf77),
                        str_jump_day(&dcf77),
                        str_jump_weekday(&dcf77),
                        str_jump_hour(&dcf77),
                        str_jump_minute(&dcf77),
                        str_jump_dst(&dcf77),
                        str_parity_3(&dcf77),
                        str_parity_2(&dcf77),
                        str_parity_1(&dcf77),
                        str_bit_0(&dcf77),
                        str_bit_20(&dcf77),
                        str_minute_length(&dcf77),
                    );
                    lcd.set_cursor_pos(get_xy(6, 0).unwrap(), &mut delay)
                        .unwrap();
                    lcd.write_str(str_buf.as_str(), &mut delay).unwrap();
                    // Decoded date and time:
                    let mut str_buf = String::<14>::from("");
                    let _ = write!(
                        str_buf,
                        "{}{}{} {} {}{}",
                        str_02(dcf77.get_radio_datetime().get_year()),
                        str_02(dcf77.get_radio_datetime().get_month()),
                        str_02(dcf77.get_radio_datetime().get_day()),
                        str_weekday(dcf77.get_radio_datetime().get_weekday()),
                        str_02(dcf77.get_radio_datetime().get_hour()),
                        str_02(dcf77.get_radio_datetime().get_minute()),
                    );
                    lcd.set_cursor_pos(get_xy(0, 1).unwrap(), &mut delay)
                        .unwrap();
                    lcd.write_str(str_buf.as_str(), &mut delay).unwrap();
                    // Unusual things:
                    let mut str_buf = String::<3>::from("");
                    let _ = write!(
                        str_buf,
                        "{}{}{}",
                        str_call_bit(&dcf77),
                        str_dst(&dcf77),
                        str_leap_second(&dcf77)
                    );
                    lcd.set_cursor_pos(get_xy(17, 1).unwrap(), &mut delay)
                        .unwrap();
                    lcd.write_str(str_buf.as_str(), &mut delay).unwrap();
                }
                dcf77.increase_second();
                lcd.set_cursor_pos(get_xy(14, 1).unwrap(), &mut delay)
                    .unwrap();
                lcd.write_str(str_02(Some(dcf77.get_second())).as_str(), &mut delay)
                    .unwrap();
            }
            G_TIMER_TICK.store(false, Ordering::Release);
        }
    }
}

/// Return a string version of the given value with leading 0, truncated to two digits or ** for None.
fn str_02(value: Option<u8>) -> String<2> {
    let mut s = String::<2>::from("");
    if value.is_some() {
        write!(s, "{:>02}", value.unwrap()).unwrap();
    } else {
        write!(s, "**").unwrap();
    }
    s
}

/// Return a textual representation of the weekday, Mo-Su or ** for None.
fn str_weekday(weekday: Option<u8>) -> String<2> {
    String::<2>::from(match weekday {
        Some(1) => "Mo",
        Some(2) => "Tu",
        Some(3) => "We",
        Some(4) => "Th",
        Some(5) => "Fr",
        Some(6) => "Sa",
        Some(7) => "Su",
        _ => "**",
    })
}

/// Return if the year has jumped unexpectedly, 'y' or ' '
fn str_jump_year(dcf77: &DCF77Utils) -> char {
    if dcf77.get_radio_datetime().get_jump_year() {
        'y'
    } else {
        ' '
    }
}

/// Return if the month has jumped unexpectedly, 'm' or ' '
fn str_jump_month(dcf77: &DCF77Utils) -> char {
    if dcf77.get_radio_datetime().get_jump_month() {
        'm'
    } else {
        ' '
    }
}

/// Return if the day-of-month has jumped unexpectedly, 'd' or ' '.
fn str_jump_day(dcf77: &DCF77Utils) -> char {
    if dcf77.get_radio_datetime().get_jump_day() {
        'd'
    } else {
        ' '
    }
}

/// Return if the day-of-week has jumped unexpectedly, 'w' or ' '.
fn str_jump_weekday(dcf77: &DCF77Utils) -> char {
    if dcf77.get_radio_datetime().get_jump_weekday() {
        'w'
    } else {
        ' '
    }
}

/// Return if the hour has jumped unexpectedly, 'h' or ' '.
fn str_jump_hour(dcf77: &DCF77Utils) -> char {
    if dcf77.get_radio_datetime().get_jump_hour() {
        'h'
    } else {
        ' '
    }
}

/// Return if the minute has jumped unexpectedly, 'm' or ' '.
fn str_jump_minute(dcf77: &DCF77Utils) -> char {
    if dcf77.get_radio_datetime().get_jump_minute() {
        'm'
    } else {
        ' '
    }
}

/// Return if the daylight saving time status jumped unexpectedly, 't' or ' '.
fn str_jump_dst(dcf77: &DCF77Utils) -> char {
    if dcf77.get_radio_datetime().get_dst().is_some()
        && (dcf77.get_radio_datetime().get_dst().unwrap() & radio_datetime_utils::DST_JUMP) != 0
    {
        't'
    } else {
        ' '
    }
}

/// Returns a character representation of the current DST status.
fn str_dst(dcf77: &DCF77Utils) -> char {
    if let Some(dst) = dcf77.get_radio_datetime().get_dst() {
        let mut res_dst = if (dst & radio_datetime_utils::DST_SUMMER) != 0 {
            's'
        } else {
            'w'
        };
        if (dst & (radio_datetime_utils::DST_ANNOUNCED | radio_datetime_utils::DST_PROCESSED)) != 0
        {
            res_dst = res_dst.to_ascii_uppercase();
        }
        res_dst
    } else {
        '*'
    }
}

/// Returns a character representation of the current leap second status.
fn str_leap_second(dcf77: &DCF77Utils) -> char {
    if let Some(leap_second) = dcf77.get_radio_datetime().get_leap_second() {
        if (leap_second & radio_datetime_utils::LEAP_PROCESSED) != 0 {
            'L'
        } else if (leap_second & radio_datetime_utils::LEAP_NON_ZERO) != 0 {
            '1'
        } else if (leap_second & radio_datetime_utils::LEAP_MISSING) != 0 {
            'm'
        } else if (leap_second & radio_datetime_utils::LEAP_ANNOUNCED) != 0 {
            'l'
        } else {
            ' ' // LEAP_NONE
        }
    } else {
        '*'
    }
}

/// Get a textual version of the minute parity bit, ' ' for OK or '1' for error.
fn str_parity_1(dcf77: &DCF77Utils) -> char {
    if dcf77.get_parity_1() == Some(false) {
        ' '
    } else {
        '1'
    }
}

/// Get a textual version of the hour parity bit, ' ' for OK or '2' for error.
fn str_parity_2(dcf77: &DCF77Utils) -> char {
    if dcf77.get_parity_2() == Some(false) {
        ' '
    } else {
        '2'
    }
}

/// Get a textual version of the date parity bit, ' ' for OK or '3' for error.
fn str_parity_3(dcf77: &DCF77Utils) -> char {
    if dcf77.get_parity_3() == Some(false) {
        ' '
    } else {
        '3'
    }
}

/// Return a character representation of the minute length status.
fn str_minute_length(dcf77: &DCF77Utils) -> char {
    match dcf77.get_second().cmp(&dcf77.get_minute_length()) {
        spaceship::Less => '<',
        spaceship::Equal => ' ',
        spaceship::Greater => '>',
    }
}

/// Return a character representation of the bit 0 status
fn str_bit_0(dcf77: &DCF77Utils) -> char {
    if dcf77.get_bit_0() == Some(false) {
        ' '
    } else {
        'M'
    }
}

/// Return a character representation of the call bit status
fn str_call_bit(dcf77: &DCF77Utils) -> char {
    if dcf77.get_call_bit() == Some(true) {
        'C'
    } else {
        ' '
    }
}

/// Return a character representation of the bit 20 status
fn str_bit_20(dcf77: &DCF77Utils) -> char {
    if dcf77.get_bit_20() == Some(true) {
        ' '
    } else {
        'S'
    }
}

// FIXME skips pulses, updating too slow? HD44780 driver perhaps uses too many delays.
//       Works fine with just showing the pulse lengths of one station.
fn show_pulses<D: DelayUs<u16> + DelayMs<u8>>(
    lcd: &mut HD44780<I2CBus<I2cDisplay>>,
    delay: &mut D,
    base_row: u8,
    is_low_edge: bool,
    t0: u32,
    t1: u32,
) {
    let mut str_buf = String::<12>::from("");
    lcd.set_cursor_pos(get_xy(7, base_row).unwrap(), delay)
        .unwrap();
    str_buf.clear();
    let _ = write!(
        str_buf,
        "{} {:<10}",
        if is_low_edge { 'L' } else { 'H' },
        radio_datetime_utils::time_diff(t0, t1)
    );
    lcd.write_str(str_buf.as_str(), delay).unwrap();
    lcd.set_cursor_pos(get_xy(0, 1).unwrap(), delay).unwrap();
    str_buf.clear();
    let _ = write!(str_buf, "{:<10}  ", t1);
    lcd.write_str(str_buf.as_str(), delay).unwrap();
}

fn update_leds_dcf77(
    dcf77: &DCF77Utils,
    led_time: &mut gpio::Pin<gpio::pin::bank0::Gpio12, gpio::PushPullOutput>,
    led_bit: &mut gpio::Pin<gpio::pin::bank0::Gpio13, gpio::PushPullOutput>,
    led_error: &mut gpio::Pin<gpio::pin::bank0::Gpio14, gpio::PushPullOutput>,
) {
    if dcf77.get_ind_time() {
        led_time.set_high().unwrap();
    } else {
        led_time.set_low().unwrap();
    }
    if dcf77.get_ind_bit() {
        led_bit.set_high().unwrap();
    } else {
        led_bit.set_low().unwrap();
    }
    if dcf77.get_ind_error() {
        led_error.set_high().unwrap();
    } else {
        led_error.set_low().unwrap();
    }
}

/// Gets the one-dimensional HD44780 coordinate for position (x, y) (zero-based)
///
/// <https://web.alfredstate.edu/faculty/weimandn/lcd/lcd_addressing/lcd_addressing_index.html>
/// Assumes type-2 addressing for 16x1 displays
fn get_xy(x: u8, y: u8) -> Option<u8> {
    if (x >= DISPLAY_COLUMNS) || (y >= DISPLAY_ROWS) {
        return None;
    }
    let mut addr = x & 0x3f;
    if (y & 1) == 1 {
        addr += 0x40;
    }
    if (y & 2) == 2 {
        addr += DISPLAY_COLUMNS;
    }
    Some(addr)
}

#[interrupt]
fn IO_IRQ_BANK0() {
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
        G_TIMER_TICK.store(true, Ordering::Release);
        alarm.clear_interrupt();
        // alarm is oneshot, so re-arm it here:
        alarm
            .schedule((1_000_000 / FRAMES_PER_SECOND as u32).microseconds())
            .unwrap();
    }
}

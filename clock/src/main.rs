//! DCF77/NPL radio clock on a Pico board
#![no_std]
#![no_main]

use crate::frontend::{dcf77, npl};
use core::{
    fmt::Write,
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
};
use cortex_m::delay::Delay;
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
    gpio::{bank0, FunctionI2C, Pin, PullDownInput, PullUpInput},
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
static HW_KY040_SW: ClockHardware = ClockHardware::new();

/// Ticks (frames) in each second, to control LEDs and display
const FRAMES_PER_SECOND: u8 = 10;
static G_TIMER_TICK: AtomicBool = AtomicBool::new(false); // tick-tock

type DCF77SignalPin = Pin<bank0::Gpio11, PullDownInput>;
type NPLSignalPin = Pin<bank0::Gpio6, PullDownInput>;
type Ky040SwPin = Pin<bank0::Gpio7, PullUpInput>;

// needed to transfer our pin(s) into the ISR:
static mut GLOBAL_PIN_DCF77: Option<DCF77SignalPin> = None;
static mut GLOBAL_PIN_NPL: Option<NPLSignalPin> = None;
// timer to get the timestamp of the edges:
static mut GLOBAL_TIMER: Option<Timer> = None;
// and one for the timer alarm:
static mut GLOBAL_ALARM: Option<Alarm0> = None;
// click button of the Ky040:
static mut GLOBAL_PIN_KY040_SW: Option<Ky040SwPin> = None;

static G_PREVIOUS_LOW_DCF77: AtomicBool = AtomicBool::new(false);
static G_PREVIOUS_LOW_NPL: AtomicBool = AtomicBool::new(false);
static G_PREVIOUS_LOW_KY040_SW: AtomicBool = AtomicBool::new(false);

type I2CDisplay = I2C<
    I2C0,
    (
        Pin<bank0::Gpio0, FunctionI2C>,
        Pin<bank0::Gpio1, FunctionI2C>,
    ),
>;
enum DisplayMode {
    Status,
    PulsesHigh,
    PulsesLow,
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
    let ky040_sw_pin = pins.gpio7.into_mode();
    ky040_sw_pin.set_interrupt_enabled(gpio::Interrupt::EdgeHigh, true);
    ky040_sw_pin.set_interrupt_enabled(gpio::Interrupt::EdgeLow, true);
    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut alarm0 = timer.alarm_0().unwrap();

    alarm0
        .schedule(MicrosDurationU32::micros(
            1_000_000 / FRAMES_PER_SECOND as u32,
        ))
        .unwrap();
    alarm0.enable_interrupt();
    // Ready, set, go!
    unsafe {
        GLOBAL_PIN_DCF77 = Some(dcf77_signal_pin);
        GLOBAL_PIN_NPL = Some(npl_signal_pin);
        GLOBAL_PIN_KY040_SW = Some(ky040_sw_pin);
        GLOBAL_TIMER = Some(timer);
        GLOBAL_ALARM = Some(alarm0);
        NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
        NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
    }
    let mut t0_dcf77 = 0;
    let mut dcf77_tick = 0;
    let mut t0_npl = 0;
    let mut npl_tick = 0;
    let mut display_mode = DisplayMode::Status;
    let mut dcf77 = DCF77Utils::default();
    let mut npl = NPLUtils::default();
    let mut str_dcf77_status = String::<14>::from("              "); // 14 spaces
    let mut str_npl_status = String::<14>::from("              "); // 14 spaces

    loop {
        if HW_KY040_SW.edge_received.load(Ordering::Acquire) {
            if !HW_KY040_SW.edge_low.load(Ordering::Acquire) {
                // negative logic for the SW pin, pin released
                if matches!(display_mode, DisplayMode::Status) {
                    display_mode = DisplayMode::PulsesHigh;
                } else if matches!(display_mode, DisplayMode::PulsesHigh) {
                    display_mode = DisplayMode::PulsesLow;
                } else {
                    display_mode = DisplayMode::Status;
                    // clear out pulse info, restore any status info
                    lcd.set_cursor_pos(hd44780_helper::get_xy(6, 0).unwrap(), &mut delay)
                        .unwrap();
                    lcd.write_str(str_dcf77_status.as_str(), &mut delay)
                        .unwrap();
                    lcd.set_cursor_pos(hd44780_helper::get_xy(6, 2).unwrap(), &mut delay)
                        .unwrap();
                    lcd.write_str(str_npl_status.as_str(), &mut delay).unwrap();
                }
            }
            HW_KY040_SW.edge_received.store(false, Ordering::Release);
        }
        if HW_DCF77.edge_received.load(Ordering::Acquire) {
            let t1_dcf77 = HW_DCF77.timer_tick.load(Ordering::Acquire);
            let is_low_edge = HW_DCF77.edge_low.load(Ordering::Acquire);
            if matches!(display_mode, DisplayMode::PulsesHigh) {
                show_pulses(
                    &mut lcd,
                    &mut delay,
                    0,
                    false,
                    is_low_edge,
                    t0_dcf77,
                    t1_dcf77,
                );
            } else if matches!(display_mode, DisplayMode::PulsesLow) {
                show_pulses(
                    &mut lcd,
                    &mut delay,
                    0,
                    true,
                    is_low_edge,
                    t0_dcf77,
                    t1_dcf77,
                );
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
            if matches!(display_mode, DisplayMode::PulsesHigh) {
                show_pulses(&mut lcd, &mut delay, 2, false, is_low_edge, t0_npl, t1_npl);
            } else if matches!(display_mode, DisplayMode::PulsesLow) {
                show_pulses(&mut lcd, &mut delay, 2, true, is_low_edge, t0_npl, t1_npl);
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
                    if matches!(display_mode, DisplayMode::Status) {
                        str_dcf77_status = dcf77::str_status(&dcf77);
                        lcd.set_cursor_pos(hd44780_helper::get_xy(6, 0).unwrap(), &mut delay)
                            .unwrap();
                        lcd.write_str(str_dcf77_status.as_str(), &mut delay)
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
                    if matches!(display_mode, DisplayMode::Status) {
                        str_npl_status = npl::str_status(&npl);
                        lcd.set_cursor_pos(hd44780_helper::get_xy(6, 2).unwrap(), &mut delay)
                            .unwrap();
                        lcd.write_str(str_npl_status.as_str(), &mut delay).unwrap();
                    }
                    // Decoded date and time:
                    lcd.set_cursor_pos(hd44780_helper::get_xy(0, 3).unwrap(), &mut delay)
                        .unwrap();
                    lcd.write_str(
                        frontend::str_datetime(npl.get_radio_datetime()).as_str(),
                        &mut delay,
                    )
                    .unwrap();
                    // Other things:
                    lcd.set_cursor_pos(hd44780_helper::get_xy(17, 3).unwrap(), &mut delay)
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
    // Our edge interrupts don't clear themselves.
    // Do that at the end of the various conditions, so we don't immediately jump back to the ISR.
    ($pin:ident, $previous_low:ident, $hw:ident, $now:expr) => {
        let s_pin = $pin.as_mut().unwrap();
        let is_low = s_pin.is_low().unwrap();
        if is_low != $previous_low.load(Ordering::Acquire) {
            $previous_low.store(is_low, Ordering::Release);
            $hw.timer_tick.store($now, Ordering::Release);
            $hw.edge_low.store(is_low, Ordering::Release);
            $hw.edge_received.store(true, Ordering::Release);
        }
        s_pin.clear_interrupt(if is_low {
            gpio::Interrupt::EdgeLow
        } else {
            gpio::Interrupt::EdgeHigh
        });
    };
}

fn show_pulses<D: DelayUs<u16> + DelayMs<u8>>(
    lcd: &mut HD44780<I2CBus<I2CDisplay>>,
    delay: &mut D,
    base_row: u8,
    show_low: bool,
    is_low_edge: bool,
    t0: u32,
    t1: u32,
) {
    if is_low_edge != show_low {
        return;
    }
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

#[allow(non_snake_case)]
#[interrupt]
unsafe fn IO_IRQ_BANK0() {
    let tick = GLOBAL_TIMER.as_mut().unwrap();
    let now = tick.get_counter_low();

    handle_tick!(GLOBAL_PIN_DCF77, G_PREVIOUS_LOW_DCF77, HW_DCF77, now);
    handle_tick!(GLOBAL_PIN_NPL, G_PREVIOUS_LOW_NPL, HW_NPL, now);
    handle_tick!(
        GLOBAL_PIN_KY040_SW,
        G_PREVIOUS_LOW_KY040_SW,
        HW_KY040_SW,
        now
    );
}

#[allow(non_snake_case)]
#[interrupt]
unsafe fn TIMER_IRQ_0() {
    G_TIMER_TICK.store(true, Ordering::Release);

    let alarm = GLOBAL_ALARM.as_mut().unwrap();
    alarm.clear_interrupt();
    // alarm is oneshot, so re-arm it here:
    alarm
        .schedule(MicrosDurationU32::micros(
            1_000_000 / FRAMES_PER_SECOND as u32,
        ))
        .unwrap();
}

//! DCF77/MSF radio clock on a Pico board
#![no_std]
#![no_main]

use crate::clock_hardware::{set_leds_dcf77, set_leds_msf, HardwareEdge};
use crate::frontend::{dcf77, msf};
use core::{
    fmt::Write,
    sync::atomic::{AtomicBool, Ordering},
};
use cortex_m::delay::Delay;
use dcf77_utils::DCF77Utils;
use embedded_hal::digital::v2::{InputPin, OutputPin, ToggleableOutputPin};
use fugit::{MicrosDurationU32, RateExtU32};
use hd44780_driver::{Cursor, CursorBlink, HD44780};
use heapless::String;
use msf60_utils::MSFUtils;
use radio_datetime_utils::radio_datetime_helpers;
use rp_pico::hal::{
    clocks,
    clocks::Clock,
    gpio,
    gpio::{bank0, FunctionI2C, FunctionSioInput, Pin, PullDown, PullUp},
    sio::Sio,
    timer::{Alarm, Alarm0},
    watchdog::Watchdog,
    Timer, I2C,
};
use rp_pico::pac;
use rp_pico::pac::{interrupt, CorePeripherals, Peripherals, NVIC};
use rp_pico::Pins;

extern crate panic_halt; // provides a #[panic_handler] function

mod clock_hardware;
mod frontend;
mod hd44780_helper;

/// I²C address of the PCF8574 adapter, change as needed
const I2C_ADDRESS: u8 = 0x27;

// Definitions for the DCF77 receiver
static HW_DCF77: HardwareEdge = HardwareEdge::new();
static mut GLOBAL_PIN_DCF77: Option<Pin<bank0::Gpio11, FunctionSioInput, PullDown>> = None;

// Definitions for the MSF receiver
static HW_MSF: HardwareEdge = HardwareEdge::new();
static mut GLOBAL_PIN_MSF: Option<Pin<bank0::Gpio6, FunctionSioInput, PullDown>> = None;

// Definitions for the rotary switch
static HW_KY040_SW: HardwareEdge = HardwareEdge::new();
// click button of the Ky040:
static mut GLOBAL_PIN_KY040_SW: Option<Pin<bank0::Gpio7, FunctionSioInput, PullUp>> = None;

// Definitions for the display controller

/// Ticks (frames) in each second, to control LEDs and display
const FRAMES_PER_SECOND: u8 = 10;
static G_TIMER_TICK: AtomicBool = AtomicBool::new(false); // tick-tock

// timer to get the timestamp of the edges:
static mut GLOBAL_TIMER: Option<Timer> = None;
// and one for the timer alarm:
static mut GLOBAL_ALARM: Option<Alarm0> = None;

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
    // Set up basic peripherals:
    let mut pac = Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    // Set up the RP2040 clock:
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

    let core = CorePeripherals::take().unwrap();
    let mut delay = Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // Set up the I/O pin bank:
    let sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure the LCD display using the I²C interface:
    let sda_pin = pins.gpio26.into_function::<FunctionI2C>();
    let scl_pin = pins.gpio27.into_function::<FunctionI2C>();
    let i2c = I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.peripheral_clock,
    );
    let mut lcd = HD44780::new_i2c(i2c, I2C_ADDRESS, &mut delay).unwrap();
    // Initialize the display:
    lcd.reset(&mut delay).unwrap();
    lcd.clear(&mut delay).unwrap();
    lcd.set_cursor_blink(CursorBlink::Off, &mut delay).unwrap(); // small static cursor
    lcd.set_cursor_visibility(Cursor::Invisible, &mut delay)
        .unwrap(); // turn off completely

    hd44780_helper::write_at((0, 0), "DCF77", &mut lcd, &mut delay);
    hd44780_helper::write_at((0, 2), "MSF", &mut lcd, &mut delay);

    // Set power-down pins to LOW, i.e. receiver ON:
    let mut dcf77_pdn = pins.gpio15.into_push_pull_output();
    dcf77_pdn.set_low().unwrap();
    let mut msf_pdn = pins.gpio10.into_push_pull_output();
    msf_pdn.set_low().unwrap();

    // set AGC pins to HIGH, i.e. AGC ON:
    let mut dcf77_aon = pins.gpio22.into_push_pull_output();
    dcf77_aon.set_high().unwrap();
    let mut msf_aon = pins.gpio28.into_push_pull_output();
    msf_aon.set_high().unwrap();

    // Set up the LEDs and signal the "looking for signal" state (time and error LEDs on):
    let mut dcf77_led_time = pins.gpio12.into_push_pull_output();
    let mut dcf77_led_bit = pins.gpio13.into_push_pull_output();
    let mut dcf77_led_error = pins.gpio14.into_push_pull_output();
    let mut dcf77_leds = dcf77::init_leds();
    set_leds_dcf77(
        &mut dcf77_led_time,
        &mut dcf77_led_bit,
        &mut dcf77_led_error,
        dcf77_leds,
    );

    let mut msf_led_time = pins.gpio2.into_push_pull_output();
    let mut msf_led_bit_a = pins.gpio3.into_push_pull_output();
    let mut msf_led_bit_b = pins.gpio4.into_push_pull_output();
    let mut msf_led_error = pins.gpio5.into_push_pull_output();
    let mut msf_leds = msf::init_leds();
    set_leds_msf(
        &mut msf_led_time,
        &mut msf_led_bit_a,
        &mut msf_led_bit_b,
        &mut msf_led_error,
        msf_leds,
    );

    // Set up the on-board heartbeat LED:
    let mut led_pin = pins.led.into_push_pull_output();
    led_pin.set_low().unwrap();

    init_pin!(pins.gpio11.into_pull_down_input(), GLOBAL_PIN_DCF77);
    init_pin!(pins.gpio6.into_pull_down_input(), GLOBAL_PIN_MSF);
    init_pin!(pins.gpio7.into_pull_up_input(), GLOBAL_PIN_KY040_SW);

    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let mut alarm0 = timer.alarm_0().unwrap();
    alarm0
        .schedule(MicrosDurationU32::micros(
            1_000_000 / FRAMES_PER_SECOND as u32,
        ))
        .unwrap();
    alarm0.enable_interrupt();
    // Ready, set, go!
    unsafe {
        GLOBAL_TIMER = Some(timer);
        GLOBAL_ALARM = Some(alarm0);
        NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
        NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
    }
    let mut t0_dcf77 = 0;
    let mut dcf77_tick = 0;
    let mut t0_msf = 0;
    let mut msf_tick = 0;
    let mut display_mode = DisplayMode::Status;
    let mut dcf77 = DCF77Utils::default();
    let mut msf = MSFUtils::default();
    let mut str_dcf77_status: String<14> = String::new();
    write!(str_dcf77_status, "              ").unwrap(); // 14 spaces
    let mut str_msf_status: String<14> = String::new();
    write!(str_msf_status, "              ").unwrap(); // 14 spaces

    loop {
        if HW_KY040_SW.is_new.load(Ordering::Acquire) {
            if !HW_KY040_SW.is_low.load(Ordering::Acquire) {
                // negative logic for the SW pin, pin released
                match display_mode {
                    DisplayMode::Status => display_mode = DisplayMode::PulsesHigh,
                    DisplayMode::PulsesHigh => display_mode = DisplayMode::PulsesLow,
                    DisplayMode::PulsesLow => {
                        display_mode = DisplayMode::Status;
                        // clear out pulse info, restore any status info
                        hd44780_helper::write_at(
                            (6, 0),
                            str_dcf77_status.as_str(),
                            &mut lcd,
                            &mut delay,
                        );
                        hd44780_helper::write_at(
                            (6, 2),
                            str_msf_status.as_str(),
                            &mut lcd,
                            &mut delay,
                        );
                    }
                }
            }
            HW_KY040_SW.is_new.store(false, Ordering::Release);
        }
        let show_low = match display_mode {
            DisplayMode::PulsesHigh => Some(false),
            DisplayMode::PulsesLow => Some(true),
            _ => None,
        };
        if HW_DCF77.is_new.load(Ordering::Acquire) {
            let t1_dcf77 = HW_DCF77.when.load(Ordering::Acquire);
            let is_low_edge = HW_DCF77.is_low.load(Ordering::Acquire);
            if Some(is_low_edge) == show_low {
                hd44780_helper::write_at(
                    (6, 0),
                    str_pulses(is_low_edge, t0_dcf77, t1_dcf77).as_str(),
                    &mut lcd,
                    &mut delay,
                );
            }
            dcf77.handle_new_edge(is_low_edge, t1_dcf77);
            if dcf77.get_new_second() {
                dcf77_tick = 0;
            }
            dcf77_leds = dcf77::update_bit_leds(dcf77_leds, dcf77_tick, &dcf77);
            set_leds_dcf77(
                &mut dcf77_led_time,
                &mut dcf77_led_bit,
                &mut dcf77_led_error,
                dcf77_leds,
            );
            t0_dcf77 = t1_dcf77;
            HW_DCF77.is_new.store(false, Ordering::Release);
        }
        if HW_MSF.is_new.load(Ordering::Acquire) {
            let t1_msf = HW_MSF.when.load(Ordering::Acquire);
            let is_low_edge = HW_MSF.is_low.load(Ordering::Acquire);
            if Some(is_low_edge) == show_low {
                hd44780_helper::write_at(
                    (6, 2),
                    str_pulses(is_low_edge, t0_msf, t1_msf).as_str(),
                    &mut lcd,
                    &mut delay,
                );
            }
            msf.handle_new_edge(is_low_edge, t1_msf);
            if msf.get_new_second() {
                msf_tick = 0;
            }
            msf_leds = msf::update_bit_leds(msf_leds, msf_tick, &msf);
            set_leds_msf(
                &mut msf_led_time,
                &mut msf_led_bit_a,
                &mut msf_led_bit_b,
                &mut msf_led_error,
                msf_leds,
            );
            t0_msf = t1_msf;
            HW_MSF.is_new.store(false, Ordering::Release);
        }
        if G_TIMER_TICK.load(Ordering::Acquire) {
            led_pin.toggle().unwrap();
            if t0_dcf77 != 0 {
                // first DCF77 pulse arrived
                dcf77_leds = dcf77::update_time_led(dcf77_leds, dcf77_tick, &dcf77);
                set_leds_dcf77(
                    &mut dcf77_led_time,
                    &mut dcf77_led_bit,
                    &mut dcf77_led_error,
                    dcf77_leds,
                );
            }
            if t0_msf != 0 {
                // first MSF pulse arrived
                msf_leds = msf::update_time_led(msf_leds, msf_tick, &msf);
                set_leds_msf(
                    &mut msf_led_time,
                    &mut msf_led_bit_a,
                    &mut msf_led_bit_b,
                    &mut msf_led_error,
                    msf_leds,
                );
            }
            if dcf77_tick == 0 {
                let mut second = dcf77.get_second() + 1;
                if second == dcf77.get_next_minute_length() {
                    second = 0;
                }
                hd44780_helper::write_at(
                    (14, 1),
                    frontend::str_02(Some(second)).as_str(),
                    &mut lcd,
                    &mut delay,
                );
            }
            if dcf77_tick == 1 && dcf77.get_new_minute() {
                // print date/time/status
                dcf77.decode_time();
                if !dcf77.get_first_minute() {
                    if matches!(display_mode, DisplayMode::Status) {
                        str_dcf77_status = dcf77::str_status(&dcf77);
                        hd44780_helper::write_at(
                            (6, 0),
                            str_dcf77_status.as_str(),
                            &mut lcd,
                            &mut delay,
                        );
                    }
                    // Decoded date and time:
                    hd44780_helper::write_at(
                        (0, 1),
                        frontend::str_datetime(dcf77.get_radio_datetime(), 7).as_str(),
                        &mut lcd,
                        &mut delay,
                    );
                    // Other things:
                    hd44780_helper::write_at(
                        (17, 1),
                        dcf77::str_misc(&dcf77).as_str(),
                        &mut lcd,
                        &mut delay,
                    );
                }
            }
            if dcf77_tick == 7 {
                dcf77.increase_second();
            }
            dcf77_tick += 1;
            if dcf77_tick == FRAMES_PER_SECOND {
                dcf77_tick = 0;
            }
            if msf_tick == 0 {
                let mut second = msf.get_second() + 1;
                if second == msf.get_minute_length() {
                    second = 0;
                }
                hd44780_helper::write_at(
                    (14, 3),
                    frontend::str_02(Some(second)).as_str(),
                    &mut lcd,
                    &mut delay,
                );
            }
            if msf_tick == 1 && msf.get_new_minute() {
                // print date/time/status
                msf.decode_time();
                if !msf.get_first_minute() {
                    if matches!(display_mode, DisplayMode::Status) {
                        str_msf_status = msf::str_status(&msf);
                        hd44780_helper::write_at(
                            (6, 2),
                            str_msf_status.as_str(),
                            &mut lcd,
                            &mut delay,
                        );
                    }
                    // Decoded date and time:
                    hd44780_helper::write_at(
                        (0, 3),
                        frontend::str_datetime(msf.get_radio_datetime(), 0).as_str(),
                        &mut lcd,
                        &mut delay,
                    );
                    // Other things:
                    hd44780_helper::write_at(
                        (17, 3),
                        msf::str_misc(&msf).as_str(),
                        &mut lcd,
                        &mut delay,
                    );
                }
            }
            if msf_tick == 7 {
                msf.increase_second();
            }
            msf_tick += 1;
            if msf_tick == FRAMES_PER_SECOND {
                msf_tick = 0;
            }
            G_TIMER_TICK.store(false, Ordering::Release);
        }
    }
}

fn str_pulses(is_low_edge: bool, t0: u32, t1: u32) -> String<14> {
    let mut str_buf: String<14> = String::new();
    write!(
        str_buf,
        " {} {:<10} ",
        if is_low_edge { 'L' } else { 'H' },
        radio_datetime_helpers::time_diff(t0, t1)
    )
    .unwrap();
    str_buf
}

macro_rules! handle_edge {
    // Our edge interrupts don't clear themselves.
    // Do that at the end of the various conditions, so we don't immediately jump back to the ISR.
    ($pin:ident, $hw:ident, $now:expr) => {
        let s_pin = $pin.as_mut().unwrap();
        let is_low = s_pin.is_low().unwrap();
        if is_low != $hw.was_low.load(Ordering::Acquire) {
            $hw.was_low.store(is_low, Ordering::Release);
            $hw.when.store($now, Ordering::Release);
            $hw.is_low.store(is_low, Ordering::Release);
            $hw.is_new.store(true, Ordering::Release);
        }
        s_pin.clear_interrupt(if is_low {
            gpio::Interrupt::EdgeLow
        } else {
            gpio::Interrupt::EdgeHigh
        });
    };
}

#[allow(non_snake_case)]
#[interrupt]
unsafe fn IO_IRQ_BANK0() {
    let now = GLOBAL_TIMER.as_mut().unwrap().get_counter_low();

    handle_edge!(GLOBAL_PIN_DCF77, HW_DCF77, now);
    handle_edge!(GLOBAL_PIN_MSF, HW_MSF, now);
    handle_edge!(GLOBAL_PIN_KY040_SW, HW_KY040_SW, now);
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

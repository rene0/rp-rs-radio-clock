//! DCF77/NPL radio clock on a Pico board
#![no_std]
#![no_main]

use cortex_m_rt::entry; // the macro for our start-up function
use defmt_rtt as _; // otherwise "linking with `flip-link`" fails
use panic_halt as _;

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
#[entry]
fn main() -> ! {
    loop {
    }
}

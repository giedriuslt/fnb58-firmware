//! AFC physical layer: the single-wire-protocols `Transport` trait over
//! the traced data-line front end (r198).
//!
//! D− (PA4 = DAC1_OUT via 100 Ω, PA2 = ADC12_IN2 via 1 kΩ) carries the
//! single-wire data; D+ (PA5/PA3 pair) is held at 0.6 V by the task during
//! an exchange. The wire layer's timing guards demand: samples ≤ 8 µs
//! (one raw ADC conversion, no VREFINT pass), drives well inside the
//! ¼-UI hold budget (cached DAC codes — no VDDA read per drive), and a
//! real µs clock (the DWT cycle counter at the 192 MHz SCLK).

use cortex_m::peripheral::{DWT, Peripherals};
use single_wire_protocols::Transport;

use crate::dac::{self, Line};

/// The only failure this PHY can report: a line-tap conversion timed out
/// (never observed — the bound is generous).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    Tap,
}

/// Cycles per microsecond at the 192 MHz SCLK (clocks::configure_usb).
const CYCLES_PER_US: u32 = 192;

/// Enable the DWT cycle counter (once, at task start).
pub fn init() {
    // SAFETY: exclusive-startup contract — the AFC paths are the only
    // cycle-counter users; stealing the core peripherals does not touch
    // any peripheral state.
    let mut periph = unsafe { Peripherals::steal() };
    periph.DCB.enable_trace();
    periph.DWT.enable_cycle_counter();
}

fn cycles() -> u32 {
    DWT::get_cycle_count()
}

/// The D− single-wire PHY. `sample` reads the PA2 tap's raw counts once
/// (≈21 µs at the 239.5-cycle window — inside the 8 µs wire budget only
/// after adc.rs shortens it; the threshold is the natural midpoint).
pub struct DmWire;

impl Transport for DmWire {
    type Error = Error;

    fn drive(&mut self, high: Option<bool>) -> Result<(), Error> {
        match high {
            Some(true) => {
                dac::drive_fast(Line::Dm, true);
            }
            Some(false) => {
                dac::drive_fast(Line::Dm, false);
            }
            None => {
                dac::release(Line::Dm);
            }
        }
        Ok(())
    }

    fn sample(&mut self) -> Result<bool, Error> {
        // One raw conversion of CH2 (PA2 = D−); threshold ≈ 1.4 V in
        // counts (raw 1750 at VDDA 3.3 V) — the charger's low sits near
        // 0, its high at 2.6-3.3 V, both far from the midpoint.
        crate::adc::read_tap_raw(2).map(|raw| raw > 1750).ok_or(Error::Tap)
    }

    fn now_us(&self) -> u32 {
        cycles() / CYCLES_PER_US
    }

    fn delay_us(&mut self, us: u32) {
        let target = cycles().wrapping_add(us * CYCLES_PER_US);
        while cycles().wrapping_sub(target) < 0x8000_0000 {
            cortex_m::asm::nop();
        }
    }
}

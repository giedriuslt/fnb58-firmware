//! AFC physical layer: the single-wire-protocols `Transport` trait over
//! the traced data-line front end (r198).
//!
//! D− (PA4 = DAC1_OUT via 100 Ω, PA2 = ADC3_IN2 via 1 kΩ) carries the
//! single-wire data; D+ (PA5/PA3 pair) is held at 0.6 V by the task during
//! an exchange. The wire layer's timing guards demand: samples ≤ 8 µs
//! (one raw ADC conversion, no VREFINT pass), drives well inside the
//! ¼-UI hold budget (cached DAC codes — no VDDA read per drive), and a
//! real µs clock (the DWT cycle counter at the 192 MHz SCLK).
//!
//! r210: `now_us` is RELATIVE to the exchange start, exactly like
//! kws-x1's Phy (single_wire_drive.rs `Phy::new` captures `start`) —
//! dividing the raw cycle counter by 192 restarted the µs timeline at
//! every 22.36 s counter wrap, and any deadline arithmetic straddling it
//! would corrupt an in-flight exchange. The wire layer also records the
//! raw-count envelope of every exchange (min/max/count, r210) so a
//! failed detection names whether the charger's D− ever moved at all.

use core::sync::atomic::{AtomicU16, AtomicU32, Ordering};
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

/// r210: the exchange's raw-count envelope — sample count, minimum and
/// maximum raw counts seen on the D− tap. Read (and reset) by the task
/// right after a detection exchange: `hi` never lifting above the
/// threshold means the charger never drove D− (a silent or unreachable
/// adapter); `hi` well above it with a failure means framing/timing.
static SAMPLES: AtomicU32 = AtomicU32::new(0);
static RAW_MIN: AtomicU16 = AtomicU16::new(u16::MAX);
static RAW_MAX: AtomicU16 = AtomicU16::new(0);
/// r213: the ping-time sample (our own drive) recorded SEPARATELY from
/// the receive-window envelope — merging them hid whether the adapter's
/// response ever crossed the threshold (its high may sit near 1.2-1.4 V
/// if it is 1.8 V-logic, like the reference phone's GPIO).
static PING_RAW: AtomicU16 = AtomicU16::new(0);

/// Take the recorded envelope (r210/r213), resetting it for the next
/// exchange: `(receive_samples, receive_min, receive_max, ping_raw)`.
pub fn take_envelope() -> (u32, u16, u16, u16) {
    let samples = SAMPLES.swap(0, Ordering::Relaxed);
    let min = RAW_MIN.swap(u16::MAX, Ordering::Relaxed);
    let max = RAW_MAX.swap(0, Ordering::Relaxed);
    let ping = PING_RAW.swap(0, Ordering::Relaxed);
    (samples, min, max, ping)
}

/// Enable the DWT cycle counter (once, at task start).
pub fn init() {
    // SAFETY: exclusive-startup contract — the AFC paths are the only
    // cycle-counter users; stealing the core peripherals does not touch
    // any peripheral state.
    let mut periph = unsafe { Peripherals::steal() };
    periph.DCB.enable_trace();
    periph.DWT.enable_cycle_counter();
}

/// r214: a raw ping-sweep probe, independent of the crate — does ANY
/// master-ping width wake this adapter? For each width: drive D− at the
/// AFC level for `width_ms`, release, then watch the tap for 20 ms
/// recording the peak. Called from the qc task (console 'ap') AFTER a
/// DCP entry (D+ already at 0.6 V, D− free). The adapter answering some
/// width shows a peak far above the ~200-count noise; flat peaks at
/// every width mean this adapter does not speak the D− single-wire AFC
/// we know.
pub fn ping_sweep(log_line: &mut dyn FnMut(&[u8])) {
    const WIDTHS_MS: [u32; 8] = [1, 2, 3, 5, 10, 16, 30, 60];
    for &width_ms in WIDTHS_MS.iter() {
        // A quiet gap before each ping, then the ping and the watch.
        dac::release(Line::Dm);
        spin_us(20_000);
        dac::drive_afc(Line::Dm, true);
        spin_us(width_ms * 1_000);
        dac::release(Line::Dm);
        let mut peak: u16 = 0;
        for _ in 0..125 {
            // One 160 µs UI per sample, matching the crate's cadence.
            spin_us(160);
            if let Some(raw) = crate::adc::read_tap_raw(2) {
                peak = peak.max(raw);
            }
        }
        let mut d = [0u8; 48];
        let mut n = 0;
        for &b in b"[afc] ping w=" {
            d[n] = b;
            n += 1;
        }
        n = crate::push_dec(&mut d, n, u32::from(width_ms));
        for &b in b"ms hi=" {
            d[n] = b;
            n += 1;
        }
        n = crate::push_dec(&mut d, n, u32::from(peak));
        for &b in b"\r\n" {
            d[n] = b;
            n += 1;
        }
        log_line(&d[..n]);
    }
}

/// Busy-wait `us` microseconds on the DWT cycle counter.
fn spin_us(us: u32) {
    let target = cycles().wrapping_add(us * CYCLES_PER_US);
    while cycles().wrapping_sub(target) < 0x8000_0000 {
        cortex_m::asm::nop();
    }
}

fn cycles() -> u32 {
    DWT::get_cycle_count()
}

/// The D− single-wire PHY. One exchange per instance: `new()` captures
/// the DWT start so `now_us` runs relative to it (kws-x1's Phy shape).
/// The FIRST high drive of the exchange samples the line once (r211) —
/// the master ping is 2.56 ms long, so one ≈3 µs conversion cannot hurt
/// its hold budget, and the envelope then PROVES whether our DAC drive
/// lifts D− against the charger's hold-low (contention would leave the
/// raw count near 0 while we drive).
pub struct DmWire {
    start: u32,
    ping_sampled: bool,
}

impl DmWire {
    pub fn new() -> Self {
        Self {
            start: cycles(),
            ping_sampled: false,
        }
    }

    fn record(&mut self, raw: u16) {
        SAMPLES.fetch_add(1, Ordering::Relaxed);
        RAW_MIN.fetch_min(raw, Ordering::Relaxed);
        RAW_MAX.fetch_max(raw, Ordering::Relaxed);
    }
}

impl Transport for DmWire {
    type Error = Error;

    fn drive(&mut self, high: Option<bool>) -> Result<(), Error> {
        match high {
            Some(true) => {
                // r214: the 1.8 V level, not the rail — the reference
                // phone's GPIO drives AFC data at 1.8 V.
                dac::drive_afc(Line::Dm, true);
                if !self.ping_sampled {
                    self.ping_sampled = true;
                    if let Some(raw) = crate::adc::read_tap_raw(2) {
                        PING_RAW.store(raw, Ordering::Relaxed);
                    }
                }
            }
            Some(false) => {
                dac::drive_afc(Line::Dm, false);
            }
            None => {
                dac::release(Line::Dm);
            }
        }
        Ok(())
    }

    fn sample(&mut self) -> Result<bool, Error> {
        // One raw conversion of CH2 (PA2 = D−) on ADC3; threshold ≈ 1.4 V
        // in counts (raw 1750 at VDDA 3.3 V) — the charger's low sits near
        // 0, its high at 2.6-3.3 V, both far from the midpoint. The raw
        // value feeds the exchange envelope (r210).
        match crate::adc::read_tap_raw(2) {
            Some(raw) => {
                self.record(raw);
                Ok(raw > 1750)
            }
            None => Err(Error::Tap),
        }
    }

    fn now_us(&self) -> u32 {
        // Relative to this exchange's start (r210): deadline arithmetic
        // survives the 22.36 s cycle-counter wrap that dividing the raw
        // count could not.
        cycles().wrapping_sub(self.start) / CYCLES_PER_US
    }

    fn delay_us(&mut self, us: u32) {
        let target = cycles().wrapping_add(us * CYCLES_PER_US);
        while cycles().wrapping_sub(target) < 0x8000_0000 {
            cortex_m::asm::nop();
        }
    }
}

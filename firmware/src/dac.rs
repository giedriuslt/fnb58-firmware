//! AT32F403A DAC board layer for the QC2.0 data-line drive (r189).
//!
//! Traced hardware (user board trace, 2026-11-08): PA4 = DAC1_OUT drives
//! D− through 100 Ω, PA5 = DAC2_OUT drives D+ through 100 Ω; the data
//! lines also carry 1 kΩ taps into PA2/PA3 for the ADC. A DAC needs no
//! resistor ladder — the QC levels are codes computed against the
//! measured VDDA (0.6 V ≈ 745, 3.3 V = 4095, 0 V = 0).
//!
//! r194 RELEASE FIX (the r193 hardware evidence): releasing by
//! reconfiguring the pad to input-floating does NOT disconnect the DAC
//! — with the channels enabled at code 0 the lines were held at ~0 V
//! (the charger's 2.7 V divider sagged to 8 mV; during the handshake our
//! own DAC1 held D− at 234 mV → NODCP), while with the DAC never enabled
//! (r192) the same lines read 2700/2700. The DAC output buffer is wired
//! to the pad regardless of the GPIO mode on this part. Release is now
//! CHANNEL DISABLE (`d1en`/`d2en` clear = power-down, output Hi-Z); the
//! pads stay analog permanently and the pad mode is never touched.
//!
//! r198 FAST PATH: the single-wire (AFC) wire layer's hold() guard gives
//! drives a fraction of a 160 µs UI — a per-drive VREFINT ADC pass
//! (hundreds of µs) cannot run there. The High/Mid codes are computed
//! ONCE at init from the measured VDDA and cached; `drive_fast` (boolean
//! level) and `release` are pure register writes. The QC paths keep the
//! Level-based `drive`, now also on the cached codes.

use core::sync::atomic::{AtomicU16, Ordering};
use embassy_at32::gpio::{AnalogPin, Port};
use embassy_at32::pac;
use qc_protocol::qc2::Level;

/// Which data line a drive call targets. GPIOA pin numbers: D− = PA4
/// (DAC1), D+ = PA5 (DAC2).
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Line {
    Dp,
    Dm,
}

/// Nominal level voltage in mV (CHY100 Table 1), for log lines.
pub fn level_mv(level: Level) -> u32 {
    match level {
        Level::Low => 0,
        Level::Mid => 600,
        Level::High => 3300,
    }
}

/// Cached 12-bit codes (r198): computed once at init against the
/// measured VDDA. VDDA drift within a session is far inside the QC
/// comparator windows, and every driven level is ADC-verified anyway.
static CODE_MID: AtomicU16 = AtomicU16::new(745); // 600 mV at 3.3 V
static CODE_HIGH: AtomicU16 = AtomicU16::new(4095); // rail
/// r214: the AFC wire's high level — the Samsung reference phone drives
/// its AFC data GPIO at 1.8 V; our rail-level ping (3.2 V unloaded on
/// this front end) may sit outside the adapter's input range. QC's High
/// requests keep the rail code.
static CODE_AFC: AtomicU16 = AtomicU16::new(2230); // 1.8 V at 3.3 V

fn code_for(mv: u32, vdda_mv: u32) -> u16 {
    if vdda_mv == 0 {
        return 0;
    }
    ((mv * 4095) / vdda_mv).min(4095) as u16
}

/// Enable the DAC clock, put PA4/PA5 in analog mode, cache the level
/// codes from the measured VDDA, and configure both channels with the
/// output buffers ON but the CHANNELS DISABLED (r194: an enabled channel
/// drives its pad in every GPIO mode). Call once before the first drive.
pub fn init() {
    let crm = unsafe { pac::at32f403a::CRM::steal() };
    crm.apb1en().modify(|_, w| w.dac().set_bit());

    // SAFETY: exclusive-startup contract — the QC task owns PA4/PA5.
    unsafe { AnalogPin::new(Port::A, 4).expect("PA4 claim") };
    unsafe { AnalogPin::new(Port::A, 5).expect("PA5 claim") };

    // r198: one VDDA pass here (ADC1 is up — adc::init runs first at
    // boot); every later drive is a pure register write.
    let vdda = crate::adc::vdda_mv().unwrap_or(3300);
    CODE_MID.store(code_for(600, vdda), Ordering::Relaxed);
    CODE_HIGH.store(code_for(3300, vdda), Ordering::Relaxed);
    CODE_AFC.store(code_for(1800, vdda).min(CODE_HIGH.load(Ordering::Relaxed)), Ordering::Relaxed);

    let dac = unsafe { pac::at32f403a::DAC::steal() };
    dac.ctrl().modify(|_, w| {
        w.d1en().clear_bit();
        w.d2en().clear_bit();
        w.d1obdis().clear_bit();
        w.d2obdis().clear_bit()
    });
}

/// Drive one line to `Some(level)` (cached code, then channel enable —
/// the buffer starts at that level; returns the applied mV for logging)
/// or `None` (release: channel power-down, the line returns to the
/// charger).
pub fn drive(line: Line, level: Option<Level>) -> u32 {
    let Some(level) = level else {
        release(line);
        return 0;
    };
    let applied = level_mv(level);
    let code = match level {
        Level::Low => 0,
        Level::Mid => CODE_MID.load(Ordering::Relaxed),
        Level::High => CODE_HIGH.load(Ordering::Relaxed),
    };
    write_code(line, code);
    applied
}

/// Fast boolean drive for the single-wire paths: high = cached High code,
/// low = code 0. Pure register writes — safe inside a ¼-UI hold.
pub fn drive_fast(line: Line, high: bool) {
    let code = if high {
        CODE_HIGH.load(Ordering::Relaxed)
    } else {
        0
    };
    write_code(line, code);
}

/// r214: the AFC wire variant — high = the cached 1.8 V code (the
/// reference phone's GPIO level; a rail-level ping read as out-of-range
/// by the adapter on hardware).
pub fn drive_afc(line: Line, high: bool) {
    let code = if high {
        CODE_AFC.load(Ordering::Relaxed)
    } else {
        0
    };
    write_code(line, code);
}

/// Release one line: channel power-down (r194), output Hi-Z.
pub fn release(line: Line) {
    let dac = unsafe { pac::at32f403a::DAC::steal() };
    match line {
        Line::Dm => {
            dac.ctrl().modify(|_, w| w.d1en().clear_bit());
        }
        Line::Dp => {
            dac.ctrl().modify(|_, w| w.d2en().clear_bit());
        }
    }
}

/// Code first, then enable: the buffer starts driving at the code.
fn write_code(line: Line, code: u16) {
    let dac = unsafe { pac::at32f403a::DAC::steal() };
    match line {
        Line::Dm => {
            dac.d1dth12r().write(|w| unsafe { w.d1dt12r().bits(code) });
            dac.ctrl().modify(|_, w| w.d1en().set_bit());
        }
        Line::Dp => {
            dac.d2dth12r().write(|w| unsafe { w.d2dt12r().bits(code) });
            dac.ctrl().modify(|_, w| w.d2en().set_bit());
        }
    }
}

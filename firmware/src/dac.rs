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

/// Enable the DAC clock, put PA4/PA5 in analog mode, and configure both
/// channels with the output buffers ON but the CHANNELS DISABLED
/// (r194: an enabled channel drives its pad in every GPIO mode — the
/// r193 release-by-pad-mode left code-0 outputs crushing both data
/// lines to ~0 V). Call once before the QC task's first drive.
pub fn init() {
    let crm = unsafe { pac::at32f403a::CRM::steal() };
    crm.apb1en().modify(|_, w| w.dac().set_bit());

    // SAFETY: exclusive-startup contract — the QC task owns PA4/PA5.
    unsafe { AnalogPin::new(Port::A, 4).expect("PA4 claim") };
    unsafe { AnalogPin::new(Port::A, 5).expect("PA5 claim") };

    let dac = unsafe { pac::at32f403a::DAC::steal() };
    dac.ctrl().modify(|_, w| {
        w.d1en().clear_bit();
        w.d2en().clear_bit();
        w.d1obdis().clear_bit();
        w.d2obdis().clear_bit()
    });
}

/// Drive one line to `Some(level)` (write the code, then enable the
/// channel — the buffer starts driving at that level; returns the
/// applied mV for logging) or `None` (release: channel power-down, the
/// pad stays analog and the line returns to the charger).
pub fn drive(line: Line, level: Option<Level>) -> u32 {
    let dac = unsafe { pac::at32f403a::DAC::steal() };
    let Some(level) = level else {
        match line {
            Line::Dm => dac.ctrl().modify(|_, w| w.d1en().clear_bit()),
            Line::Dp => dac.ctrl().modify(|_, w| w.d2en().clear_bit()),
        };
        return 0;
    };
    let vdda = crate::adc::vdda_mv().unwrap_or(3300);
    let code = code_for(level_mv(level), vdda);
    match line {
        // Code first, then enable: the buffer starts at the right level.
        Line::Dm => {
            dac.d1dth12r().write(|w| unsafe { w.d1dt12r().bits(code) });
            dac.ctrl().modify(|_, w| w.d1en().set_bit());
        }
        Line::Dp => {
            dac.d2dth12r().write(|w| unsafe { w.d2dt12r().bits(code) });
            dac.ctrl().modify(|_, w| w.d2en().set_bit());
        }
    };
    level_mv(level)
}

fn code_for(mv: u32, vdda_mv: u32) -> u16 {
    if vdda_mv == 0 {
        return 0;
    }
    ((mv * 4095) / vdda_mv).min(4095) as u16
}

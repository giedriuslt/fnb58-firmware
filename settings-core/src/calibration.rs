//! r221: per-unit corrections to the current and bus voltage readings,
//! the calibration steps that find them, and their settings records —
//! the port of kws-x1's `firmware-core/src/calibration.rs` onto the
//! FNB58's meter.
//!
//! The meter is a TPA626 driven in the INA226 protocol (the same chip
//! family kws-x1 calibrates), so the model ports 1:1: a zero offset
//! subtracted first, then a gain correction in parts per million, taken
//! from user-supplied reference values (the wizard's entry steps start
//! at this meter's own averaged reading; the user walks the value to
//! what their reference meter shows — no fixed calibration points).
//!
//! Two adaptations, both documented at the site:
//! - the current register LSB is 250 µA here vs kws-x1's 500 µA — all
//!   the limits stay kws-x1's. The register scaling follows the CAL
//!   register the board writes (main.rs): one current-register count =
//!   (2048/CAL) x 2.5 µV of shunt, so r46's CAL=10240 made the register
//!   span ±16.38 mV = ±1.638 A through the 10 mOhm shunt — 4x short of
//!   the FNB58's 0–7 A range (never tested above ~1.6 A, so it never
//!   showed). r221 programs CAL=2048 (kws-x1's own CAL for their
//!   5 mOhm/500 µA setup): 2.5 µV/count = 250 µA through the
//!   user-confirmed 10 mOhm (2x20 mOhm parallel), the register maps the
//!   shunt register 1:1, and the span is ±8.19 A — the full range with
//!   margin. CAL=2560 (200 µA) was rejected: it clips at 6.55 A;
//! - kws-x1's zero step is a HELD OK with a plain OK skipping it; the
//!   FNB58 has no long-press support (r199), so [`Wizard::ok`] on the
//!   zero step takes the zero directly and stays on refusal.

/// Current register LSB in µA: 0.25 mA — one register count = 2.5 µV of
/// shunt under CAL=2048 (main.rs), through the user-confirmed 10 mOhm
/// shunt (2x20 mOhm parallel). NOT kws-x1's 500 µA (their 5 mOhm shunt).
pub const CURRENT_LSB_UA: i64 = 250;
/// Bus Voltage register LSB in µV (INA Table 7-8, same as kws-x1).
pub const BUS_LSB_UV: i64 = 1250;
/// Corrections are kept in parts per million.
const PPM: i64 = 1_000_000;
/// Largest correction: ±5%. The INA226's gain errors are at most 0.1%
/// (INA Electrical Characteristics) and a current shunt is typically
/// within 1%, so a reference further off than this is more likely wrong
/// than the meter.
pub const MAX_CORRECTION_PPM: i64 = 50_000;
/// Largest zero offset: ten times the INA226's 10 µV worst-case shunt
/// offset across this board's 10 mOhm shunt (1 mA), rounded up to
/// kws-x1's 20 mA — more current than this is a load, not an offset.
pub const MAX_ZERO_UA: i64 = 20_000;
/// Least current a current correction is taken at: 0.5 A, where one
/// 0.05 mA register step is 0.01%.
pub const MIN_LOAD_UA: i64 = 500_000;
/// Least bus voltage a voltage correction is taken at: 4 V, below any
/// USB supply's 5 V, where one 1.25 mV register step is 0.03%.
pub const MIN_BUS_UV: i64 = 4_000_000;
/// Readings averaged for a calibration step: 16 of the main loop's
/// 200 ms meter reads = a 3.2 s window (kws-x1: 16 x 100 ms = 1.6 s —
/// same sample count, twice the window).
pub const AVERAGE_SAMPLES: usize = 16;

/// What the meter adds to its raw readings. The default changes nothing.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Calibration {
    /// Current read with nothing drawing it, in µA, subtracted first.
    pub zero_ua: i32,
    /// Current and bus voltage gain corrections, in ppm.
    pub current_ppm: i32,
    pub voltage_ppm: i32,
}

/// `value` scaled by `ppm`, no unit change (the µV/µA-level variant the
/// OpenFNB58 report path uses; the display conversions divide by 1000
/// afterwards). Unsigned division inside, like kws-x1's `scaled_milli`:
/// the firmware already links unsigned 64-bit division, and signed
/// division would add a routine it has no room for.
fn scaled(value: i64, ppm: i32) -> i64 {
    let gain = (PPM + i64::from(ppm)) as u64;
    let magnitude = (value.unsigned_abs() * gain / PPM as u64) as i64;
    if value < 0 { -magnitude } else { magnitude }
}

/// [`scaled`] divided by 1000, toward zero (µV -> mV, µA -> mA).
fn scaled_milli(value: i64, ppm: i32) -> i64 {
    scaled(value, ppm) / 1000
}

/// The correction that turns `measured` into `target` (both positive, in
/// µ-units), rounded to the nearest ppm; unsigned as in [`scaled`].
fn correction(measured: i64, target: i64) -> Result<i32, Refusal> {
    let (measured, target) = (measured as u64, target as u64);
    let ppm = ((target * PPM as u64 + measured / 2) / measured) as i64 - PPM;
    if ppm.abs() > MAX_CORRECTION_PPM {
        return Err(Refusal::OutOfRange);
    }
    Ok(ppm as i32)
}

impl Calibration {
    /// Bus voltage in mV from the Bus Voltage register.
    pub fn millivolts(self, raw: u16) -> u32 {
        scaled_milli(i64::from(raw) * BUS_LSB_UV, self.voltage_ppm) as u32
    }

    /// Current in mA from the two's complement Current register (INA
    /// Table 7-1); negative when it flows from plug to receptacle.
    pub fn milliamps(self, raw: u16) -> i32 {
        let ua = i64::from(raw as i16) * CURRENT_LSB_UA - i64::from(self.zero_ua);
        scaled_milli(ua, self.current_ppm) as i32
    }

    /// Bus voltage in µV, calibrated at the register's native resolution
    /// (the OpenFNB58 report path; the display's mV truncates 1.25 mV
    /// register steps to 1 mV).
    pub fn microvolts(self, raw: u16) -> i64 {
        scaled(i64::from(raw) * BUS_LSB_UV, self.voltage_ppm)
    }

    /// Current in µA, calibrated at the register's native resolution.
    pub fn microamps(self, raw: u16) -> i64 {
        scaled(
            i64::from(raw as i16) * CURRENT_LSB_UA - i64::from(self.zero_ua),
            self.current_ppm,
        )
    }

    /// The averaged current's magnitude in mA, as this calibration shows it.
    pub fn mean_milliamps(self, mean: Mean) -> u32 {
        scaled_milli(mean.ua - i64::from(self.zero_ua), self.current_ppm).unsigned_abs() as u32
    }

    /// The averaged bus voltage in mV, as this calibration shows it.
    pub fn mean_millivolts(self, mean: Mean) -> u32 {
        scaled_milli(mean.uv, self.voltage_ppm) as u32
    }

    /// Take `mean`, read with nothing drawing current, as the zero.
    pub fn zeroed(self, mean: Mean) -> Result<Self, Refusal> {
        if mean.ua.abs() > MAX_ZERO_UA {
            return Err(Refusal::Load);
        }
        Ok(Self {
            zero_ua: mean.ua as i32,
            ..self
        })
    }

    /// Correct the current so `mean` shows as `target_ma`.
    pub fn current_matched(self, mean: Mean, target_ma: u32) -> Result<Self, Refusal> {
        let measured = (mean.ua - i64::from(self.zero_ua)).abs();
        if measured < MIN_LOAD_UA {
            return Err(Refusal::NoLoad);
        }
        Ok(Self {
            current_ppm: correction(measured, i64::from(target_ma) * 1000)?,
            ..self
        })
    }

    /// Correct the bus voltage so `mean` shows as `target_mv`.
    pub fn voltage_matched(self, mean: Mean, target_mv: u32) -> Result<Self, Refusal> {
        if mean.uv < MIN_BUS_UV {
            return Err(Refusal::NoVoltage);
        }
        Ok(Self {
            voltage_ppm: correction(mean.uv, i64::from(target_mv) * 1000)?,
            ..self
        })
    }

    /// One settings record per field, see [`Found::read`].
    pub fn records(self) -> [u32; FIELDS] {
        let values = [self.zero_ua, self.current_ppm, self.voltage_ppm];
        core::array::from_fn(|field| {
            RECORD_TAG << 24 | (field as u32) << FIELD_SHIFT | values[field] as u32 & VALUE_MASK
        })
    }
}

/// Why a calibration step kept the previous value. The messages are the
/// FNB58 LCD's own (lcd.rs FONT_CHARS has no '%', ',' or '<'/'>' — the
/// strings must only use its glyphs; kws-x1's prose messages are the
/// reference).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Refusal {
    /// No averaged reading yet, or the bus is not read.
    NoReading,
    /// Zero: more current flows than an offset explains.
    Load,
    /// Current: too little flows to correct it precisely.
    NoLoad,
    /// Voltage: too little on the bus to correct it precisely.
    NoVoltage,
    /// Over [`MAX_CORRECTION_PPM`] off.
    OutOfRange,
}

impl Refusal {
    /// What the calibration screen shows; the limits it names are
    /// [`MIN_LOAD_UA`], [`MIN_BUS_UV`] and [`MAX_CORRECTION_PPM`].
    pub fn message(self) -> &'static str {
        match self {
            Self::NoReading => "NO STEADY READ YET",
            Self::Load => "CURRENT FLOWS: UNPLUG",
            Self::NoLoad => "NEEDS 0.5A OR MORE",
            Self::NoVoltage => "NEEDS 4V OR MORE",
            Self::OutOfRange => "OVER 5 OUT OF RANGE",
        }
    }
}

/// The last [`AVERAGE_SAMPLES`] raw readings, uncalibrated, in µV and µA.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Mean {
    pub uv: i64,
    pub ua: i64,
}

/// Running window of raw readings for [`Mean`].
#[derive(Clone, Copy, Debug, Default)]
pub struct Average {
    samples: [(u16, u16); AVERAGE_SAMPLES],
    next: usize,
    filled: usize,
}

impl Average {
    /// Add one pair of raw Bus Voltage and Current registers.
    pub fn add(&mut self, raw_v: u16, raw_i: u16) {
        self.samples[self.next] = (raw_v, raw_i);
        self.next = (self.next + 1) % AVERAGE_SAMPLES;
        self.filled = (self.filled + 1).min(AVERAGE_SAMPLES);
    }

    /// Forget the window, e.g. while the meter is not read.
    pub fn clear(&mut self) {
        self.filled = 0;
    }

    /// The window's mean, once it is full.
    pub fn mean(&self) -> Option<Mean> {
        if self.filled < AVERAGE_SAMPLES {
            return None;
        }
        // In 32 bits, which the core divides in hardware: 16 full-scale
        // registers times their LSB stay below 2^32 (2^31 for the current).
        let count = AVERAGE_SAMPLES as u32;
        let (v, i) = self
            .samples
            .iter()
            .fold((0u32, 0i32), |(v, i), &(raw_v, raw_i)| {
                (v + u32::from(raw_v), i + i32::from(raw_i as i16))
            });
        Some(Mean {
            uv: i64::from(v * BUS_LSB_UV as u32 / count),
            ua: i64::from(i * CURRENT_LSB_UA as i32 / count as i32),
        })
    }
}

/// Calibration fields, each its own record: zero, current, voltage.
pub const FIELDS: usize = 3;
/// Record tag in the top byte, distinct from the settings word's, so
/// firmware without calibration skips these records.
const RECORD_TAG: u32 = 0xca;
/// The field index in bits 23..22, a two's complement value in 21..0.
const FIELD_SHIFT: u32 = 22;
const VALUE_MASK: u32 = (1 << FIELD_SHIFT) - 1;
/// Largest value a record may hold: the zero limit and the correction
/// limit both fit, anything larger is not a record this firmware wrote.
const MAX_VALUE: i64 = if MAX_ZERO_UA > MAX_CORRECTION_PPM {
    MAX_ZERO_UA
} else {
    MAX_CORRECTION_PPM
};

/// The newest record of each field found so far.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Found([Option<i32>; FIELDS]);

impl Found {
    /// Take `word` if it is a calibration record; later records win.
    pub fn read(&mut self, word: u32) -> bool {
        let field = (word >> FIELD_SHIFT & 0b11) as usize;
        // Sign-extend the 22-bit value.
        let value = ((word & VALUE_MASK) << (32 - FIELD_SHIFT)) as i32 >> (32 - FIELD_SHIFT);
        if word >> 24 != RECORD_TAG || field >= FIELDS || i64::from(value).abs() > MAX_VALUE {
            return false;
        }
        self.0[field] = Some(value);
        true
    }

    pub fn any(self) -> bool {
        self.0.iter().any(Option::is_some)
    }

    pub fn calibration(self) -> Calibration {
        let [zero_ua, current_ppm, voltage_ppm] = self.0.map(|field| field.unwrap_or(0));
        Calibration {
            zero_ua,
            current_ppm,
            voltage_ppm,
        }
    }
}

/// What an entry step corrects.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Quantity {
    Current,
    Voltage,
}

/// A calibration step, in order; each can be skipped.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Step {
    /// OK with nothing drawing current.
    Zero,
    /// Left/Right move this meter's averaged reading at the step's start
    /// (`None` without one) by `adjust`, in mA or mV, to the reference
    /// meter's value.
    Entry {
        quantity: Quantity,
        start: Option<u32>,
        adjust: i32,
    },
}

impl Step {
    /// The value being entered: the start plus the adjustment.
    pub fn entered(self) -> Option<u32> {
        match self {
            Self::Entry {
                start: Some(start),
                adjust,
                ..
            } => Some(start.saturating_add_signed(adjust)),
            _ => None,
        }
    }
}

/// Entry steps per Left/Right press and per repeat, in mA or mV: one
/// display digit, then ten (the repeat is the main loop's held-button
/// adjust — the FNB58 has no button-repeat of its own).
const ENTRY_STEP: i32 = 1;
const ENTRY_REPEAT: i32 = 10;

/// The calibration steps in progress: nothing is applied until the last
/// OK (kws-x1's Wizard; the FNB58's zero step folds the hold-OK into a
/// plain OK — no long-press support, r199).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Wizard {
    pub step: Step,
    /// The calibration so far, applied once the steps finish.
    pub pending: Calibration,
    /// Why the last OK kept the previous value.
    pub refusal: Option<Refusal>,
}

impl Wizard {
    pub fn new(calibration: Calibration) -> Self {
        Self {
            step: Step::Zero,
            pending: calibration,
            refusal: None,
        }
    }

    /// OK. On the zero step, take the zero (kws-x1's hold-OK; their
    /// plain-OK skip has no FNB58 gesture) and go on — a refusal stays
    /// on the step with its message. On an entry step: keep an unchanged
    /// value, else take the entered one, and go on. Returns the
    /// calibration once the last step is done.
    pub fn ok(&mut self, mean: Option<Mean>) -> Option<Calibration> {
        if self.step == Step::Zero {
            match mean
                .ok_or(Refusal::NoReading)
                .and_then(|mean| self.pending.zeroed(mean))
            {
                Ok(zeroed) => {
                    self.pending = zeroed;
                    self.next(mean);
                }
                Err(refusal) => self.refusal = Some(refusal),
            }
            return None;
        }
        let Step::Entry {
            quantity, adjust, ..
        } = self.step
        else {
            // Unreachable by construction (Zero is handled above); refuse
            // to panic on-device (panic = abort = a dead screen).
            return None;
        };
        if let Some(target) = self.step.entered().filter(|_| adjust != 0) {
            let matched = mean
                .ok_or(Refusal::NoReading)
                .and_then(|mean| match quantity {
                    Quantity::Current => self.pending.current_matched(mean, target),
                    Quantity::Voltage => self.pending.voltage_matched(mean, target),
                });
            match matched {
                Ok(matched) => self.pending = matched,
                Err(refusal) => {
                    self.refusal = Some(refusal);
                    return None;
                }
            }
        }
        if quantity == Quantity::Voltage {
            return Some(self.pending);
        }
        self.next(mean);
        None
    }

    /// Left (`forward` false) or Right; `repeat` for the held-button
    /// adjust. A step without a start reading adjusts nothing.
    pub fn step(&mut self, forward: bool, repeat: bool) {
        let size = if repeat { ENTRY_REPEAT } else { ENTRY_STEP };
        if let Step::Entry {
            start: Some(start),
            adjust,
            ..
        } = &mut self.step
        {
            // Never below zero.
            *adjust = (*adjust + if forward { size } else { -size }).max(-(*start as i32));
            self.refusal = None;
        }
    }

    /// The following step, starting from this meter's averaged reading.
    fn next(&mut self, mean: Option<Mean>) {
        let pending = self.pending;
        let quantity = match self.step {
            Step::Zero => Quantity::Current,
            Step::Entry { .. } => Quantity::Voltage,
        };
        self.refusal = None;
        self.step = Step::Entry {
            quantity,
            start: mean.map(|mean| match quantity {
                Quantity::Current => pending.mean_milliamps(mean),
                Quantity::Voltage => pending.mean_millivolts(mean),
            }),
            adjust: 0,
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every reading goes through the calibration: the default must give
    /// exactly what the plain register conversions gave (the FNB58's
    /// 1.25 mV and 0.25 mA register steps, main.rs).
    #[test]
    fn default_changes_no_reading() {
        let calibration = Calibration::default();
        for raw in [0u16, 1, 2, 3, 4001, 16000, 0x7fff, 0x8000, 0xfffe, 0xffff] {
            assert_eq!(calibration.millivolts(raw), u32::from(raw) * 5 / 4);
            assert_eq!(calibration.milliamps(raw), i32::from(raw as i16) / 4);
        }
    }

    /// The report path's native-resolution variants agree with the
    /// display conversions (mV = µV/1000) and keep sub-mA detail.
    #[test]
    fn micro_conversions_match_milli() {
        let calibration = Calibration {
            zero_ua: -2_500,
            current_ppm: 8_000,
            voltage_ppm: -1_000,
        };
        for raw in [0u16, 1, 4001, 16000, 0x7fff, 0xfffe] {
            assert_eq!(
                calibration.microvolts(raw) / 1000,
                i64::from(calibration.millivolts(raw))
            );
        }
        for raw in [0u16, 1, 100, 1000, 0x7fff, 0x8000, 0xfffe] {
            assert_eq!(calibration.microamps(raw) / 1000, i64::from(calibration.milliamps(raw)));
        }
        // Sub-mA detail survives: one register step above the zero.
        assert_eq!(calibration.microamps(1), scaled(250 - (-2_500), 8_000));
    }

    #[test]
    fn steps_correct_a_reading_and_keep_skipped_ones() {
        // 1.000 A and 20.000 V averaged, with a 1.5 mA offset. The µA
        // means are raw counts x 250 (the CAL=2048 register step,
        // main.rs): raw 4006 = 1.0015 A, raw 6 = 1.5 mA — all inside
        // the register's +/-8.19 A span (r46's CAL=10240 capped it at
        // 1.638 A, fixed r221b).
        let loaded = Mean {
            uv: 20_000_000,
            ua: 1_001_500,
        };
        let mut wizard = Wizard::new(Calibration::default());
        // The zero step: an idle mean with a 1.5 mA offset reads as the zero.
        wizard.ok(Some(Mean {
            uv: 20_000_000,
            ua: 1_500,
        }));
        assert_eq!(wizard.pending.zero_ua, 1_500);
        // The current step starts at the reading without the offset.
        wizard.step = Step::Entry {
            quantity: Quantity::Current,
            start: Some(wizard.pending.mean_milliamps(loaded)),
            adjust: 0,
        };
        assert_eq!(wizard.step.entered(), Some(1_000));
        // The meter reads 1.0015 A; the user walks +20 (two repeats).
        for _ in 0..2 {
            wizard.step(true, true);
        }
        assert_eq!(wizard.step.entered(), Some(1_020));
        assert_eq!(wizard.ok(Some(loaded)), None);
        // Voltage left as it was: the steps finish without a voltage change.
        let calibration = wizard.ok(Some(loaded)).unwrap_or_default();
        assert_eq!(calibration.voltage_ppm, 0);
        // 4006 raw = 4006 x 250 uA = 1.0015 A; the +1.02 A entry (+2%)
        // reads back as 1020 mA.
        assert_eq!(calibration.milliamps(4006), 1_020);
        assert_eq!(calibration.millivolts(16000), 20_000);
    }

    #[test]
    fn zero_refusal_stays_and_entry_refusals_keep_the_step() {
        let calibration = Calibration::default();
        let idle = Mean {
            uv: 5_000_000,
            ua: 100_000,
        };
        assert_eq!(calibration.zeroed(idle), Err(Refusal::Load));
        assert_eq!(calibration.current_matched(idle, 101), Err(Refusal::NoLoad));
        assert_eq!(
            calibration.voltage_matched(idle, 5_300),
            Err(Refusal::OutOfRange)
        );
        // A zero-step OK under load stays on the zero step with the
        // refusal (the FNB58 has no skip gesture — unplug and retry).
        let mut wizard = Wizard::new(calibration);
        wizard.ok(Some(idle));
        assert_eq!(wizard.step, Step::Zero);
        assert_eq!(wizard.refusal, Some(Refusal::Load));
        assert_eq!(wizard.pending.zero_ua, 0);
        // Unplugged, the retry takes the zero and moves on.
        wizard.ok(Some(Mean { uv: 0, ua: 500 }));
        assert_eq!(wizard.pending.zero_ua, 500);
        assert!(matches!(wizard.step, Step::Entry { .. }));
        // A no-reading current step refuses and keeps the entry.
        wizard.step(true, false);
        assert_eq!(wizard.ok(None), None);
        assert_eq!(wizard.refusal, Some(Refusal::NoReading));
        // An unadjusted current-step OK skips to the voltage step; the
        // voltage entry starts at this meter's own reading, so walking
        // it past ±5% is what refuses: 5.000 V -> 5.300 V is 6% off.
        let mut wizard = Wizard::new(calibration);
        wizard.ok(Some(Mean { uv: 0, ua: 0 }));
        wizard.ok(Some(idle));
        assert!(matches!(
            wizard.step,
            Step::Entry {
                quantity: Quantity::Voltage,
                ..
            }
        ));
        for _ in 0..30 {
            wizard.step(true, true);
        }
        assert_eq!(wizard.step.entered(), Some(5_300));
        assert_eq!(wizard.ok(Some(idle)), None);
        assert_eq!(wizard.refusal, Some(Refusal::OutOfRange));
    }

    #[test]
    fn records_round_trip_and_stay_apart_from_settings() {
        let calibration = Calibration {
            zero_ua: -2_500,
            current_ppm: 8_000,
            voltage_ppm: -1_000,
        };
        let mut found = Found::default();
        for word in calibration.records() {
            assert!(found.read(word));
            assert_eq!(crate::dim_minutes_from_record(word), None);
        }
        assert_eq!(found.calibration(), calibration);
        // The dim records never read as calibration records.
        assert!(!Found::default().read(0));
        // Out-of-range values are not records this firmware wrote.
        assert!(!Found::default().read(
            0xca << 24 | 0 << 22 | ((MAX_VALUE + 1) as u32 & VALUE_MASK)
        ));
    }
}

//! Ripple statistics, reference-normalized estimates and a mean-triggered trace.
//! Ported from kws-x1 `firmware-core/src/ripple.rs`; deviations are marked
//! with DEVIATION comments and both are hardware-forced (see lib.rs).

use crate::spectrum::Spectrum;

/// Samples per capture, matching the established ADC DMA acquisition.
pub const SAMPLE_COUNT: usize = 1024;
/// Trace columns: two pixels each inside the display's 224-pixel plot.
pub const TRACE_COLUMNS: usize = 112;
/// Most samples a trace shows, leaving 384 for trigger alignment.
pub const MAX_WINDOW: usize = SAMPLE_COUNT * 5 / 8;
/// Eight horizontal divisions across the displayed trace's sample bins.
pub const HORIZONTAL_DIVISIONS: u32 = 8;
/// Maximum count from a right-aligned 12-bit ADC result.
pub const FULL_SCALE: u16 = (1 << 12) - 1;
/// DEVIATION (FNB58): kws-x1 configures the HC32's internal PGA at 2x
/// gain and divides every voltage estimate by it. The AT32F403A has no
/// PGA — PA6 is sampled unity — so the gain is 1 and the same division
/// sites keep their shape. Like kws-x1, the result is deliberately NOT
/// bus-calibrated.
const PGA_GAIN: u32 = 1;
/// Sixteenth-count RMS precision retains sub-count noise without float state.
const RMS_FRACTION: u64 = 16;
/// Four vertical divisions, with the capture mean on the centre grid line.
pub const VERTICAL_DIVISIONS: u32 = 4;
/// Trigger hysteresis as a fraction of the capture's peak-to-peak: after
/// reaching the mean, the signal must rise an eighth of its range above it.
const TRIGGER_BAND_DIVISOR: u16 = 8;
/// Largest step: ±1 V around the mean stays inside the pin's 0..VDDA
/// range with a 3.3 V supply; only a larger one-sided excursion is cut
/// off. (kws-x1's ±1 V budget was for its 1.8 V PGA input range.)
const MAX_MV_PER_DIV: u16 = 500;

/// Volts-per-division choices, in millivolts; transient, never persisted.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Scale {
    #[default]
    Auto,
    Mv2,
    Mv5,
    Mv10,
    Mv20,
    Mv50,
    Mv100,
    Mv200,
    Mv500,
}

impl Scale {
    pub const ALL: [Self; 9] = [
        Self::Auto,
        Self::Mv2,
        Self::Mv5,
        Self::Mv10,
        Self::Mv20,
        Self::Mv50,
        Self::Mv100,
        Self::Mv200,
        Self::Mv500,
    ];

    pub const fn millivolts(self) -> Option<u16> {
        match self {
            Self::Auto => None,
            Self::Mv2 => Some(2),
            Self::Mv5 => Some(5),
            Self::Mv10 => Some(10),
            Self::Mv20 => Some(20),
            Self::Mv50 => Some(50),
            Self::Mv100 => Some(100),
            Self::Mv200 => Some(200),
            Self::Mv500 => Some(MAX_MV_PER_DIV),
        }
    }
}

/// DEVIATION (FNB58): the AT32's ordinary sample-time windows, in whole
/// half-cycles. kws-x1 programs any 20..255 cycle count (HC32 SSTRn);
/// the AT32 offers these eight codes (RM 16.4.8: 1.5/7.5/13.5/28.5/
/// 41.5/55.5/71.5/239.5), so `code` doubles as the SPTx field value.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SampleTime {
    Cycles1_5,
    Cycles7_5,
    Cycles13_5,
    Cycles28_5,
    Cycles41_5,
    Cycles55_5,
    Cycles71_5,
    Cycles239_5,
}

impl SampleTime {
    /// Fastest to slowest — `sampling` walks this order.
    pub const ALL: [Self; 8] = [
        Self::Cycles1_5,
        Self::Cycles7_5,
        Self::Cycles13_5,
        Self::Cycles28_5,
        Self::Cycles41_5,
        Self::Cycles55_5,
        Self::Cycles71_5,
        Self::Cycles239_5,
    ];

    /// Sample window in half ADC clock cycles (1.5 cycles = 3 halves).
    pub const fn half_cycles(self) -> u16 {
        match self {
            Self::Cycles1_5 => 3,
            Self::Cycles7_5 => 15,
            Self::Cycles13_5 => 27,
            Self::Cycles28_5 => 57,
            Self::Cycles41_5 => 83,
            Self::Cycles55_5 => 111,
            Self::Cycles71_5 => 143,
            Self::Cycles239_5 => 479,
        }
    }

    /// The SPTx field value (RM 16.4.8 code order equals ALL's order).
    pub const fn code(self) -> u8 {
        match self {
            Self::Cycles1_5 => 0,
            Self::Cycles7_5 => 1,
            Self::Cycles13_5 => 2,
            Self::Cycles28_5 => 3,
            Self::Cycles41_5 => 4,
            Self::Cycles55_5 => 5,
            Self::Cycles71_5 => 6,
            Self::Cycles239_5 => 7,
        }
    }
}

/// DEVIATION (FNB58): a 12-bit successive-approximation conversion takes
/// 12.5 cycles on the AT32 (RM 16.3.8) where the HC32 takes 13; kept in
/// half-cycles so the rate math stays integer.
const COMPARISON_HALF: u16 = 25;
/// One million µs per second.
const US_PER_SECOND: u64 = 1_000_000;
/// Captures in the smoothed p-p readout, about 3 s at the 100 ms capture
/// cadence. The reference firmware keeps up to 30 (recovered behavior).
const PEAK_HISTORY: usize = 30;
/// Smoothed p-p weights in percent: mean and maximum of the history. The
/// reference firmware's faster-timebase blend (recovered behavior); it uses
/// 75/25 in slower modes.
const PEAK_MEAN_PERCENT: u32 = 65;
const PEAK_MAX_PERCENT: u32 = 35;

/// Horizontal timebase in a scope's 1-2-5 steps; transient, never persisted.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Timebase {
    Us20,
    #[default]
    Us50,
    Us100,
    Us200,
    Us500,
}

/// How a timebase is acquired: ADC sampling window and samples plotted.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Sampling {
    pub sample_time: SampleTime,
    pub window: usize,
}

impl Sampling {
    /// Samples per second at `adc_clock_hz`, for the spectrum's axis.
    /// Doubled so the half-cycle counts divide exactly.
    pub fn rate_hz(self, adc_clock_hz: u32) -> u32 {
        (u64::from(adc_clock_hz) * 2
            / u64::from(self.sample_time.half_cycles() + COMPARISON_HALF)) as u32
    }
}

impl Timebase {
    pub const ALL: [Self; 5] = [
        Self::Us20,
        Self::Us50,
        Self::Us100,
        Self::Us200,
        Self::Us500,
    ];

    pub const fn us_per_div(self) -> u32 {
        match self {
            Self::Us20 => 20,
            Self::Us100 => 100,
            Self::Us50 => 50,
            Self::Us200 => 200,
            Self::Us500 => 500,
        }
    }

    /// The nearest sampling to this timebase at `adc_clock_hz`: the
    /// FASTEST sample window whose rounded whole-screen window fits in
    /// [`MAX_WINDOW`] samples (kws-x1's rule of the smallest cycle count
    /// that fits, applied to the AT32's discrete set), with the window
    /// rounded half-up from the exact screen length. `None` when no
    /// combination reaches a plotable window.
    pub fn sampling(self, adc_clock_hz: u32) -> Option<Sampling> {
        let screen_half = u64::from(self.us_per_div())
            * u64::from(HORIZONTAL_DIVISIONS)
            * u64::from(adc_clock_hz)
            * 2
            / US_PER_SECOND;
        for time in SampleTime::ALL {
            let total = u64::from(time.half_cycles() + COMPARISON_HALF);
            let window = ((screen_half + total / 2) / total) as usize;
            if window <= MAX_WINDOW {
                return (TRACE_COLUMNS..=MAX_WINDOW)
                    .contains(&window)
                    .then_some(Sampling {
                        sample_time: time,
                        window,
                    });
            }
        }
        None
    }
}

/// Mean-centred plot bounds in ADC counts; may extend beyond the ADC rails.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PlotScale {
    pub min: i32,
    pub max: i32,
    /// Missing reference means a raw-count autoscale, never a voltage scale.
    pub mv_per_div: Option<u16>,
    /// The scale cuts off peaks.
    pub limited: bool,
}

/// Preserve the peaks within each displayed bin rather than decimating them.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Envelope {
    pub min: u16,
    pub max: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Reading {
    pub min: u16,
    pub max: u16,
    pub mean: u16,
    /// RMS after exact DC removal, in sixteenths of an ADC count.
    rms_counts_x16: u32,
    /// Peak-to-peak counts blended over recent captures; see [`PeakHistory`].
    smoothed: u16,
    pub triggered: bool,
    /// The capture's timebase, so a held trace keeps its own label.
    pub timebase: Timebase,
    pub trace: [Envelope; TRACE_COLUMNS],
    /// The whole capture's spectrum, set by the caller ([`Spectrum::new`]).
    pub spectrum: Spectrum,
}

impl Reading {
    /// Peak-to-peak counts over the entire capture, not only the trace window.
    pub fn peak_to_peak(&self) -> u16 {
        self.max - self.min
    }

    /// Smoothed peak-to-peak as estimated mV from the shared ADC reference,
    /// rounding only the final input-equivalent ratio rather than truncating
    /// an intermediate voltage.
    /// `reference_mv` is the application's internal-reference calibration.
    /// No estimate is available with a zero/saturated reference or overflow.
    pub fn peak_to_peak_mv(&self, reference_count: u16, reference_mv: u16) -> Option<u16> {
        if reference_count == 0 || reference_count >= FULL_SCALE || reference_mv == 0 {
            return None;
        }
        let numerator = u64::from(self.smoothed) * u64::from(reference_mv);
        let denominator = u64::from(reference_count) * u64::from(PGA_GAIN);
        u16::try_from((numerator + denominator / 2) / denominator).ok()
    }

    /// DC-removed ripple RMS in tenths of a millivolt, preserving fractional
    /// counts before rounding the reference-normalized ratio.
    pub fn rms_tenths_mv(&self, reference_count: u16, reference_mv: u16) -> Option<u16> {
        if reference_count == 0 || reference_count >= FULL_SCALE || reference_mv == 0 {
            return None;
        }
        // Ten tenths per mV; u64 also covers the largest u16 reference value.
        let numerator = u64::from(self.rms_counts_x16) * u64::from(reference_mv) * 10;
        let denominator = RMS_FRACTION * u64::from(reference_count) * u64::from(PGA_GAIN);
        u16::try_from((numerator + denominator / 2) / denominator).ok()
    }

    /// Auto picks the smallest step that fits the larger excursion from the mean.
    pub fn plot_scale(&self, scale: Scale, reference: Option<u16>, reference_mv: u16) -> PlotScale {
        let excursion = u32::from((self.max - self.mean).max(self.mean - self.min)).max(1);
        let mv_per_div = reference
            .filter(|count| *count > 0 && *count < FULL_SCALE)
            .filter(|_| reference_mv > 0)
            .map(|reference| {
                let needed = (excursion * u32::from(reference_mv))
                    .div_ceil(PGA_GAIN * u32::from(reference) * (VERTICAL_DIVISIONS / 2));
                scale.millivolts().unwrap_or_else(|| {
                    Scale::ALL
                        .into_iter()
                        .filter_map(Scale::millivolts)
                        .find(|mv| u32::from(*mv) >= needed)
                        .unwrap_or(MAX_MV_PER_DIV)
                })
            });
        let half_span = match (mv_per_div, reference) {
            (Some(mv), Some(reference)) => {
                (u32::from(mv) * (VERTICAL_DIVISIONS / 2) * PGA_GAIN * u32::from(reference))
                    .div_ceil(u32::from(reference_mv))
                    .max(1)
            }
            _ => excursion,
        } as i32;
        let min = i32::from(self.mean) - half_span;
        let max = i32::from(self.mean) + half_span;
        PlotScale {
            min,
            max,
            mv_per_div,
            limited: i32::from(self.min) < min || i32::from(self.max) > max,
        }
    }

    /// A rail hit means peak-to-peak may underestimate the actual input.
    pub fn clipped(&self) -> bool {
        self.min == 0 || self.max >= FULL_SCALE
    }
}

/// Rolling peak-to-peak history behind the smoothed readout, so one noisy
/// capture does not make the number jump. The caller clears it when the
/// sample rate changes or capture pauses.
#[derive(Clone, Copy, Debug, Default)]
pub struct PeakHistory {
    values: [u16; PEAK_HISTORY],
    len: usize,
    next: usize,
}

impl PeakHistory {
    /// Add the capture's peak-to-peak and set its smoothed value: the
    /// weighted mean and maximum of the history, rounded.
    pub fn add(&mut self, reading: &mut Reading) {
        self.values[self.next] = reading.peak_to_peak();
        self.next = (self.next + 1) % PEAK_HISTORY;
        self.len = (self.len + 1).min(PEAK_HISTORY);
        let values = &self.values[..self.len];
        let sum: u32 = values.iter().map(|&value| u32::from(value)).sum();
        let max = values.iter().copied().max().map_or(0, u32::from);
        let len = self.len as u32;
        let weighted = PEAK_MEAN_PERCENT * sum + PEAK_MAX_PERCENT * max * len;
        let total = (PEAK_MEAN_PERCENT + PEAK_MAX_PERCENT) * len;
        reading.smoothed = ((weighted + total / 2) / total) as u16;
    }

    pub fn clear(&mut self) {
        *self = Self::default();
    }
}

/// Statistics over the whole capture, trace over the first `window`
/// samples from the trigger ([`Sampling::window`]).
pub fn analyze(samples: &[u16; SAMPLE_COUNT], timebase: Timebase, window: usize) -> Reading {
    let window = window.clamp(TRACE_COLUMNS, MAX_WINDOW);
    let mut min = u16::MAX;
    let mut max = 0;
    let mut sum = 0u32;
    let mut square_sum = 0u64;
    for &sample in samples {
        min = min.min(sample);
        max = max.max(sample);
        sum += u32::from(sample);
        square_sum += u64::from(sample).pow(2);
    }
    // N*sum(x²) - sum(x)² removes the exact mean before taking the root.
    // Cauchy-Schwarz guarantees nonnegative variance. Even u16 samples at
    // this fixed capture length fit u64, including the fractional scale.
    let variance = SAMPLE_COUNT as u64 * square_sum - u64::from(sum).pow(2);
    let rms_counts_x16 = ((variance * RMS_FRACTION.pow(2)).isqrt() / SAMPLE_COUNT as u64) as u32;
    let mean = (sum / SAMPLE_COUNT as u32) as u16;
    // Rising crossing only, and only where a whole trace window remains.
    // Never wrap the capture: its end and start are not adjacent in time.
    // Arm at or below the mean, fire above it plus the hysteresis band.
    let level = mean + (max - min) / TRIGGER_BAND_DIVISOR;
    let mut armed = false;
    let trigger = samples[..=SAMPLE_COUNT - window]
        .iter()
        .position(|&sample| {
            let fired = armed && sample > level;
            armed |= sample <= mean;
            fired
        });
    let start = trigger.unwrap_or(0);
    let trace = core::array::from_fn(|column| {
        let first = start + column * window / TRACE_COLUMNS;
        let end = start + (column + 1) * window / TRACE_COLUMNS;
        let mut bin = Envelope {
            min: u16::MAX,
            max: 0,
        };
        for &sample in &samples[first..end] {
            bin.min = bin.min.min(sample);
            bin.max = bin.max.max(sample);
        }
        bin
    });
    Reading {
        min,
        max,
        mean,
        rms_counts_x16,
        smoothed: max - min,
        triggered: trigger.is_some(),
        timebase,
        trace,
        spectrum: Spectrum::EMPTY,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A half-capture trace window, which these cases were written for.
    const WINDOW: usize = SAMPLE_COUNT / 2;

    /// The FNB58's ADC clock: PCLK2 96 MHz / 8 (adc.rs ADCDIV).
    const CLOCK_HZ: u32 = 12_000_000;

    #[test]
    fn timebases_stay_near_their_label_within_adc_limits() {
        // The FNB58 clock must land each 1-2-5 step within 1% using only
        // the AT32's discrete sampling windows and plottable windows.
        for timebase in Timebase::ALL {
            let Some(sampling) = timebase.sampling(CLOCK_HZ) else {
                panic!("{timebase:?} unreachable at {CLOCK_HZ} Hz");
            };
            assert_eq!(
                sampling.sample_time.code(),
                SampleTime::ALL
                    .iter()
                    .position(|&time| time == sampling.sample_time)
                    .unwrap() as u8
            );
            assert!((TRACE_COLUMNS..=MAX_WINDOW).contains(&sampling.window));
            let total = u64::from(sampling.sample_time.half_cycles() + COMPARISON_HALF);
            let screen_us = (sampling.window as u64 * total * US_PER_SECOND) as f64
                / (f64::from(CLOCK_HZ) * 2.0);
            let label = f64::from(timebase.us_per_div() * HORIZONTAL_DIVISIONS);
            assert!((screen_us / label - 1.0).abs() < 0.01, "{timebase:?}");
        }
        assert_eq!(Timebase::Us20.sampling(0), None);
        // A clock so slow even 239.5 cycles overshoots MAX_WINDOW: none.
        assert_eq!(Timebase::Us500.sampling(1), None);
    }

    #[test]
    fn smoothed_peak_to_peak_blends_history_mean_and_maximum() {
        // One spiky capture lifts the readout without replacing it, and the
        // spike ages out after a full history instead of sticking.
        let mut history = PeakHistory::default();
        let capture = |peak: u16| {
            let mut samples = [1000; SAMPLE_COUNT];
            samples[0] += peak;
            analyze(&samples, Timebase::Us50, WINDOW)
        };
        let mut reading = capture(100);
        history.add(&mut reading);
        assert_eq!(reading.smoothed, 100);
        let mut reading = capture(200);
        history.add(&mut reading);
        // 0.65 * 150 + 0.35 * 200 = 167.5, rounded half-up.
        assert_eq!(reading.smoothed, 168);
        for _ in 0..PEAK_HISTORY {
            reading = capture(100);
            history.add(&mut reading);
        }
        assert_eq!(reading.smoothed, 100);
        history.clear();
        let mut reading = capture(40);
        history.add(&mut reading);
        assert_eq!(reading.smoothed, 40);
    }

    #[test]
    fn voltage_estimate_rounds_reference_ratio_and_rejects_bad_reference() {
        // Protect small-signal precision and final half-up rounding: early
        // truncation used to discard nonzero ranges of up to four counts.
        // Expectations recomputed for PGA_GAIN 1 (the FNB58 deviation).
        const REFERENCE_MV: u16 = 1083;
        let mut samples = [0; SAMPLE_COUNT];
        for (counts, reference, expected) in [
            (0, 1344, 0),
            // One count is 0.806 mV at this reference: rounds up to 1
            // (kws-x1's 2x gain halved it below the 0.5 mV threshold).
            (1, 1344, 1),
            (1, 1083, 1),
            (4, 1344, 3),
            (5, 1344, 4),
            (159, 1344, 128),
            (159, 1200, 143),
            (159, 1500, 115),
            (1024, 1344, 825),
            (FULL_SCALE, 1344, 3300),
        ] {
            samples[0] = counts;
            let reading = analyze(&samples, Timebase::Us50, WINDOW);
            assert_eq!(
                reading.peak_to_peak_mv(reference, REFERENCE_MV),
                Some(expected)
            );
            assert_eq!(reading.peak_to_peak_mv(0, REFERENCE_MV), None);
            assert_eq!(reading.peak_to_peak_mv(FULL_SCALE, REFERENCE_MV), None);
            assert_eq!(reading.peak_to_peak_mv(reference, 0), None);
        }
        let reading = analyze(&samples, Timebase::Us50, WINDOW);
        assert_eq!(reading.peak_to_peak_mv(1, REFERENCE_MV), None);
    }

    #[test]
    fn rms_removes_dc_and_preserves_fractional_mean() {
        // Tenths of mV at unity gain: 10 counts RMS is ~8.06 mV.
        for baseline in [1000, 2000] {
            let mut samples = [baseline; SAMPLE_COUNT];
            samples[SAMPLE_COUNT / 2..].fill(baseline + 20);
            let reading = analyze(&samples, Timebase::Us50, WINDOW);
            assert_eq!(reading.rms_tenths_mv(1344, 1083), Some(81));
            assert_eq!(reading.rms_tenths_mv(1500, 1083), Some(72));
            assert_eq!(reading.rms_tenths_mv(0, 1083), None);
            assert_eq!(reading.rms_tenths_mv(FULL_SCALE, 1083), None);
            assert_eq!(reading.rms_tenths_mv(1344, 0), None);
        }
        let mut samples = [2047; SAMPLE_COUNT];
        samples[SAMPLE_COUNT / 2..].fill(2048);
        assert_eq!(
            analyze(&samples, Timebase::Us50, WINDOW).rms_tenths_mv(1344, 1083),
            Some(4)
        );
        samples[..SAMPLE_COUNT / 2].fill(0);
        samples[SAMPLE_COUNT / 2..].fill(FULL_SCALE);
        let reading = analyze(&samples, Timebase::Us50, WINDOW);
        // RMS 2047.5 counts = 1649.88 mV: 16,499 tenths after the
        // half-up rounding of the exact integer ratio.
        assert_eq!(reading.rms_tenths_mv(1344, 1083), Some(16_499));
        assert_eq!(reading.rms_tenths_mv(1, 1083), None);
    }

    #[test]
    fn plot_scales_preserve_spikes_and_flag_range_limits() {
        let mut samples = [1000; SAMPLE_COUNT];
        samples[SAMPLE_COUNT - 1] = 1100;
        let reading = analyze(&samples, Timebase::Us50, WINDOW);
        let auto = reading.plot_scale(Scale::Auto, Some(1344), 1083);
        assert_eq!(auto.mv_per_div, Some(50));
        assert!(auto.min <= i32::from(reading.min));
        assert!(auto.max >= i32::from(reading.max));
        assert!(!auto.limited);
        let locked = reading.plot_scale(Scale::Mv2, Some(1344), 1083);
        assert_eq!(locked.mv_per_div, Some(2));
        assert!(locked.limited);
        // A missing/invalid reference cannot leave a voltage label on a raw
        // plot, even when the user selected a fixed mV range.
        for reference in [None, Some(0), Some(FULL_SCALE)] {
            let raw = reading.plot_scale(Scale::Mv2, reference, 1083);
            assert_eq!(raw.mv_per_div, None);
            assert!(!raw.limited);
        }
        assert_eq!(
            reading.plot_scale(Scale::Mv2, Some(1344), 0).mv_per_div,
            None
        );
        samples.fill(0);
        samples[0] = FULL_SCALE;
        let reading = analyze(&samples, Timebase::Us50, WINDOW);
        let auto = reading.plot_scale(Scale::Auto, Some(1344), 1083);
        assert_eq!(auto.mv_per_div, Some(500));
        assert!(auto.limited);
    }

    #[test]
    fn flat_capture_has_no_trigger_or_invented_ripple() {
        let reading = analyze(&[2048; SAMPLE_COUNT], Timebase::Us50, WINDOW);
        assert_eq!((reading.min, reading.max, reading.mean), (2048, 2048, 2048));
        assert_eq!(reading.peak_to_peak(), 0);
        assert_eq!(reading.rms_tenths_mv(1344, 1083), Some(0));
        let plot = reading.plot_scale(Scale::Auto, Some(1344), 1083);
        assert!(plot.min < i32::from(reading.mean) && plot.max > i32::from(reading.mean));
        assert!(!plot.limited);
        assert!(!reading.triggered);
        assert!(!reading.clipped());
        assert!(reading.trace.iter().all(|bin| *bin
            == Envelope {
                min: 2048,
                max: 2048
            }));
    }

    #[test]
    fn trigger_window_preserves_peaks_without_wrapping() {
        let mut samples = [1000; SAMPLE_COUNT];
        samples[10..20].fill(3000);
        samples[SAMPLE_COUNT - 1] = FULL_SCALE;
        let reading = analyze(&samples, Timebase::Us50, WINDOW);
        assert_eq!(
            reading.mean,
            (samples.iter().map(|&v| u32::from(v)).sum::<u32>() / SAMPLE_COUNT as u32) as u16
        );
        assert!(reading.triggered);
        assert_eq!(reading.trace[0].max, 3000);
        assert_eq!(reading.trace.last().map(|bin| bin.max), Some(1000));
        assert!(reading.trace.iter().all(|bin| bin.max <= 3000));
        assert_eq!(reading.peak_to_peak(), FULL_SCALE - 1000);
        assert!(reading.clipped());
    }

    #[test]
    fn noise_at_the_mean_does_not_trigger_before_the_real_edge() {
        // Without hysteresis the noise below crossed the mean at sample 1
        // and the trace jumped between captures.
        let mut samples = [1000; SAMPLE_COUNT];
        samples[WINDOW..].fill(1100);
        for (index, sample) in samples[..100].iter_mut().enumerate() {
            *sample = if index % 2 == 0 { 1053 } else { 1056 };
        }
        let reading = analyze(&samples, Timebase::Us50, WINDOW);
        assert_eq!(reading.mean, 1055);
        assert!(reading.triggered);
        assert_eq!(
            reading.trace[0],
            Envelope {
                min: 1100,
                max: 1100
            }
        );
    }

    #[test]
    fn late_crossing_uses_untriggered_window_and_bins_keep_narrow_peaks() {
        let mut samples = [2000; SAMPLE_COUNT];
        samples[WINDOW + 1] = 4000;
        let reading = analyze(&samples, Timebase::Us50, WINDOW);
        assert!(!reading.triggered);
        assert!(reading.trace.iter().all(|bin| bin.max == 2000));
        // A one-sample peak must survive binning when it is in the window.
        samples[2] = 4000;
        let reading = analyze(&samples, Timebase::Us50, WINDOW);
        assert!(reading.triggered);
        assert_eq!(
            reading.trace[0],
            Envelope {
                min: 2000,
                max: 4000
            }
        );
    }
}

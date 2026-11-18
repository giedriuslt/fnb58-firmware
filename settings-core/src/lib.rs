//! Settings-store record format (r218): a port of kws-x1's
//! `firmware/src/settings_store.rs` rotating-record scheme — `"KWST"`
//! claim word, 8-byte `[word, !word]` records appended after the claim,
//! newest valid record wins, a nearly-full sector is padded with
//! `FILLER` words before moving to the next claimed sector (so only the
//! newest claimed sector ever has room). Pure logic: codecs, record
//! validity and the sector scan fold; the flash access itself is the
//! board layer's (`firmware/src/settings.rs`).
//!
//! r221: calibration rides the same records — one per field, kws-x1's
//! tag/field/value codec (module [`calibration`]).
//!
//! Ported deviation (hardware-forced, documented in the board layer):
//! kws-x1 never erases (their internal flash starts blank); the FNB58's
//! W25Q128 carries the stock firmware's littlefs, so claiming a sector
//! erases it first. The record format is unchanged.

#![cfg_attr(not(test), no_std)]

pub mod calibration;
pub mod energy;
pub mod energy_store;
pub mod power_history;

use calibration::Found;

/// First word of a sector claimed for settings ("KWST" — kws-x1's
/// claim, kept verbatim; the diagnostic log claims other words and
/// never takes this one).
pub const CLAIM: u32 = 0x4b57_5354;
/// Record size in bytes: the settings word, then its complement.
pub const RECORD: usize = 8;
/// First record's offset in a claimed sector, after the claim word.
pub const FIRST: usize = 8;
/// Pads a nearly-full sector before the store moves on: a word whose
/// check would not be its complement — every scan skips it.
pub const FILLER: u32 = 0;

/// A record is valid when the check word is the complement of the data
/// word and neither is blank (an interrupted write fails the check and
/// is skipped by the scan; blank pairs END the scan instead).
pub const fn valid(word: u32, check: u32) -> bool {
    word != BLANK && check == !word
}

/// The erased state of NOR flash.
pub const BLANK: u32 = 0xFFFF_FFFF;

/// The stored word for `minutes`, or `None` when the value is not one
/// this build knows (unknown records are skipped, so older builds
/// tolerate newer fields in other bits).
pub fn dim_minutes_to_record(minutes: u32) -> Option<u32> {
    match minutes {
        0 | 1 | 2 | 5 | 10 => Some(minutes & 0xFF),
        _ => None,
    }
}

/// Decode a record word's dim-minutes setting: the low byte, valid
/// values only. r226: the bits above now carry the energy-memory index
/// (the r218 reservation taken) — they are ignored here.
pub fn dim_minutes_from_record(word: u32) -> Option<u32> {
    match word & 0xFF {
        0 | 1 | 2 | 5 | 10 => Some(word & 0xFF),
        _ => None,
    }
}

/// r226: energy checkpoint interval (kws-x1's EnergyMemory). Off never
/// writes totals and starts each boot from zero; the other choices
/// trade flash endurance for less power-loss lag.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum EnergyMemory {
    #[default]
    Off,
    Seconds5,
    Seconds10,
    Seconds30,
    Seconds60,
}

impl EnergyMemory {
    pub const ALL: [Self; 5] = [
        Self::Off,
        Self::Seconds5,
        Self::Seconds10,
        Self::Seconds30,
        Self::Seconds60,
    ];

    pub fn seconds(self) -> Option<u64> {
        match self {
            Self::Off => None,
            Self::Seconds5 => Some(5),
            Self::Seconds10 => Some(10),
            Self::Seconds30 => Some(30),
            Self::Seconds60 => Some(60),
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Self::Off => "OFF",
            Self::Seconds5 => "5S",
            Self::Seconds10 => "10S",
            Self::Seconds30 => "30S",
            Self::Seconds60 => "60S",
        }
    }
}

/// The settings record word: the dim minutes in the low byte, the
/// energy-memory index in bits 8..10 (kws-x1 packs its settings word
/// as bitfields; the r218 word's reserved bits were held for exactly
/// this) and the PD passthrough flag in bit 11 (r233). Callers validate
/// `minutes` via [`dim_minutes_to_record`].
pub fn settings_word(minutes: u32, energy_idx: usize, passthrough: bool) -> u32 {
    (minutes & 0xFF) | ((energy_idx as u32 & 0x7) << 8) | (u32::from(passthrough) << 11)
}

/// Decode a record word's energy-memory index (bits 8..10, valid
/// indices only).
pub fn energy_memory_from_record(word: u32) -> Option<usize> {
    let idx = (word >> 8 & 0x7) as usize;
    if idx < EnergyMemory::ALL.len() { Some(idx) } else { None }
}

/// Decode a record word's PD passthrough flag (bit 11).
pub fn passthrough_from_record(word: u32) -> bool {
    word >> 11 & 1 == 1
}

/// The scan fold's carried findings: the newest dim minutes, the newest
/// energy-memory index and the newest calibration fields.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ScanState {
    pub dim: Option<u32>,
    pub energy: Option<usize>,
    pub passthrough: bool,
    pub cal: Found,
}

/// The scan fold over one claimed sector's record words (later records
/// win). Every valid non-calibration word decodes BOTH settings fields
/// (kws-x1's scan: the calibration reader first, then the settings
/// word — one word is one settings snapshot); returns the updated state
/// and the byte offset of the next free record slot, or `None` when the
/// sector is full (`slot + RECORD > end`). Blank pair = end of records.
pub fn scan_records(
    words: impl Iterator<Item = (u32, u32)>,
    mut state: ScanState,
    end: usize,
) -> (ScanState, Option<usize>) {
    let mut slot = FIRST;
    for (word, check) in words {
        if word == BLANK && check == BLANK {
            return (state, Some(slot));
        }
        if valid(word, check) {
            if !state.cal.read(word) {
                // kws-x1's fold (settings_store.rs:67): the LATER record
                // replaces — records iterate oldest-to-newest, and claims
                // advance 0,1,... (r220) — a later-claimed (newer) sector
                // scanned later overrides an earlier one, exactly the
                // "only the newest claimed sector has room" invariant.
                if let Some(minutes) = dim_minutes_from_record(word) {
                    state.dim = Some(minutes);
                }
                if let Some(idx) = energy_memory_from_record(word) {
                    state.energy = Some(idx);
                }
                state.passthrough = passthrough_from_record(word);
            }
        }
        slot += RECORD;
        if slot + RECORD > end {
            return (state, None);
        }
    }
    (state, None)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn record_validity_distinguishes_blank_filler_and_complement() {
        assert!(valid(0x1234_5678, !0x1234_5678));
        // Blank pair is the scan terminator, not a record.
        assert!(!valid(BLANK, BLANK));
        // A FILLER pair is (0, 0) — the check is not the complement, so
        // every scan skips it (kws-x1's filler trick).
        assert!(!valid(FILLER, FILLER));
        // An interrupted write fails the complement check.
        assert!(!valid(0x1234_5678, !0x1234_5678 ^ 1));
        // The claim pair passes valid() mathematically but is never
        // SCANNED: records start at FIRST, after the claim word (the
        // kws-x1 structure, kept verbatim).
        assert!(valid(CLAIM, !CLAIM));
    }

    #[test]
    fn dim_minutes_codec_roundtrips_known_values_only() {
        for minutes in [0u32, 1, 2, 5, 10] {
            let word = dim_minutes_to_record(minutes).unwrap();
            assert_eq!(dim_minutes_from_record(word), Some(minutes));
        }
        assert_eq!(dim_minutes_to_record(3), None);
        assert_eq!(dim_minutes_to_record(60), None);
        // Unknown values are skipped by the scan; r226: the high bits now
        // carry the energy index and no longer reject the word.
        assert_eq!(dim_minutes_from_record(3), None);
        assert_eq!(dim_minutes_from_record(0x100), Some(0));
    }

    /// r226: the settings word packs dim + energy; both decode from one
    /// record (one word = one settings snapshot, later records win).
    #[test]
    fn settings_word_packs_dim_and_energy() {
        for (minutes, idx) in [(0u32, 0usize), (10, 4), (5, 2)] {
            let word = settings_word(minutes, idx, false);
            assert_eq!(dim_minutes_from_record(word), Some(minutes));
            assert_eq!(energy_memory_from_record(word), Some(idx));
        }
        // An invalid energy index decodes as nothing; dim survives.
        assert_eq!(energy_memory_from_record(settings_word(5, 6, false)), None);
        // The old single-field words read as energy=Off (index 0).
        assert_eq!(energy_memory_from_record(5), Some(0));
    }

    #[test]
    fn scan_wins_latest_and_stops_at_blank() {
        let dim = dim_minutes_to_record(2).unwrap();
        let dim5 = dim_minutes_to_record(5).unwrap();
        // words: dim(2), filler, dim(5), blank, blank... — the LATER
        // (newer) record wins (kws-x1's decode.or fold).
        let records = [
            (dim, !dim),
            (FILLER, FILLER),
            (dim5, !dim5),
            (BLANK, BLANK),
            (BLANK, BLANK),
        ];
        let (state, next) =
            scan_records(records.into_iter(), ScanState::default(), FIRST + records.len() * RECORD);
        assert_eq!(state.dim, Some(5));
        assert_eq!(state.energy, Some(0));
        assert_eq!(next, Some(FIRST + 3 * RECORD));
        // Full sector: no blank pair before the end.
        let records = [(dim, !dim), (dim5, !dim5)];
        let (state, next) =
            scan_records(records.into_iter(), ScanState::default(), FIRST + records.len() * RECORD);
        assert_eq!(state.dim, Some(5));
        assert_eq!(next, None);
        // Empty sector: blank at the first slot.
        let (state, next) =
            scan_records([(BLANK, BLANK), (BLANK, BLANK)].into_iter(), ScanState::default(), 32);
        assert_eq!(state.dim, None);
        assert_eq!(next, Some(FIRST));
        // Interrupted writes and garbage are skipped.
        let records = [(0xABCD, BLANK), (dim, !dim), (BLANK, BLANK)];
        let (state, _) =
            scan_records(records.into_iter(), ScanState::default(), FIRST + records.len() * RECORD);
        assert_eq!(state.dim, Some(2));
        // Across sectors (claims advance 0,1,...: later-claimed sectors
        // hold newer records) — the accumulated fold lets the later scan
        // override: the r218 hardware bug was first-wins keeping the
        // stale record from the same sector.
        let (state, _) = scan_records([(dim, !dim)].into_iter(), ScanState { dim: Some(10), ..ScanState::default() }, 32);
        assert_eq!(state.dim, Some(2));
        // A packed dim+energy word updates both fields.
        let word = settings_word(5, 3, true);
        let (state, _) =
            scan_records([(word, !word)].into_iter(), ScanState::default(), FIRST + RECORD);
        assert_eq!(state.dim, Some(5));
        assert_eq!(state.energy, Some(3));
        assert!(state.passthrough);
    }

    /// r221: calibration records ride the same scan — a calibration word
    /// feeds the calibration reader (later records win), never the dim
    /// decode, and a newer claim's records override older ones.
    #[test]
    fn scan_reads_calibration_records_too() {
        use crate::calibration::Calibration;
        let dim = dim_minutes_to_record(5).unwrap();
        let cal = Calibration {
            zero_ua: -2_500,
            current_ppm: 8_000,
            voltage_ppm: -1_000,
        };
        let records: Vec<(u32, u32)> = cal
            .records()
            .iter()
            .map(|&w| (w, !w))
            .chain([(dim, !dim)])
            .collect();
        let (state, _) =
            scan_records(records.into_iter(), ScanState::default(), FIRST + 4 * RECORD);
        assert_eq!(state.dim, Some(5));
        assert_eq!(state.cal.calibration(), cal);
        // A newer (later-scanned) calibration record overrides.
        let newer = Calibration {
            zero_ua: 100,
            ..cal
        };
        let records: Vec<(u32, u32)> = cal
            .records()
            .iter()
            .map(|&w| (w, !w))
            .chain(newer.records().iter().map(|&w| (w, !w)))
            .collect();
        let (state, _) =
            scan_records(records.into_iter(), ScanState::default(), FIRST + 6 * RECORD);
        assert_eq!(state.cal.calibration(), newer);
        // Carried across sectors: a later scan merges over the earlier
        // findings (the zero field only in the older sector survives).
        let (state, _) = scan_records(
            [(dim, !dim)].into_iter(),
            state,
            FIRST + RECORD,
        );
        assert_eq!(state.dim, Some(5));
        assert_eq!(state.cal.calibration(), newer);
    }
}

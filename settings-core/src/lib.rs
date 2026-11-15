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

/// Decode a record word's dim-minutes setting. Bits above the low byte
/// must be zero (reserved for future settings); an unknown low byte is
/// not a dim record.
pub fn dim_minutes_from_record(word: u32) -> Option<u32> {
    if word >> 8 != 0 {
        return None;
    }
    match word & 0xFF {
        0 | 1 | 2 | 5 | 10 => Some(word & 0xFF),
        _ => None,
    }
}

/// The scan fold over one claimed sector's record words (later records
/// win). `dim` carries the newest dim value so far and `cal` the newest
/// calibration fields (kws-x1's scan: a valid word feeds the calibration
/// reader first, and only a non-calibration word decodes as the settings
/// word); returns the updated pair and the byte offset of the next free
/// record slot, or `None` when the sector is full (`slot + RECORD >
/// end`). Blank pair = end of records.
pub fn scan_records(
    words: impl Iterator<Item = (u32, u32)>,
    mut dim: Option<u32>,
    mut cal: Found,
    end: usize,
) -> (Option<u32>, Found, Option<usize>) {
    let mut slot = FIRST;
    for (word, check) in words {
        if word == BLANK && check == BLANK {
            return (dim, cal, Some(slot));
        }
        if valid(word, check) {
            if !cal.read(word) {
                if let Some(minutes) = dim_minutes_from_record(word) {
                    // kws-x1's fold (settings_store.rs:67): decode(word).or(found)
                    // — the LATER record replaces, so the newest valid record
                    // in the scan wins. Records iterate oldest-to-newest, and
                    // claims advance 0,1,...,19 (kws-x1's
                    // low-end-first direction, r220) — a later-claimed (newer)
                    // sector scanned later overrides an earlier one, exactly
                    // the "only the newest claimed sector has room" invariant.
                    dim = Some(minutes);
                }
            }
        }
        slot += RECORD;
        if slot + RECORD > end {
            return (dim, cal, None);
        }
    }
    (dim, cal, None)
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
        // Unknown values and reserved bits are skipped by the scan.
        assert_eq!(dim_minutes_from_record(3), None);
        assert_eq!(dim_minutes_from_record(0x100), None);
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
        let (found, _, next) =
            scan_records(records.into_iter(), None, Found::default(), FIRST + records.len() * RECORD);
        assert_eq!(found, Some(5));
        assert_eq!(next, Some(FIRST + 3 * RECORD));
        // Full sector: no blank pair before the end.
        let records = [(dim, !dim), (dim5, !dim5)];
        let (found, _, next) =
            scan_records(records.into_iter(), None, Found::default(), FIRST + records.len() * RECORD);
        assert_eq!(found, Some(5));
        assert_eq!(next, None);
        // Empty sector: blank at the first slot.
        let (found, _, next) =
            scan_records([(BLANK, BLANK), (BLANK, BLANK)].into_iter(), None, Found::default(), 32);
        assert_eq!(found, None);
        assert_eq!(next, Some(FIRST));
        // Interrupted writes and garbage are skipped.
        let records = [(0xABCD, BLANK), (dim, !dim), (BLANK, BLANK)];
        let (found, _, _) =
            scan_records(records.into_iter(), None, Found::default(), FIRST + records.len() * RECORD);
        assert_eq!(found, Some(2));
        // Across sectors (claims advance 0,1,...: later-claimed sectors
        // hold newer records) — the accumulated fold lets the later scan
        // override: the r218 hardware bug was first-wins keeping the
        // stale record from the same sector.
        let (found, _, _) = scan_records([(dim, !dim)].into_iter(), Some(10), Found::default(), 32);
        assert_eq!(found, Some(2));
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
        let (dim_found, cal_found, _) =
            scan_records(records.into_iter(), None, Found::default(), FIRST + 4 * RECORD);
        assert_eq!(dim_found, Some(5));
        assert_eq!(cal_found.calibration(), cal);
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
        let (_, cal_found, _) =
            scan_records(records.into_iter(), None, Found::default(), FIRST + 6 * RECORD);
        assert_eq!(cal_found.calibration(), newer);
        // Carried across sectors: a later scan merges over the earlier
        // findings (the zero field only in the older sector survives).
        let (dim_found, cal_found, _) = scan_records(
            [(dim, !dim)].into_iter(),
            None,
            cal_found,
            FIRST + RECORD,
        );
        assert_eq!(dim_found, Some(5));
        assert_eq!(cal_found.calibration(), newer);
    }
}

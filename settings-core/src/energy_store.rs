//! r226: the two-sector energy journal — kws-x1's
//! `firmware-core/energy_store.rs` durability core (sequence +
//! word-wise complements + a final COMMIT word per record, power-cut
//! recovery by scanning past torn records, erase-the-partner-then-claim
//! rollover) onto the W25Q128's 4 KiB sectors.
//!
//! Two documented deviations:
//! - the flash backend is PASSED PER CALL instead of owned by the store
//!   ([`Store`] keeps only the cursor state): the FNB58's W25Q128 is
//!   shared between the settings store, this journal and the console's
//!   sector dumps, so the journal cannot hold it;
//! - kws-x1's legacy-log migration and previous-format ("EN01")
//!   carrying are dropped — no such history exists on this tester, and
//!   the pair lives in the firmware's own directed 20-sector region
//!   (sectors 18/19), so `prepare` reclaims unclaimed content.

use crate::energy::Totals;

/// Backend sector size in bytes (the W25Q128's 4 KiB sectors).
pub const SECTOR_BYTES: usize = 4096;
/// Pair ownership/version marker ("EN02"), followed by its complement.
const CLAIM: u32 = 0x454e_3032;
const HEADER_BYTES: usize = 8;
/// Sequence plus three u64 counters (charge, energy, elapsed time), their
/// word-wise complements and a final nonblank commit word. Complements alone
/// can equal erased words, so they cannot identify the last programmed word
/// after a power cut.
const DATA_WORDS: usize = 7;
const RECORD_WORDS: usize = DATA_WORDS * 2 + 1;
/// Final record commit marker ("DONE"), written after all counter words.
const COMMIT: u32 = 0x444f_4e45;
const RECORD_BYTES: usize = RECORD_WORDS * 4;

/// The journal's two-sector view of the board's flash (the board layer
/// maps sector 0/1 onto its reserved pair).
pub trait Storage {
    fn read(&mut self, sector: usize, offset: usize) -> u32;
    fn write(&mut self, sector: usize, offset: usize, words: &[u32]) -> bool;
    fn erase(&mut self, sector: usize) -> bool;
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Preparation {
    Ready,
    WriteFailed,
}

/// Cursor state only — the flash backend is passed per call.
#[derive(Clone, Copy, Debug)]
pub struct Store {
    current: usize,
    next: usize,
    sequence: u32,
    saved: Option<Totals>,
    claimed: bool,
    failed: bool,
}

fn header(flash: &mut impl Storage, sector: usize, claim: u32) -> bool {
    flash.read(sector, 0) == claim && flash.read(sector, 4) == !claim
}

fn blank(flash: &mut impl Storage, sector: usize) -> bool {
    (0..SECTOR_BYTES)
        .step_by(4)
        .all(|offset| flash.read(sector, offset) == u32::MAX)
}

/// The record at `offset`, complete only when its COMMIT word is present
/// and every counter word's complement matches.
fn record(flash: &mut impl Storage, sector: usize, offset: usize) -> Option<(u32, Totals)> {
    let mut words = [0u32; DATA_WORDS * 2 + 1];
    for (i, w) in words.iter_mut().enumerate() {
        *w = flash.read(sector, offset + i * 4);
    }
    if words[DATA_WORDS * 2] != COMMIT
        || (0..DATA_WORDS).any(|i| words[i + DATA_WORDS] != !words[i])
    {
        return None;
    }
    let wide = |i: usize| u64::from(words[i]) | u64::from(words[i + 1]) << 32;
    Some((
        words[0],
        Totals {
            ma_ms: wide(1),
            uw_ms: wide(3),
            elapsed_ms: wide(5),
        },
    ))
}

/// `sequence` is newer than `last`, across wrapping.
fn later(sequence: u32, last: u32) -> bool {
    sequence.wrapping_sub(last) as i32 > 0
}

fn record_words(sequence: u32, totals: Totals) -> [u32; RECORD_WORDS] {
    let data = [
        sequence,
        totals.ma_ms as u32,
        (totals.ma_ms >> 32) as u32,
        totals.uw_ms as u32,
        (totals.uw_ms >> 32) as u32,
        totals.elapsed_ms as u32,
        (totals.elapsed_ms >> 32) as u32,
    ];
    core::array::from_fn(|i| {
        if i < DATA_WORDS {
            data[i]
        } else if i < DATA_WORDS * 2 {
            !data[i - DATA_WORDS]
        } else {
            COMMIT
        }
    })
}

/// Prepare the reserved pair before any protocol work starts: a claimed
/// sector keeps its records; an unclaimed one is reclaimed (see the
/// module doc — the pair lives in this firmware's own region).
pub fn prepare(flash: &mut impl Storage) -> Preparation {
    let headers = [header(flash, 0, CLAIM), header(flash, 1, CLAIM)];
    for (sector, claimed) in headers.into_iter().enumerate() {
        if !claimed && !blank(flash, sector) && !flash.erase(sector) {
            return Preparation::WriteFailed;
        }
    }
    Preparation::Ready
}

impl Store {
    /// Read-only recovery. Foreign contents are left untouched;
    /// unavailable storage is reported rather than reclaiming settings
    /// or stock data.
    pub fn open(flash: &mut impl Storage) -> Option<Self> {
        let headers = [header(flash, 0, CLAIM), header(flash, 1, CLAIM)];
        let claimed = headers.iter().any(|found| *found);
        if !claimed && !(blank(flash, 0) && blank(flash, 1)) {
            return None;
        }
        let mut current = usize::from(!headers[0]);
        let mut next = [HEADER_BYTES; 2];
        let mut newest = None;
        for sector in 0..2 {
            if !headers[sector] {
                continue;
            }
            for offset in (HEADER_BYTES..=SECTOR_BYTES - RECORD_BYTES).step_by(RECORD_BYTES) {
                if (0..RECORD_WORDS).any(|i| flash.read(sector, offset + i * 4) != u32::MAX) {
                    next[sector] = offset + RECORD_BYTES;
                }
                if let Some((sequence, totals)) = record(flash, sector, offset)
                    && newest.is_none_or(|(last, _)| later(sequence, last))
                {
                    newest = Some((sequence, totals));
                    current = sector;
                }
            }
        }
        // With no headers both sectors are blank; the first save claims 0.
        if !claimed {
            current = 0;
        }
        Some(Self {
            current,
            next: next[current],
            sequence: newest.map_or(0, |(sequence, _)| sequence),
            saved: newest.map(|(_, totals)| totals),
            claimed,
            failed: false,
        })
    }

    pub fn totals(&self) -> Totals {
        self.saved.unwrap_or_default()
    }

    /// Append only changed totals. On rollover, erase the partner, claim
    /// it, and commit there before it becomes current. A failed
    /// operation stops further writes this boot; recovery scans past
    /// torn records next boot.
    pub fn save(&mut self, flash: &mut impl Storage, totals: Totals) -> bool {
        if self.failed {
            return false;
        }
        if self.saved == Some(totals) {
            return true;
        }
        let written = self.append(flash, totals);
        self.failed = !written;
        written
    }

    fn append(&mut self, flash: &mut impl Storage, totals: Totals) -> bool {
        let mut sector = self.current;
        let mut offset = self.next;
        if !self.claimed {
            if !flash.write(sector, 0, &[CLAIM, !CLAIM]) {
                return false;
            }
            self.claimed = true;
        } else if offset + RECORD_BYTES > SECTOR_BYTES {
            sector = 1 - sector;
            // The current sector's intact header keeps ownership of the pair
            // through a reset at any point in this erase/claim sequence.
            if !blank(flash, sector) && !flash.erase(sector) {
                return false;
            }
            if !flash.write(sector, 0, &[CLAIM, !CLAIM]) {
                return false;
            }
            offset = HEADER_BYTES;
        }
        let sequence = self.sequence.wrapping_add(1);
        if !flash.write(sector, offset, &record_words(sequence, totals)) {
            return false;
        }
        self.current = sector;
        self.next = offset + RECORD_BYTES;
        self.sequence = sequence;
        self.saved = Some(totals);
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Clone)]
    struct Flash {
        words: Vec<u32>,
        remaining: Option<usize>,
        erases: usize,
    }

    impl Flash {
        fn new() -> Self {
            Self {
                words: vec![u32::MAX; SECTOR_BYTES / 4 * 2],
                remaining: None,
                erases: 0,
            }
        }
        fn step(&mut self) -> bool {
            match &mut self.remaining {
                Some(0) => false,
                Some(left) => {
                    *left -= 1;
                    true
                }
                None => true,
            }
        }
    }

    impl Storage for Flash {
        fn read(&mut self, sector: usize, offset: usize) -> u32 {
            self.words[(sector * SECTOR_BYTES + offset) / 4]
        }
        fn write(&mut self, sector: usize, offset: usize, words: &[u32]) -> bool {
            for (i, word) in words.iter().enumerate() {
                if !self.step() {
                    return false;
                }
                let index = (sector * SECTOR_BYTES + offset) / 4 + i;
                assert_eq!(self.words[index], u32::MAX);
                self.words[index] = *word;
            }
            true
        }
        fn erase(&mut self, sector: usize) -> bool {
            self.erases += 1;
            for offset in (0..SECTOR_BYTES).step_by(4) {
                if !self.step() {
                    return false;
                }
                self.words[(sector * SECTOR_BYTES + offset) / 4] = u32::MAX;
            }
            true
        }
    }

    fn totals(n: u64) -> Totals {
        Totals {
            ma_ms: n,
            uw_ms: n * 5000,
            elapsed_ms: n * 100,
        }
    }

    /// kws-x1's own test shape: save across TWO rollovers (the first
    /// finds the partner already blank - no erase; the second erases the
    /// non-blank survivor), recover the newest checkpoint.
    #[test]
    fn saves_across_rollover_and_recovers_newest() {
        let mut flash = Flash::new();
        assert_eq!(prepare(&mut flash), Preparation::Ready);
        let mut store = Store::open(&mut flash).unwrap();
        assert_eq!(store.totals(), Totals::default());
        // 68 records fill one 4 KiB sector ((4096-8)/60 = 68): 140 saves
        // cross the rollover twice, the second time through the erase.
        for n in 1..=140u64 {
            assert!(store.save(&mut flash, totals(n)), "save {n}");
        }
        assert!(flash.erases >= 1);
        let recovered = Store::open(&mut flash).unwrap();
        assert_eq!(recovered.totals(), totals(140));
        // The checkpoint is idempotent.
        assert!(store.save(&mut flash, totals(140)));
        assert_eq!(Store::open(&mut flash).unwrap().totals(), totals(140));
    }

    #[test]
    fn torn_records_are_scanned_past() {
        let mut flash = Flash::new();
        let mut store = Store::open(&mut flash).unwrap();
        assert!(store.save(&mut flash, totals(1)));
        assert!(store.save(&mut flash, totals(2)));
        // A power cut mid-record: the sequence landed but the commit
        // word (and complements) did not — the record is incomplete.
        let record_start = (HEADER_BYTES + RECORD_BYTES) / 4;
        flash.words[record_start + DATA_WORDS * 2] = u32::MAX; // COMMIT lost
        let mut recovered = Store::open(&mut flash).unwrap();
        assert_eq!(recovered.totals(), totals(1));
        // The torn record's slot is reused: the next save rewrites it.
        assert!(recovered.save(&mut flash, totals(3)));
        assert_eq!(Store::open(&mut flash).unwrap().totals(), totals(3));
    }

    #[test]
    fn foreign_content_refuses_recovery_but_prepare_reclaims() {
        let mut flash = Flash::new();
        flash.words[10] = 0x1234;
        assert!(Store::open(&mut flash).is_none());
        assert_eq!(prepare(&mut flash), Preparation::Ready);
        assert!(Store::open(&mut flash).is_some());
    }

    #[test]
    fn claim_only_when_both_blank_or_one_claimed() {
        let mut flash = Flash::new();
        // First save claims sector 0.
        let mut store = Store::open(&mut flash).unwrap();
        assert!(store.save(&mut flash, totals(1)));
        assert_eq!(flash.read(0, 0), CLAIM);
        assert_eq!(flash.read(0, 4), !CLAIM);
        // The partner stays blank until the rollover.
        assert_eq!(flash.read(1, 0), u32::MAX);
        assert!(store.save(&mut flash, totals(2)));
        assert_eq!(flash.read(1, 0), u32::MAX);
    }
}

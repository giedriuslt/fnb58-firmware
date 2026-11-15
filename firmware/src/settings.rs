//! r218: the settings store — the port of kws-x1's
//! `firmware/src/settings_store.rs` rotating-record scheme onto the
//! W25Q128's first 20 sectors (user directive).
//!
//! kws-x1's scheme (settings_store.rs, kept 1:1): a sector is CLAIMED
//! by writing "KWST" (0x4b57_5354) at its base; 8-byte `[word, !word]`
//! records are appended after the claim; the scan walks claims
//! OLDEST-first and the later record wins (so the newest claim's
//! records override); a nearly-full sector is padded with FILLER (0, 0)
//! pairs before the store moves to the next candidate; only the newest
//! claimed sector ever has room.
//!
//! Claim direction (r220 fix): kws-x1's SECTORS list is high-address
//! first and candidates() iterates it REVERSED — their first claim is
//! the region's LOWEST sector (0x72000), advancing upward, opposite to
//! their diagnostic log which grows down from the top. My r218 port
//! inverted this (low-first list × .rev() = high-first), making the
//! first claim sector 19. Claims now advance 0 → 19, leaving the high
//! end free for a future diagnostic log — kws-x1's arrangement.
//!
//! DEVIATION (hardware-forced): kws-x1 never erases — their internal
//! flash starts blank. The W25Q128 carries the stock firmware's
//! littlefs, so a claim ERASES the sector first (flash.rs, confined to
//! the settings region). Records still never erase: a save appends
//! in-place, and a claim happens once per ~10,240 records (20 sectors
//! x 512 records each).
//!
//! Second deviation (the migration): open() stops at the first
//! UNCLAIMED candidate in claim order — claims are contiguous, and a
//! claim beyond a gap is a relic of the r218 direction (the sector-19
//! claim this build's predecessors left) and is ignored. kws-x1 needs
//! no such rule (their region starts fully blank).
//!
//! First setting (r218): the idle-dim timeout, 1/2/5/10 minutes or
//! Never (0) — one word in the low byte, reserved bits zero (the
//! codecs live in the host-tested `settings-core` crate).
//!
//! r221: the calibration rides the same records — one per field
//! (zero/current/voltage, the kws-x1 tag codec in `settings-core::
//! calibration`), saved beside the dim word and written as the FULL
//! record set when a new sector is claimed (kws-x1's "a newly claimed
//! sector starts with all of them").

use settings_core::calibration::{Calibration, FIELDS, Found};
use settings_core::energy_store::{self, Storage};
use settings_core::{CLAIM, FIRST, FILLER, RECORD, BLANK};
use settings_core::{dim_minutes_to_record, energy_memory_from_record, scan_records};
use settings_core::{settings_word, ScanState};

use crate::flash::{self, Flash};

/// The settings area: the external flash's first 20 sectors
/// (0x000000..0x014000, 80 KiB). User directive; everything beyond
/// stays the stock firmware's (untouched).
pub const SECTORS: u32 = 20;

/// r226: the energy journal's pair — the TOP of the settings area
/// (kws-x1 reserves two sectors for their journal the same way). The
/// settings claims must never reach them.
pub const ENERGY_SECTORS: [u32; 2] = [SECTORS - 2, SECTORS - 1];

/// The applied idle-dim timeout in minutes (0 = never) — the idle
/// logic and the Settings screen both read this.
pub static DIM_MINUTES: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(1);

/// The applied calibration's fields (atomics: the meter read path and
/// the wizard's live rows read them every 200 ms, single-core).
/// Defaults change nothing (settings-core's tested invariant).
pub static CAL_ZERO_UA: core::sync::atomic::AtomicI32 = core::sync::atomic::AtomicI32::new(0);
pub static CAL_CURRENT_PPM: core::sync::atomic::AtomicI32 = core::sync::atomic::AtomicI32::new(0);
pub static CAL_VOLTAGE_PPM: core::sync::atomic::AtomicI32 = core::sync::atomic::AtomicI32::new(0);

/// r226: the applied energy checkpoint interval, as the EnergyMemory
/// index (settings-core's ALL order: 0 = Off, then 5/10/30/60 s).
pub static ENERGY_IDX: core::sync::atomic::AtomicUsize =
    core::sync::atomic::AtomicUsize::new(0);

/// r226: the applied interval in seconds, `None` = Off (the main
/// loop's checkpoint cadence reads this).
pub fn energy_interval() -> Option<u64> {
    let idx = ENERGY_IDX.load(core::sync::atomic::Ordering::Relaxed);
    settings_core::EnergyMemory::ALL
        .get(idx)
        .and_then(|&m| m.seconds())
}

/// r226: the journal's two-sector view of the board flash — a fresh
/// borrow per operation (the store keeps only cursor state, so the
/// flash stays shareable with the settings save and the console).
pub struct EnergyFlash<'a>(pub &'a mut Flash);

impl Storage for EnergyFlash<'_> {
    fn read(&mut self, sector: usize, offset: usize) -> u32 {
        let addr = ENERGY_SECTORS[sector] * flash::SECTOR_SIZE as u32 + offset as u32;
        // An error reads as non-blank garbage: the scan skips it, the
        // blank() checks fail, and nothing is ever written over it.
        self.0.read_u32le(addr).unwrap_or(0)
    }
    fn write(&mut self, sector: usize, offset: usize, words: &[u32]) -> bool {
        let addr = ENERGY_SECTORS[sector] * flash::SECTOR_SIZE as u32 + offset as u32;
        // One word per page-program call: every word address is 4-aligned
        // (record offsets are 60 B steps from 8), so no 4-byte program
        // ever crosses the W25Q's 256-B page wrap.
        for (i, word) in words.iter().enumerate() {
            let b = word.to_le_bytes();
            if self.0.page_program(addr + (i * 4) as u32, &b).is_err() {
                return false;
            }
        }
        true
    }
    fn erase(&mut self, sector: usize) -> bool {
        self.0.erase_sector(ENERGY_SECTORS[sector]).is_ok()
    }
}

/// r226: prepare the journal pair (before any protocol task runs).
pub fn prepare_energy(flash: &mut Flash) -> energy_store::Preparation {
    energy_store::prepare(&mut EnergyFlash(flash))
}

/// r226: recover the journal's cursor state (read-only).
pub fn open_energy(flash: &mut Flash) -> Option<energy_store::Store> {
    energy_store::Store::open(&mut EnergyFlash(flash))
}

/// The calibration the meter readings currently go through.
pub fn applied() -> Calibration {
    use core::sync::atomic::Ordering::Relaxed;
    Calibration {
        zero_ua: CAL_ZERO_UA.load(Relaxed),
        current_ppm: CAL_CURRENT_PPM.load(Relaxed),
        voltage_ppm: CAL_VOLTAGE_PPM.load(Relaxed),
    }
}

/// Make `cal` the calibration the meter readings go through.
pub fn store_cal(cal: Calibration) {
    use core::sync::atomic::Ordering::Relaxed;
    CAL_ZERO_UA.store(cal.zero_ua, Relaxed);
    CAL_CURRENT_PPM.store(cal.current_ppm, Relaxed);
    CAL_VOLTAGE_PPM.store(cal.voltage_ppm, Relaxed);
}

/// The Settings screen's cycle order (r218: 1-2-5-10-Never, user spec).
pub const DIM_CHOICES: [u32; 5] = [1, 2, 5, 10, 0];

/// The DIM_CHOICES index of a stored minutes value (unknown → the
/// default's index, 1 minute).
pub fn sel_of(minutes: u32) -> usize {
    DIM_CHOICES
        .iter()
        .position(|&m| m == minutes)
        .unwrap_or(0)
}

/// Where the next record goes (sector index, byte offset in the
/// sector), `None` when no claimed sector has room, plus what the last
/// attempted save wrote (kws-x1's `saved`: updated before the write —
/// one attempt per change, a failed write is not retried, the values
/// stay applied in RAM).
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct Store {
    next: Option<(u32, usize)>,
    /// The stored dim word (0xFF = nothing stored yet — never a valid
    /// dim record) and the stored calibration.
    saved: (u32, Calibration),
}

fn log(line: &[u8]) {
    crate::pd_log(line);
}

/// Read the claim word of one sector.
fn claim_word(flash: &mut Flash, sector: u32) -> u32 {
    flash
        .read_u32le(sector * flash::SECTOR_SIZE as u32)
        .unwrap_or(0)
}

/// Scan one claimed sector's records (one 4 KiB read, ~3 ms). Returns
/// the updated scan state (later records win, kws-x1's fold) and the
/// sector's first free slot.
fn scan(flash: &mut Flash, sector: u32, state: ScanState) -> (ScanState, Option<(u32, usize)>) {
    let mut buf = [0u8; flash::SECTOR_SIZE];
    if flash.read_sector(sector, &mut buf).is_err() {
        return (state, None);
    }
    let words = buf
        .chunks_exact(8)
        .map(|pair| {
            (
                u32::from_le_bytes(pair[..4].try_into().unwrap()),
                u32::from_le_bytes(pair[4..].try_into().unwrap()),
            )
        })
        .skip(FIRST / 8);
    let end = FIRST + (flash::SECTOR_SIZE - FIRST) / RECORD * RECORD;
    let (state, slot) = scan_records(words, state, end);
    (state, slot.map(|slot| (sector, slot)))
}

/// Open the store: JEDEC-gated, candidates scanned oldest-claimed
/// first (claims advance 0 → 17, r226 — the top two sectors are the
/// energy journal's pair). Applies the loaded settings to their
/// statics. Returns the store and the loaded dim value, `None` =
/// nothing stored.
pub fn open(flash: &mut Flash) -> (Store, Option<u32>) {
    let store = Store {
        next: None,
        saved: (0xFF, Calibration::default()),
    };
    match flash.read_jedec_id() {
        Ok(id) if id == flash::JEDEC_W25Q128 => {}
        _ => {
            log(b"[set] flash JEDEC FAIL\r\n");
            return (store, None);
        }
    }
    let mut next = None;
    let mut state = ScanState::default();
    // Claims advance 0 → 17 (r226 reserves the top two sectors for the
    // energy journal): scan oldest-claimed first, and stop at the first
    // unclaimed candidate — a claimed sector beyond the gap is a
    // pre-r220 relic and must not override newer records.
    for sector in 0..SECTORS {
        if claim_word(flash, sector) != CLAIM {
            break;
        }
        let (newer, room) = scan(flash, sector, state);
        state = newer;
        next = next.or(room);
    }
    let ScanState { dim, energy, cal } = state;
    match dim {
        Some(minutes) => {
            DIM_MINUTES.store(minutes, core::sync::atomic::Ordering::Relaxed);
            if let Some(idx) = energy {
                ENERGY_IDX.store(idx, core::sync::atomic::Ordering::Relaxed);
            }
            store_cal(cal.calibration());
            let mut d = [0u8; 40];
            let mut n = 0;
            for &b in b"[set] stored dim=" {
                d[n] = b;
                n += 1;
            }
            n = crate::push_dec(&mut d, n, minutes);
            for &b in b" min\r\n" {
                d[n] = b;
                n += 1;
            }
            log(&d[..n]);
            log_energy();
            log_cal(b"[set] stored cal", cal.calibration());
            (
                Store {
                    next,
                    saved: (settings_word(minutes, energy.unwrap_or(0)), cal.calibration()),
                },
                Some(minutes),
            )
        }
        None => {
            // No dim record (kws-x1's scan folds every field
            // independently, so a calibration-only find still loads —
            // ours cannot exist today, save() always writes the dim
            // word, but the fold is honored anyway).
            log(b"[set] no stored settings (default dim=1)\r\n");
            if let Some(idx) = energy {
                ENERGY_IDX.store(idx, core::sync::atomic::Ordering::Relaxed);
                log_energy();
            }
            if cal.any() {
                store_cal(cal.calibration());
                log_cal(b"[set] stored cal", cal.calibration());
            }
            (
                Store {
                    next,
                    saved: (0xFF, cal.calibration()),
                },
                None,
            )
        }
    }
}

/// r226: log the applied energy checkpoint interval.
fn log_energy() {
    let idx = ENERGY_IDX.load(core::sync::atomic::Ordering::Relaxed);
    let label = settings_core::EnergyMemory::ALL
        .get(idx)
        .map(|&m| m.label())
        .unwrap_or("OFF");
    let mut d = [0u8; 32];
    let mut n = 0;
    for &b in b"[set] energy " {
        d[n] = b;
        n += 1;
    }
    for &b in label.as_bytes() {
        d[n] = b;
        n += 1;
    }
    for &b in b"\r\n" {
        d[n] = b;
        n += 1;
    }
    log(&d[..n]);
}

/// Log a calibration's fields after `prefix` (b"[set] stored cal" /
/// b"[cal] applied"): z= µA, c=/v= ppm. Every field logs — a zero
/// field is a real zero, not a missing one.
fn log_cal(prefix: &[u8], cal: Calibration) {
    let mut d = [0u8; 64];
    let mut n = 0;
    for &b in prefix {
        d[n] = b;
        n += 1;
    }
    for (label, value) in [
        (&b" z="[..], i64::from(cal.zero_ua)),
        (&b" c="[..], i64::from(cal.current_ppm)),
        (&b" v="[..], i64::from(cal.voltage_ppm)),
    ] {
        for &b in label {
            d[n] = b;
            n += 1;
        }
        if value < 0 {
            d[n] = b'-';
            n += 1;
        }
        n = crate::push_dec(&mut d, n, value.unsigned_abs() as u32);
    }
    for &b in b"\r\n" {
        d[n] = b;
        n += 1;
    }
    log(&d[..n]);
}

/// The store's full record set: the settings word (dim + energy), then
/// the calibration's one record per field (kws-x1's records[..] shape —
/// a newly claimed sector starts with all of them, so it never depends
/// on older ones).
fn record_set(minutes: u32, energy_idx: usize, cal: Calibration) -> [u32; 1 + FIELDS] {
    let mut records = [0u32; 1 + FIELDS];
    records[0] = settings_word(minutes, energy_idx);
    records[1..].copy_from_slice(&cal.records());
    records
}

/// Save the settings word (dim + energy index) and the calibration
/// (append only the changed records unless the store is full — then a
/// new sector is claimed with the FULL record set, kws-x1's save).
/// Every outcome logs — no silent failure branches. The values are
/// applied to their statics BEFORE the write (kws-x1: one attempt per
/// change, a failed write is not retried, the values stay in RAM).
pub fn save(
    flash: &mut Flash,
    store: &mut Store,
    minutes: u32,
    energy_idx: usize,
    cal: Calibration,
) {
    let Some(word) = dim_minutes_to_record(minutes) else {
        log(b"[set] save refused (bad value)\r\n");
        return;
    };
    let word = settings_word(minutes, energy_idx);
    if word == store.saved.0 && cal == store.saved.1 {
        return;
    }
    // Only the changed records append (kws-x1's `changed` fold: the
    // dim word if it changed, all FIELDS calibration records if the
    // calibration as a whole changed) — compared against the OLD saved
    // set, which updates only after the fold.
    let records = record_set(minutes, energy_idx, cal);
    let mut changed = [0u32; 1 + FIELDS];
    let mut count = 0;
    for (record, keep) in records.iter().zip(
        [word != store.saved.0]
            .into_iter()
            .chain([cal != store.saved.1; FIELDS]),
    ) {
        if keep {
            changed[count] = *record;
            count += 1;
        }
    }
    let changed = &changed[..count];
    DIM_MINUTES.store(minutes, core::sync::atomic::Ordering::Relaxed);
    ENERGY_IDX.store(energy_idx, core::sync::atomic::Ordering::Relaxed);
    store_cal(cal);
    store.saved = (word, cal);
    let result = match store.next {
        Some((sector, slot)) if slot + RECORD * changed.len() <= flash::SECTOR_SIZE => {
            write_records(flash, sector, slot, changed).map(|slot| (sector, slot))
        }
        // A new sector: fill the old one's tail with FILLER pairs
        // (kws-x1's invariant — only the newest claimed sector has
        // room), erase + claim the next blank candidate, then write the
        // FULL record set at FIRST.
        _ => claim_next(flash, store).and_then(|(sector, slot)| {
            write_records(flash, sector, slot, &records).map(|slot| (sector, slot))
        }),
    };
    match result {
        Ok((sector, slot)) => {
            store.next = (slot + RECORD <= flash::SECTOR_SIZE).then_some((sector, slot));
            let mut d = [0u8; 56];
            let mut n = 0;
            for &b in b"[set] dim=" {
                d[n] = b;
                n += 1;
            }
            n = crate::push_dec(&mut d, n, minutes);
            for &b in b" EN=" {
                d[n] = b;
                n += 1;
            }
            for &b in settings_core::EnergyMemory::ALL
                .get(energy_idx)
                .map(|&m| m.label())
                .unwrap_or("OFF")
                .as_bytes()
            {
                d[n] = b;
                n += 1;
            }
            for &b in b" saved s=" {
                d[n] = b;
                n += 1;
            }
            n = crate::push_dec(&mut d, n, u32::from(sector));
            for &b in b"\r\n" {
                d[n] = b;
                n += 1;
            }
            log(&d[..n]);
            log_cal(b"[set] cal", cal);
        }
        Err(()) => {
            log(b"[set] save FAIL (claim or write)\r\n");
        }
    }
}

/// Write `[word, !word]` pairs from `records` starting at `slot`;
/// returns the slot past the last record. A failed write stops the
/// sequence (the slot still advances past a partly-programmed pair on
/// the next claim — kws-x1's note).
fn write_records(
    flash: &mut Flash,
    sector: u32,
    slot: usize,
    records: &[u32],
) -> Result<usize, ()> {
    let mut slot = slot;
    for &record in records {
        let mut pair = [0u8; RECORD];
        pair[..4].copy_from_slice(&record.to_le_bytes());
        pair[4..].copy_from_slice(&(!record).to_le_bytes());
        flash.page_program(sector * flash::SECTOR_SIZE as u32 + slot as u32, &pair)?;
        slot += RECORD;
    }
    Ok(slot)
}

/// Fill the current claimed sector's tail with FILLER pairs, then
/// erase + claim the next blank candidate sector and write the CLAIM
/// word. Returns the new (sector, first-record-slot) on success.
fn claim_next(flash: &mut Flash, store: &mut Store) -> Result<(u32, usize), ()> {
    // Pad the old sector so no scan follows it past its records. A
    // FILLER pair is (0, 0): the check word is not the complement, so
    // every scan skips it.
    if let Some((sector, mut slot)) = store.next {
        // The FILLER pair (0, 0): the check word is not the complement.
        let filler = {
            let mut pair = [0u8; RECORD];
            pair[..4].copy_from_slice(&FILLER.to_le_bytes());
            pair[4..].copy_from_slice(&FILLER.to_le_bytes());
            pair
        };
        while slot + RECORD <= flash::SECTOR_SIZE {
            flash.page_program(sector * flash::SECTOR_SIZE as u32 + slot as u32, &filler)?;
            slot += RECORD;
        }
    }
    // The next claim: the first unclaimed candidate in claim order
    // (0 → 19, r220 — kws-x1's direction; claims stay contiguous). The
    // erase is our littlefs deviation.
    let sector = (0..SECTORS)
        .find(|&s| claim_word(flash, s) != CLAIM && !ENERGY_SECTORS.contains(&s))
        .ok_or(())?;
    flash.erase_sector(sector)?;
    // The erase must leave the sector blank (a failed erase would let
    // a stale CLAIM pass the next boot's scan — verify, don't assume).
    let check = flash.read_u32le(sector * flash::SECTOR_SIZE as u32)?;
    if check != BLANK {
        log(b"[set] erase verify FAIL\r\n");
        return Err(());
    }
    flash.page_program(sector * flash::SECTOR_SIZE as u32, &CLAIM.to_le_bytes())?;
    Ok((sector, FIRST))
}

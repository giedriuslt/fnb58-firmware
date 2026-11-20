//! r239: the offline recorder — calibrated V/I samples into the W25Q128's
//! free area, kws-x1's energy-journal spirit scaled to a long log.
//!
//! Layout (user directive: the settings area keeps its first 20 sectors;
//! everything here is beyond it):
//!   sector 20           meta: magic "FNRL", version, interval seconds,
//!                       and an "FEND" word programmed ONLY by an explicit
//!                       stop (blank = still recording / interrupted).
//!   sectors 21..4096    samples, 4 bytes each (u16 mV LE, u16 mA LE —
//!                       magnitudes, the overview's own display units),
//!                       1024 per 4 KiB sector; sample n lives at a fixed
//!                       address, so the write cursor IS the sample count.
//! Capacity 4,075 sectors x 1024 = 4,172,800 samples (~48 days at 1 s).
//!
//! Time axis: sample n was taken n x interval seconds after the log's
//! start (pauses stretch nothing — no samples are written while
//! stopped). The interval is captured at START from the settings row and
//! kept for the whole log; RESUME re-programs the meta with the SAME
//! value, so a log is always single-interval and the exporter reads it
//! from the meta.
//!
//! Interrupted sessions (a VBUS dip kills the meter mid-log) recover:
//! the meta has no FEND, so the next boot auto-resumes at the scanned
//! cursor. The scan is a binary search over sector blankness (the log
//! is a non-blank prefix) plus one word-scan of the cursor sector.
//!
//! Every long flash operation runs from the MAIN task only while
//! `pd_task::BUSY` is false (the r160 draw-starvation rule: a 60-400 ms
//! sector erase must never land inside a PD exchange), and CLEAR is a
//! step function the main loop drives one sector per tick so the UI
//! stays live.

use core::sync::atomic::{AtomicU16, AtomicU32, AtomicU8, Ordering};

use crate::flash::Flash;

/// The log header's magic ("FNRL", little-endian bytes F N R L).
const META_MAGIC: u32 = 0x4C52_4E46;
/// The explicit-stop marker ("FEND"), programmed at meta+8 only by
/// [`stop`]; its blank state means "recording or interrupted".
const FEND: u32 = 0x444E_4546;
/// The meta sector (inside nothing else's range: the settings store
/// keeps sectors 0..19).
const META_SECTOR: u32 = 20;
/// First sample sector.
pub const REC_FIRST: u32 = 21;
/// Samples per 4 KiB sector (4 bytes each).
pub const SAMPLES_PER_SECTOR: u32 = 1024;
/// The W25Q128's sector count; the sample area ends with the chip.
const SECTOR_COUNT: u32 = 4096;
/// Total sample capacity.
const CAPACITY: u32 = (SECTOR_COUNT - REC_FIRST) * SAMPLES_PER_SECTOR;

/// The recorder's state (the UI and the console read these).
pub const NO_LOG: u8 = 0;
pub const RUNNING: u8 = 1;
pub const STOPPED: u8 = 2;
pub const CLEARING: u8 = 3;

pub static STATE: AtomicU8 = AtomicU8::new(NO_LOG);
/// Samples written so far — the write cursor.
pub static COUNT: AtomicU32 = AtomicU32::new(0);
/// The active log's interval in seconds (the log's own meta value, not
/// the setting — they differ between a resume and a fresh start).
pub static INTERVAL: AtomicU8 = AtomicU8::new(1);
/// Clear progress (sectors erased / to erase), live while CLEARING.
pub static CLEAR_DONE: AtomicU16 = AtomicU16::new(0);
pub static CLEAR_TOTAL: AtomicU16 = AtomicU16::new(0);
/// Next sector to erase during CLEARING.
static CLEAR_NEXT: AtomicU16 = AtomicU16::new(0);
/// Last used sample sector during CLEARING.
static CLEAR_LAST: AtomicU16 = AtomicU16::new(0);

pub fn state() -> u8 {
    STATE.load(Ordering::Relaxed)
}
pub fn count() -> u32 {
    COUNT.load(Ordering::Relaxed)
}
pub fn interval() -> u32 {
    u32::from(INTERVAL.load(Ordering::Relaxed))
}
/// Sample sectors touched by the log (>= 1 once anything is stored).
pub fn used_sectors() -> u32 {
    let count = count();
    if count == 0 {
        0
    } else {
        (count + SAMPLES_PER_SECTOR - 1) / SAMPLES_PER_SECTOR
    }
}
/// Total sample capacity (the UI's FULL check).
pub fn capacity() -> u32 {
    CAPACITY
}
/// The menu the REC screen offers in this state ("CLEARING" has none —
/// the step loop owns the screen until it finishes).
pub fn menu(state: u8) -> &'static [&'static [u8]] {
    match state {
        NO_LOG => &[b"START"],
        RUNNING => &[b"STOP", b"CLEAR"],
        STOPPED => &[b"RESUME", b"CLEAR"],
        _ => &[],
    }
}

/// The sample `n`'s byte address.
fn addr(n: u32) -> u32 {
    (REC_FIRST + n / SAMPLES_PER_SECTOR) * crate::flash::SECTOR_SIZE as u32
        + (n % SAMPLES_PER_SECTOR) * 4
}

/// Write the meta sector: erase, program the header, optionally the
/// FEND marker. `interval` rides byte 5.
fn write_meta(flash: &mut Flash, interval: u8, fend: bool) -> Result<(), ()> {
    flash.erase_sector(META_SECTOR)?;
    let mut header = [0u8; 8];
    header[..4].copy_from_slice(&META_MAGIC.to_le_bytes());
    header[4] = 1;
    header[5] = interval;
    flash.page_program(META_SECTOR * crate::flash::SECTOR_SIZE as u32, &header)?;
    if fend {
        flash.page_program(
            META_SECTOR * crate::flash::SECTOR_SIZE as u32 + 8,
            &FEND.to_le_bytes(),
        )?;
    }
    Ok(())
}

/// Read the meta sector, if one is present: `Some((interval, stopped))`.
fn read_meta(flash: &mut Flash) -> Option<(u8, bool)> {
    let magic = flash.read_u32le(META_SECTOR * crate::flash::SECTOR_SIZE as u32).ok()?;
    if magic != META_MAGIC {
        return None;
    }
    let rest = flash
        .read_u32le(META_SECTOR * crate::flash::SECTOR_SIZE as u32 + 4)
        .ok()?;
    let interval = (rest >> 8) as u8;
    let fend = flash
        .read_u32le(META_SECTOR * crate::flash::SECTOR_SIZE as u32 + 8)
        .map_or(false, |word| word == FEND);
    Some((interval, fend))
}

/// Boot recovery: read the meta, binary-search the write cursor, set the
/// statics. A log without FEND auto-resumes (the offline recorder
/// survives the VBUS dip that killed it); one with FEND stays stopped.
/// Loud either way — the boot ring must show what happened to the log.
pub fn recover(flash: &mut Flash) {
    let Some((interval, fend)) = read_meta(flash) else {
        STATE.store(NO_LOG, Ordering::Relaxed);
        return;
    };
    // Binary search: sample sectors form a non-blank prefix; find the
    // first blank one (12 probes over 4075 sectors).
    let mut lo = REC_FIRST;
    let mut hi = SECTOR_COUNT;
    while lo < hi {
        let mid = (lo + hi) / 2;
        let last = flash
            .read_u32le(mid * crate::flash::SECTOR_SIZE as u32 + 4092)
            .unwrap_or(0xFFFF_FFFF);
        if last == 0xFFFF_FFFF {
            hi = mid;
        } else {
            lo = mid + 1;
        }
    }
    let count = if lo == REC_FIRST {
        0
    } else if lo == SECTOR_COUNT {
        // No blank sector: the area is full.
        CAPACITY
    } else {
        // Word-scan the cursor sector for its first blank word.
        let mut k = 0u32;
        while k < SAMPLES_PER_SECTOR {
            let word = flash
                .read_u32le((lo - 1) * crate::flash::SECTOR_SIZE as u32 + k * 4)
                .unwrap_or(0xFFFF_FFFF);
            if word == 0xFFFF_FFFF {
                break;
            }
            k += 1;
        }
        (lo - REC_FIRST - 1) * SAMPLES_PER_SECTOR + k
    };
    INTERVAL.store(interval, Ordering::Relaxed);
    COUNT.store(count, Ordering::Relaxed);
    let (state, label): (u8, &[u8]) = if fend {
        (STOPPED, b"stop")
    } else {
        (RUNNING, b"run")
    };
    STATE.store(state, Ordering::Relaxed);
    let mut d = [0u8; 56];
    let mut n = 0;
    for &b in b"[rec] boot " {
        d[n] = b;
        n += 1;
    }
    for &b in label {
        d[n] = b;
        n += 1;
    }
    for &b in b" int=" {
        d[n] = b;
        n += 1;
    }
    n = crate::push_dec(&mut d, n, u32::from(interval));
    for &b in b"s n=" {
        d[n] = b;
        n += 1;
    }
    n = crate::push_dec(&mut d, n, count);
    for &b in b"\r\n" {
        d[n] = b;
        n += 1;
    }
    crate::pd_log(&d[..n]);
}

/// Start a FRESH log with the setting's interval (a fresh log discards
/// the old one — the UI only offers START with no log present, CLEAR is
/// the explicit destructive path).
pub fn start(flash: &mut Flash, interval: u8) -> Result<(), ()> {
    write_meta(flash, interval, false)?;
    INTERVAL.store(interval, Ordering::Relaxed);
    COUNT.store(0, Ordering::Relaxed);
    STATE.store(RUNNING, Ordering::Relaxed);
    Ok(())
}

/// Explicit stop: program the FEND marker (the data stays, the boot
/// will not auto-resume). Idempotent by the state check — a second
/// program into the same bytes would corrupt them.
pub fn stop(flash: &mut Flash) -> Result<(), ()> {
    if state() != RUNNING {
        return Ok(());
    }
    flash.page_program(
        META_SECTOR * crate::flash::SECTOR_SIZE as u32 + 8,
        &FEND.to_le_bytes(),
    )?;
    STATE.store(STOPPED, Ordering::Relaxed);
    Ok(())
}

/// Resume a stopped log: erase + re-program the meta WITHOUT the FEND
/// marker, keeping the log's own interval and its samples. The cursor
/// (COUNT) is already recovered.
pub fn resume(flash: &mut Flash) -> Result<(), ()> {
    if state() != STOPPED {
        return Ok(());
    }
    let interval = INTERVAL.load(Ordering::Relaxed);
    write_meta(flash, interval, false)?;
    STATE.store(RUNNING, Ordering::Relaxed);
    Ok(())
}

/// Begin a CLEAR: bound the erases by the scanned cursor and hand the
/// work to [`clear_step`] (one sector per main-loop tick). A count of
/// zero is just the meta sector, done inline.
pub fn clear_begin(flash: &mut Flash) {
    let count = count();
    if state() == CLEARING {
        return;
    }
    if count == 0 {
        let _ = flash.erase_sector(META_SECTOR);
        STATE.store(NO_LOG, Ordering::Relaxed);
        let mut d = [0u8; 24];
        let mut n = 0;
        for &b in b"[rec] clear empty\r\n" {
            d[n] = b;
            n += 1;
        }
        crate::pd_log(&d[..n]);
        return;
    }
    let last = REC_FIRST + (count - 1) / SAMPLES_PER_SECTOR;
    CLEAR_NEXT.store(REC_FIRST as u16, Ordering::Relaxed);
    CLEAR_LAST.store(last as u16, Ordering::Relaxed);
    CLEAR_DONE.store(0, Ordering::Relaxed);
    CLEAR_TOTAL.store((last - REC_FIRST + 1 + 1) as u16, Ordering::Relaxed);
    STATE.store(CLEARING, Ordering::Relaxed);
}

/// One CLEAR step: erase one used sector (call only while CLEARING and
/// only from the main loop with `pd_task::BUSY` false). Returns true
/// when the clear is finished (the meta erased, state back to NO_LOG).
pub fn clear_step(flash: &mut Flash) -> bool {
    if state() != CLEARING {
        return true;
    }
    let next = CLEAR_NEXT.load(Ordering::Relaxed);
    let last = CLEAR_LAST.load(Ordering::Relaxed);
    if next <= last {
        let _ = flash.erase_sector(u32::from(next));
        CLEAR_NEXT.store(next + 1, Ordering::Relaxed);
        CLEAR_DONE.store(CLEAR_DONE.load(Ordering::Relaxed) + 1, Ordering::Relaxed);
        false
    } else {
        let _ = flash.erase_sector(META_SECTOR);
        STATE.store(NO_LOG, Ordering::Relaxed);
        COUNT.store(0, Ordering::Relaxed);
        crate::pd_log(b"[rec] clear done\r\n");
        true
    }
}

/// Record one sample (calibrated mV and the mA magnitude, the overview's
/// display units). Erases the sector under the cursor when a fresh one
/// starts (once per 1024 samples); stops loudly when the area is full.
/// Called from the meter block's cadence — never inside a PD exchange.
pub fn sample(flash: &mut Flash, mv: u16, ma: u16) {
    if state() != RUNNING {
        return;
    }
    let n = count();
    if n >= CAPACITY {
        let _ = stop(flash);
        crate::pd_log(b"[rec] FLASH FULL - stopped\r\n");
        return;
    }
    let sector = REC_FIRST + n / SAMPLES_PER_SECTOR;
    if n % SAMPLES_PER_SECTOR == 0 {
        let _ = flash.erase_sector(sector);
    }
    let word = (u32::from(ma) << 16) | u32::from(mv);
    if flash
        .page_program(addr(n), &word.to_le_bytes())
        .is_err()
    {
        crate::pd_log(b"[rec] write FAIL\r\n");
        return;
    }
    COUNT.store(n + 1, Ordering::Relaxed);
}

/// One record's raw word, for the console dump.
pub fn read_word(flash: &mut Flash, n: u32) -> Result<u32, ()> {
    flash.read_u32le(addr(n))
}

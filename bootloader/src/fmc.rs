//! Internal-flash (FMC) driver for the bootloader, ported from the
//! vendor `at32f403a_407_flash.c` sequences 1:1 (the HAL has no flash
//! module): key unlock, 2 KiB sector erase, word program, bounded
//! busy-waits, W1C flag clears, and a read-back verify after every
//! program.
//!
//! Hard guard: this driver can only ever touch the application region
//! and the trailer sector ([`APP_BASE`]..[`META_BASE`]+[`SECTOR`]). The
//! bootloader's own 64 KiB is refused unconditionally — a protocol bug
//! can corrupt the app, never the recovery path.
//!
//! Every failure is a typed error the caller turns into a NAK line;
//! nothing here fails silently.

use embassy_at32::pac::at32f403a::FLASH;
use fnb58_bootloader::meta::{APP_BASE, APP_END, META_BASE, SECTOR};

/// Vendor FLASH_UNLOCK_KEY1/KEY2 (`at32f403a_407_flash.h:52-54`).
const UNLOCK_KEY1: u32 = 0x4567_0123;
const UNLOCK_KEY2: u32 = 0xCDEF_89AB;
/// Vendor ERASE_TIMEOUT / PROGRAMMING_TIMEOUT (`flash.h:160-161`).
const ERASE_TIMEOUT: u32 = 0x4000_0000;
const PROGRAM_TIMEOUT: u32 = 0x0010_0000;
/// STS bits (vendor flags 0x00000004 PRGMERR, 0x00000010 EPPERR,
/// 0x00000020 ODF — W1C; OBF 0x01 is read-only).
const ERR_FLAGS: u32 = 0x34;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// Unlock keys were written but the controller stayed locked.
    Locked,
    /// Operation did not complete within the vendor timeout.
    Timeout,
    /// PRGMERR: the controller refused a program (e.g. bits already 0).
    ProgramError,
    /// EPPERR: erase/program protection is active.
    ProtectError,
    /// Read-back after program did not return what was written.
    Verify,
    /// Address outside the writable window or misaligned.
    BadAddress,
}

impl Error {
    /// Short word for a NAK reason.
    pub fn word(&self) -> &'static str {
        match self {
            Error::Locked => "locked",
            Error::Timeout => "timeout",
            Error::ProgramError => "prgmerr",
            Error::ProtectError => "protect",
            Error::Verify => "verify",
            Error::BadAddress => "range",
        }
    }
}

fn flash() -> FLASH {
    // SAFETY: the flash controller is a single static peripheral; this
    // driver is the only user (single-task bootloader, no ISRs touch it).
    unsafe { FLASH::steal() }
}

/// The writable window check. `addr..addr+len` must sit inside the app
/// region or inside the trailer sector (the WHOLE 2 KiB — bl2's guard
/// only allowed `addr == META_BASE`, so COMMIT's word programs at
/// META_BASE+4/8/12 were refused mid-write and the trailer ended up
/// half-written/invalid; found in the first hardware upload round).
fn guard(addr: u32, len: u32) -> Result<(), Error> {
    let end = addr.checked_add(len).ok_or(Error::BadAddress)?;
    let in_app = addr >= APP_BASE && end <= APP_END;
    let in_meta = addr >= META_BASE && end <= META_BASE + SECTOR;
    if (in_app || in_meta) && addr % 4 == 0 {
        Ok(())
    } else {
        Err(Error::BadAddress)
    }
}

/// Vendor `flash_unlock`: keys into both banks, then verify the lock bit
/// actually dropped (the vendor does not check; a write to a locked
/// controller is a silent no-op — exactly what this driver refuses to be).
fn unlock() -> Result<(), Error> {
    let f = flash();
    f.unlock().write(|w| unsafe { w.bits(UNLOCK_KEY1) });
    f.unlock().write(|w| unsafe { w.bits(UNLOCK_KEY2) });
    f.unlock2().write(|w| unsafe { w.bits(UNLOCK_KEY1) });
    f.unlock2().write(|w| unsafe { w.bits(UNLOCK_KEY2) });
    if f.ctrl().read().oplk().bit_is_set() || f.ctrl2().read().oplk().bit_is_set() {
        return Err(Error::Locked);
    }
    Ok(())
}

/// Vendor `flash_lock`: set the operation lock on both banks.
fn lock() {
    let f = flash();
    f.ctrl().modify(|_, w| w.oplk().set_bit());
    f.ctrl2().modify(|_, w| w.oplk().set_bit());
}

/// Vendor `flash_bank1/2_operation_status_get` + `wait_for`: poll busy,
/// then name the error. bank2 = false polls STS, true polls STS2.
fn wait_done(bank2: bool, mut timeout: u32) -> Result<(), Error> {
    let f = flash();
    loop {
        let (obf, prgmerr, epperr) = if bank2 {
            let r = f.sts2().read();
            (
                r.obf().bit_is_set(),
                r.prgmerr().bit_is_set(),
                r.epperr().bit_is_set(),
            )
        } else {
            let r = f.sts().read();
            (
                r.obf().bit_is_set(),
                r.prgmerr().bit_is_set(),
                r.epperr().bit_is_set(),
            )
        };
        if obf {
            if timeout == 0 {
                return Err(Error::Timeout);
            }
            timeout -= 1;
            continue;
        }
        if prgmerr {
            return Err(Error::ProgramError);
        }
        if epperr {
            return Err(Error::ProtectError);
        }
        return Ok(());
    }
}

/// Vendor `flash_flag_clear` for the error/done flags of one bank.
fn clear_flags(bank2: bool) {
    let f = flash();
    if bank2 {
        f.sts2().write(|w| unsafe { w.bits(ERR_FLAGS) });
    } else {
        f.sts().write(|w| unsafe { w.bits(ERR_FLAGS) });
    }
}

/// Erase one 2 KiB sector (vendor `flash_sector_erase`), then verify the
/// sector actually reads all-0xFF.
pub fn erase_sector(addr: u32) -> Result<(), Error> {
    guard(addr, SECTOR)?;
    if addr % SECTOR != 0 {
        return Err(Error::BadAddress);
    }
    let f = flash();
    unlock()?;
    let bank2 = addr >= 0x0808_0000;
    let status = if bank2 {
        f.ctrl2().modify(|_, w| w.secers().set_bit());
        f.addr2().write(|w| unsafe { w.bits(addr) });
        f.ctrl2().modify(|_, w| w.erstr().set_bit());
        let s = wait_done(true, ERASE_TIMEOUT);
        f.ctrl2().modify(|_, w| w.secers().clear_bit());
        s
    } else {
        f.ctrl().modify(|_, w| w.secers().set_bit());
        f.addr().write(|w| unsafe { w.bits(addr) });
        f.ctrl().modify(|_, w| w.erstr().set_bit());
        let s = wait_done(false, ERASE_TIMEOUT);
        f.ctrl().modify(|_, w| w.secers().clear_bit());
        s
    };
    clear_flags(bank2);
    lock();
    status?;
    // Verify erased: a sector that still holds data fails loudly.
    let bytes = unsafe { core::slice::from_raw_parts(addr as *const u8, SECTOR as usize) };
    if bytes.iter().any(|&b| b != 0xFF) {
        return Err(Error::Verify);
    }
    Ok(())
}

/// Program one word (vendor `flash_word_program`) with read-back verify.
pub fn word_program(addr: u32, word: u32) -> Result<(), Error> {
    guard(addr, 4)?;
    let f = flash();
    unlock()?;
    let bank2 = addr >= 0x0808_0000;
    let status = if bank2 {
        f.ctrl2().modify(|_, w| w.fprgm().set_bit());
        unsafe { (addr as *mut u32).write_volatile(word) };
        let s = wait_done(true, PROGRAM_TIMEOUT);
        f.ctrl2().modify(|_, w| w.fprgm().clear_bit());
        s
    } else {
        f.ctrl().modify(|_, w| w.fprgm().set_bit());
        unsafe { (addr as *mut u32).write_volatile(word) };
        let s = wait_done(false, PROGRAM_TIMEOUT);
        f.ctrl().modify(|_, w| w.fprgm().clear_bit());
        s
    };
    clear_flags(bank2);
    lock();
    status?;
    if unsafe { (addr as *const u32).read_volatile() } != word {
        return Err(Error::Verify);
    }
    Ok(())
}

/// Write a 2 KiB sector: erase, then program 512 words (a trailing
/// partial word is padded with 0xFF, the erased-flash state), verifying
/// each word. `data.len()` may be anything ≤ SECTOR.
pub fn program_sector(addr: u32, data: &[u8]) -> Result<(), Error> {
    if data.len() > SECTOR as usize {
        return Err(Error::BadAddress);
    }
    erase_sector(addr)?;
    for i in (0..SECTOR).step_by(4) {
        let idx = i as usize;
        let word = if idx + 4 <= data.len() {
            u32::from_le_bytes([data[idx], data[idx + 1], data[idx + 2], data[idx + 3]])
        } else {
            let mut bytes = [0xFFu8; 4];
            let take = data.len() - idx;
            bytes[..take].copy_from_slice(&data[idx..]);
            u32::from_le_bytes(bytes)
        };
        word_program(addr + i, word)?;
    }
    Ok(())
}

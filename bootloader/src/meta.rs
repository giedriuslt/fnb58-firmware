//! Flash layout and the image-validity trailer.
//!
//! Layout of the 1 MiB bank (AT32F403ACGT7, 2 KiB sectors everywhere):
//!
//! | Region    | Range                    | Size  |
//! |-----------|--------------------------|-------|
//! | Bootloader| 0x08000000..0x08010000   | 64 K  |
//! | App       | 0x08010000..0x080FF800   | 958 K |
//! | Trailer   | 0x080FF800..0x08100000   | 2 K   |
//!
//! The trailer sector (the bank's last one, in bank2) holds four words:
//! magic `"FNB5"`, image byte size, CRC-32 over the first `size` bytes of
//! the app region, and the inverted CRC (a second, independent check on
//! partially-programmed metadata). An all-0xFF sector is EMPTY — a fresh
//! chip or a COMMIT that never completed. Anything else that fails a
//! check is INVALID, and the bootloader refuses to jump.

use crate::crc32;

/// Where the application image is linked (firmware/memory.x must agree).
pub const APP_BASE: u32 = 0x0801_0000;
/// Exclusive end of the application region = start of the trailer sector.
pub const APP_END: u32 = 0x080F_F800;
/// The trailer sector base (bank2, the last 2 KiB of the bank).
pub const META_BASE: u32 = 0x080F_F800;
/// Internal-flash erase unit on this device (bank1 and bank2 alike).
pub const SECTOR: u32 = 0x800;
/// Region capacity in bytes (958 KiB).
pub const APP_CAPACITY: u32 = APP_END - APP_BASE;
/// The `"FNB5"` magic (little-endian word).
pub const MAGIC: u32 = u32::from_le_bytes(*b"FNB5");

/// The four trailer words, as read from flash.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Trailer {
    pub magic: u32,
    pub size: u32,
    pub crc: u32,
    pub crc_inv: u32,
}

impl Trailer {
    pub const ERASED: Trailer = Trailer {
        magic: 0xFFFF_FFFF,
        size: 0xFFFF_FFFF,
        crc: 0xFFFF_FFFF,
        crc_inv: 0xFFFF_FFFF,
    };
}

/// Verdict on the app image. Empty = the trailer sector was never
/// written; Valid = jump is allowed; Invalid = refuse and present the
/// update port.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    Empty,
    Valid { size: u32, crc: u32 },
    Invalid,
}

impl State {
    /// Word for the INFO response.
    pub fn word(&self) -> &'static str {
        match self {
            State::Empty => "empty",
            State::Valid { .. } => "valid",
            State::Invalid => "invalid",
        }
    }
}

/// The pure verdict. `image_crc` is the CRC-32 over the first
/// `trailer.size` bytes of the app region as actually present in flash
/// (the device computes it; host tests feed any value).
pub fn classify(trailer: Trailer, image_crc: u32) -> State {
    if trailer == Trailer::ERASED {
        return State::Empty;
    }
    if trailer.magic != MAGIC {
        return State::Invalid;
    }
    if trailer.crc_inv != !trailer.crc {
        return State::Invalid;
    }
    // A nonzero, word-countable size that fits the region. The host pads
    // the final partial word with 0xFF (erased-flash state), so `size`
    // itself may be any byte count but the programmed extent rounds up.
    if trailer.size == 0 || trailer.size > APP_CAPACITY {
        return State::Invalid;
    }
    if image_crc != trailer.crc {
        return State::Invalid;
    }
    State::Valid {
        size: trailer.size,
        crc: trailer.crc,
    }
}

/// The trailer words as memory-mapped reads.
pub fn read_trailer() -> Trailer {
    let p = META_BASE as *const u32;
    // SAFETY: META_BASE..+16 is the reserved trailer sector; reads of
    // internal flash are always safe.
    unsafe {
        Trailer {
            magic: p.read(),
            size: p.add(1).read(),
            crc: p.add(2).read(),
            crc_inv: p.add(3).read(),
        }
    }
}

/// CRC-32 over the first `len` bytes of the memory-mapped app region.
pub fn image_crc(len: u32) -> u32 {
    // SAFETY: the app region is internal flash, always readable, and
    // `len` is bounded by APP_CAPACITY at every call site.
    let bytes = unsafe { core::slice::from_raw_parts(APP_BASE as *const u8, len as usize) };
    crc32::crc32(bytes)
}

/// The full device-side check: read the trailer, CRC the image, classify.
pub fn check() -> State {
    let trailer = read_trailer();
    let len = if trailer.size > APP_CAPACITY {
        APP_CAPACITY
    } else {
        trailer.size
    };
    classify(trailer, image_crc(len))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn trailer(size: u32, crc: u32) -> Trailer {
        Trailer {
            magic: MAGIC,
            size,
            crc,
            crc_inv: !crc,
        }
    }

    #[test]
    fn erased_is_empty() {
        assert_eq!(classify(Trailer::ERASED, 0), State::Empty);
    }

    #[test]
    fn valid_passes() {
        let t = trailer(1024, 0xDEAD_BEEF);
        assert_eq!(classify(t, 0xDEAD_BEEF), State::Valid { size: 1024, crc: 0xDEAD_BEEF });
    }

    #[test]
    fn bad_magic_invalid() {
        let mut t = trailer(1024, 1);
        t.magic = 0x1234_5678;
        assert_eq!(classify(t, 1), State::Invalid);
    }

    #[test]
    fn broken_crc_inv_invalid() {
        let mut t = trailer(1024, 1);
        t.crc_inv = t.crc_inv.wrapping_add(1);
        assert_eq!(classify(t, 1), State::Invalid);
    }

    #[test]
    fn size_bounds_invalid() {
        assert_eq!(classify(trailer(0, 0), 0), State::Invalid);
        assert_eq!(classify(trailer(APP_CAPACITY + 1, 0), 0), State::Invalid);
        // A size that fits is fine even unaligned (padding is 0xFF).
        assert_eq!(classify(trailer(3, 5), 5), State::Valid { size: 3, crc: 5 });
    }

    #[test]
    fn crc_mismatch_invalid() {
        assert_eq!(classify(trailer(1024, 0x1111_1111), 0x2222_2222), State::Invalid);
    }
}

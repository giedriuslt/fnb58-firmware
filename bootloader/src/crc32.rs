//! Software IEEE CRC-32 (reflected, poly 0xEDB88320, init/xorout
//! 0xFFFFFFFF) — bit-identical with `zlib.crc32`, which the Python host
//! flasher uses. Deliberately not the hardware CRC unit: the same value
//! must come out of the device and the host, and a software routine is
//! the same routine on both sides of the wire.

/// Reflected IEEE polynomial.
const POLY: u32 = 0xEDB8_8320;

/// 256-entry lookup table, built at compile time (no RAM cost, no
/// first-use branch).
const TABLE: [u32; 256] = build_table();

const fn build_table() -> [u32; 256] {
    let mut table = [0u32; 256];
    let mut i = 0;
    while i < 256 {
        let mut c = i as u32;
        let mut k = 0;
        while k < 8 {
            c = if c & 1 != 0 { POLY ^ (c >> 1) } else { c >> 1 };
            k += 1;
        }
        table[i] = c;
        i += 1;
    }
    table
}

/// CRC-32 of `data` as a fresh computation.
pub fn crc32(data: &[u8]) -> u32 {
    crc32_finish(crc32_feed(0xFFFF_FFFF, data))
}

/// Incremental form: feed bytes into the raw register state (start from
/// [`INIT`]), then xor the final state with 0xFFFFFFFF.
pub const INIT: u32 = 0xFFFF_FFFF;

pub fn crc32_feed(mut state: u32, data: &[u8]) -> u32 {
    for &b in data {
        state = TABLE[((state ^ u32::from(b)) & 0xFF) as usize] ^ (state >> 8);
    }
    state
}

pub fn crc32_finish(state: u32) -> u32 {
    state ^ 0xFFFF_FFFF
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The canonical check value of the IEEE CRC-32.
    #[test]
    fn check_value_123456789() {
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
    }

    #[test]
    fn empty_is_zero() {
        assert_eq!(crc32(b""), 0x0000_0000);
    }

    #[test]
    fn incremental_matches_oneshot() {
        let data = b"FNB58 bootloader trailer payload, arbitrarily long";
        let oneshot = crc32(data);
        let (a, b) = data.split_at(7);
        let incremental = crc32_finish(crc32_feed(crc32_feed(INIT, a), b));
        assert_eq!(oneshot, incremental);
    }
}

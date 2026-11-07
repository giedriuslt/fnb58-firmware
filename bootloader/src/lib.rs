//! Pure logic for the FNB58 bootloader: flash layout constants, the
//! image CRC, and the metadata trailer format. Host-testable —
//! `cargo test -p fnb58-bootloader` runs these on the build box, so the
//! protocol the device implements is the protocol the tests checked.

#![no_std]

pub mod crc32;
pub mod meta;

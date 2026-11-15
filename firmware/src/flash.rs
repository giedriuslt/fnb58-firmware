//! W25Q128 SPI NOR flash (16 MiB) driver.
//!
//! The flash shares SPI2 with the LCD (SCK=PB13, MOSI=PB15) and adds
//! MISO=PB14; its chip-select is PA13 (LQFP48 pin 34/36 area, user-
//! traced 2026-11-04, active low). The SPI is switched to full-duplex
//! for the duration of a transaction and back to the LCD's single-wire
//! transmit mode afterwards.
//!
//! r218 (user directive): READ-WRITE now — the first 20 sectors are the
//! settings area (`settings.rs`, the kws-x1 rotating-record port), which
//! needs page program + 4 KiB erase. This supersedes the old read-only
//! rule ("the stock firmware's littlefs must not be disturbed"): the
//! erase-on-claim is confined to the settings region and only runs when
//! the store claims a sector. Everything beyond sector 20 stays
//! untouched.

use embassy_at32::gpio::{Drive, OutputPin, Port};
use embassy_at32::spi::Spi;

/// JEDEC ID of the W25Q128: EF (Winbond) 40 18.
pub const JEDEC_W25Q128: [u8; 3] = [0xEF, 0x40, 0x18];

const CMD_READ: u8 = 0x03;
const CMD_JEDEC_ID: u8 = 0x9F;
/// Write enable, before every program/erase (W25Q128 datasheet).
const CMD_WREN: u8 = 0x06;
/// Read status register 1 (bit 0 = BUSY).
const CMD_RDSR: u8 = 0x05;
/// Page program: up to 256 bytes within one 256-byte page.
const CMD_PAGE_PROGRAM: u8 = 0x02;
/// 4 KiB sector erase.
const CMD_SECTOR_ERASE: u8 = 0x20;

/// Sector size (W25Q128: 4 KiB; 4096 sectors in 16 MiB).
pub const SECTOR_SIZE: usize = 4096;
/// Page size (the program command's boundary).
pub const PAGE_SIZE: usize = 256;

/// Status-poll bounds: tPP ~0.4..0.8 ms, tSE ~45..400 ms — generous
/// multiples so a stuck part fails the call instead of hanging boot.
const PP_POLL_BOUND: u32 = 100_000;
const SE_POLL_BOUND: u32 = 2_000_000;

pub struct Flash {
    cs: OutputPin,
    spi: Spi,
}

impl Flash {
    /// Claim PA13 (flash CS, active low) and take a handle on the shared
    /// SPI2. Call after the SWJTAG remap has freed PA13 (the LCD driver's
    /// remap value `100` disables full SWJ).
    ///
    /// # Safety
    ///
    /// Exclusive-startup contract: claims PA13 once.
    pub unsafe fn new(spi: Spi) -> Self {
        let cs = unsafe { OutputPin::new_push_pull(Port::A, 13, true, Drive::Moderate) }
            .expect("PA13 claim");
        Self { cs, spi }
    }

    /// Switch the shared SPI to full-duplex for a transaction.
    fn enter(&mut self) {
        self.spi.set_full_duplex();
    }

    /// Restore the LCD's single-wire transmit mode.
    fn leave(&mut self) {
        self.spi.set_single_wire_transmit();
    }

    /// Run one full-duplex transfer with CS held low.
    fn transfer(&mut self, bytes: &[u8], out: &mut [u8]) -> Result<(), ()> {
        self.enter();
        self.cs.set_low();
        let result = self.spi.transfer_busy(bytes, out);
        self.cs.set_high();
        self.leave();
        result.map_err(|_| ())
    }

    /// JEDEC ID (3 bytes). W25Q128 answers EF 40 18. The byte received
    /// during the command itself is garbage — the ID starts on the next
    /// byte, so four bytes are collected and the first is dropped.
    pub fn read_jedec_id(&mut self) -> Result<[u8; 3], ()> {
        let mut out = [0u8; 4];
        self.transfer(&[CMD_JEDEC_ID, 0, 0, 0], &mut out)?;
        Ok([out[1], out[2], out[3]])
    }

    /// Read one 4 KiB sector (0x03 READ at `sector` * 4096). Blocks for
    /// ~3 ms at the 12 MHz SPI rate.
    pub fn read_sector(&mut self, sector: u32, out: &mut [u8; SECTOR_SIZE]) -> Result<(), ()> {
        let addr = sector * SECTOR_SIZE as u32;
        let mut tx = [0u8; 4 + SECTOR_SIZE];
        tx[0] = CMD_READ;
        tx[1] = (addr >> 16) as u8;
        tx[2] = (addr >> 8) as u8;
        tx[3] = addr as u8;
        // The address bytes clock MISO garbage; the caller's data starts
        // after the command+address, so shift the out window accordingly.
        let mut rx = [0u8; 4 + SECTOR_SIZE];
        self.transfer(&tx, &mut rx)?;
        out.copy_from_slice(&rx[4..]);
        Ok(())
    }

    /// Read one little-endian u32 from `addr` — the settings store's
    /// claim-word checks (a fixed-size form; const-generic array lengths
    /// are not usable in const arithmetic on this toolchain).
    pub fn read_u32le(&mut self, addr: u32) -> Result<u32, ()> {
        let mut tx = [0u8; 8];
        tx[0] = CMD_READ;
        tx[1] = (addr >> 16) as u8;
        tx[2] = (addr >> 8) as u8;
        tx[3] = addr as u8;
        let mut rx = [0u8; 8];
        self.transfer(&tx, &mut rx)?;
        Ok(u32::from_le_bytes([rx[4], rx[5], rx[6], rx[7]]))
    }

    /// Poll status BUSY until clear (bounded). `bound` in poll rounds.
    fn wait_ready(&mut self, bound: u32) -> Result<(), ()> {
        let mut tx = [CMD_RDSR, 0];
        for _ in 0..bound {
            let mut rx = [0u8; 2];
            self.transfer(&tx, &mut rx)?;
            if rx[1] & 1 == 0 {
                return Ok(());
            }
        }
        Err(())
    }

    /// One command with payload, CS held low for the whole transfer
    /// (page program and erase take the address and data inside one CS
    /// period; WREN is its own single-byte transaction). The rx side is
    /// garbage for write commands — it just needs to be as long as tx.
    fn command(&mut self, bytes: &[u8]) -> Result<(), ()> {
        debug_assert!(bytes.len() <= 4 + PAGE_SIZE);
        self.enter();
        self.cs.set_low();
        let mut rx = [0u8; 4 + PAGE_SIZE];
        let result = self.spi.transfer_busy(bytes, &mut rx[..bytes.len()]);
        self.cs.set_high();
        self.leave();
        result.map_err(|_| ())
    }

    /// Erase one 4 KiB sector (0x20). Blocks for the erase duration
    /// (~60 ms typical; bounded well past the worst case).
    pub fn erase_sector(&mut self, sector: u32) -> Result<(), ()> {
        let addr = sector * SECTOR_SIZE as u32;
        self.command(&[CMD_WREN])?;
        let a = [(addr >> 16) as u8, (addr >> 8) as u8, addr as u8];
        self.command(&[CMD_SECTOR_ERASE, a[0], a[1], a[2]])?;
        self.wait_ready(SE_POLL_BOUND)
    }

    /// Program `bytes` at `addr` (0x02). The caller keeps the write
    /// inside one 256-byte page (settings.rs's writes are 4/8-byte
    /// aligned pieces — they never cross). Blocks for tPP.
    pub fn page_program(&mut self, addr: u32, bytes: &[u8]) -> Result<(), ()> {
        debug_assert!(
            bytes.len() <= PAGE_SIZE && addr % PAGE_SIZE as u32 + bytes.len() as u32 <= PAGE_SIZE as u32
        );
        self.command(&[CMD_WREN])?;
        let mut tx = [0u8; 4 + PAGE_SIZE];
        tx[0] = CMD_PAGE_PROGRAM;
        tx[1] = (addr >> 16) as u8;
        tx[2] = (addr >> 8) as u8;
        tx[3] = addr as u8;
        tx[4..4 + bytes.len()].copy_from_slice(bytes);
        let len = 4 + bytes.len();
        self.command(&tx[..len])?;
        self.wait_ready(PP_POLL_BOUND)
    }
}

//! W25Q128 SPI NOR flash (16 MiB) — READ-ONLY driver.
//!
//! The flash shares SPI2 with the LCD (SCK=PB13, MOSI=PB15) and adds
//! MISO=PB14; its chip-select is PA13 (LQFP48 pin 34/36 area, user-
//! traced 2026-11-04, active low). The SPI is switched to full-duplex
//! for the duration of a transaction and back to the LCD's single-wire
//! transmit mode afterwards.
//!
//! READS ONLY by design: no page-program, no sector/block erase, no
//! write-enable. The flash still holds the stock firmware's log
//! filesystem; nothing here may disturb it.

use embassy_at32::gpio::{Drive, OutputPin, Port};
use embassy_at32::spi::Spi;

/// JEDEC ID of the W25Q128: EF (Winbond) 40 18.
pub const JEDEC_W25Q128: [u8; 3] = [0xEF, 0x40, 0x18];

const CMD_READ: u8 = 0x03;
const CMD_JEDEC_ID: u8 = 0x9F;

/// Sector size (W25Q128: 4 KiB; 4096 sectors in 16 MiB).
pub const SECTOR_SIZE: usize = 4096;

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

    /// JEDEC ID (3 bytes). W25Q128 answers EF 40 18.
    pub fn read_jedec_id(&mut self) -> Result<[u8; 3], ()> {
        let mut out = [0u8; 3];
        self.transfer(&[CMD_JEDEC_ID, 0, 0, 0], &mut out)?;
        Ok(out)
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
}

//! Passive USB-PD listener: FUSB302B (I2C 0x22) receive-only driver.
//!
//! The meter sits on the pass-through CC lines between source and DUT,
//! so it must NOT transmit (no auto-GoodCRC): the real sink answers, we
//! only listen for SOP messages and parse Source_Capabilities.
//!
//! Register map per the FUSB302B datasheet (matches the Linux fusb302
//! driver): status/interrupt/FIFO block lives at 0x3C-0x43, with the
//! FIFO at 0x43 auto-popping one byte per read.

use embassy_at32::i2c::{Error, I2c};
use embedded_hal_async::i2c::I2c as _;
use embedded_hal_async::i2c::Operation;

const FUSB: u8 = 0x22;

const REG_DEVICE_ID: u8 = 0x01;
const REG_SWITCHES1: u8 = 0x03;
const REG_POWER: u8 = 0x0C;
const REG_RESET: u8 = 0x0D;
const REG_INTERRUPT: u8 = 0x42;
const REG_FIFOS: u8 = 0x43;

/// Packet-receive switches: BMC receiver onto CC1 / CC2.
const SWITCHES1_REC_CC1: u8 = 1 << 3;
const SWITCHES1_REC_CC2: u8 = 1 << 2;
/// Interrupt (0x42) receive-pending bits.
const INT_RXSOP1: u8 = 1 << 0;
const INT_RXSOP2: u8 = 1 << 1;

/// One parsed power data object.
#[derive(Clone, Copy, Default)]
pub struct Pdo {
    /// Fixed: the voltage, mV. PPS: the maximum voltage, mV.
    pub mv: u32,
    /// Maximum current, mA.
    pub ma: u32,
    /// PPS only: minimum voltage, mV.
    pub min_mv: u32,
    pub is_pps: bool,
}

/// Latest Source_Capabilities message (up to 7 PDOs).
#[derive(Clone, Copy, Default)]
pub struct SourceCaps {
    pub n: usize,
    pub pdos: [Pdo; 7],
    /// PD spec revision of the message (header bits 15:14).
    pub spec_rev: u8,
    /// Bumped on every new Source_Capabilities message.
    pub updates: u32,
}

async fn wr(i2c: &mut I2c, reg: u8, val: u8) -> Result<(), Error> {
    let mut buf = [reg, val];
    i2c.transaction(FUSB, &mut [Operation::Write(&mut buf)]).await
}

async fn rd(i2c: &mut I2c, reg: u8, out: &mut [u8]) -> Result<(), Error> {
    let mut reg_buf = [reg];
    i2c.transaction(
        FUSB,
        &mut [Operation::Write(&mut reg_buf), Operation::Read(out)],
    )
    .await
}

/// Power up and configure the FUSB302 as a passive listener on both CC
/// lines (both orientations). Returns the device id (0x91 expected).
pub async fn init(i2c: &mut I2c) -> Result<u8, Error> {
    wr(i2c, REG_POWER, 0x0F).await?; // bandgap+osc+receiver+measure on
    wr(i2c, REG_RESET, 0x03).await?; // full reset (clears power reg)
    wr(i2c, REG_POWER, 0x0F).await?; // power back up after reset
    wr(i2c, REG_SWITCHES1, SWITCHES1_REC_CC1 | SWITCHES1_REC_CC2).await?;
    let mut id = [0u8];
    rd(i2c, REG_DEVICE_ID, &mut id).await?;
    Ok(id[0])
}

/// Poll for a received message; parse Source_Capabilities into `caps`.
/// Ok(true) = new source capabilities were stored.
pub async fn poll(i2c: &mut I2c, caps: &mut SourceCaps) -> Result<bool, Error> {
    let mut int = [0u8];
    rd(i2c, REG_INTERRUPT, &mut int).await?;
    if int[0] & (INT_RXSOP1 | INT_RXSOP2) == 0 {
        return Ok(false);
    }

    // FIFO order: receive token, 16-bit header, N data objects, CRC32.
    let mut head = [0u8; 3];
    rd(i2c, REG_FIFOS, &mut head).await?;
    let header = u16::from_le_bytes([head[1], head[2]]);
    let n = ((header >> 12) & 0x7) as usize;
    // Drain objects + CRC so the FIFO re-aligns for the next message.
    let mut rest = [0u8; 28 + 4];
    rd(i2c, REG_FIFOS, &mut rest[..n * 4 + 4]).await?;

    // SOP messages only (SOP'/SOP'' cable traffic never carries caps).
    if (head[0] >> 6) != 0b01 {
        return Ok(false);
    }
    // Data message type 1 with payload = Source_Capabilities.
    if n == 0 || (header & 0x1F) != 1 {
        return Ok(false);
    }

    caps.n = n.min(7);
    caps.spec_rev = ((header >> 14) & 0x3) as u8;
    for i in 0..caps.n {
        let o = &rest[i * 4..i * 4 + 4];
        let pdo = u32::from_le_bytes([o[0], o[1], o[2], o[3]]);
        caps.pdos[i] = parse_pdo(pdo);
    }
    caps.updates = caps.updates.wrapping_add(1);
    Ok(true)
}

fn parse_pdo(pdo: u32) -> Pdo {
    match pdo >> 30 {
        0b11 => Pdo {
            // PPS APDO: max V [24:17] 100 mV, min V [15:8] 100 mV, I 50 mA.
            mv: ((pdo >> 17) & 0xFF) * 100,
            min_mv: ((pdo >> 8) & 0xFF) * 100,
            ma: (pdo & 0x7F) * 50,
            is_pps: true,
        },
        0b00 => Pdo {
            // Fixed: V [19:10] 50 mV, I [9:0] 10 mA.
            mv: ((pdo >> 10) & 0x3FF) * 50,
            min_mv: 0,
            ma: (pdo & 0x3FF) * 10,
            is_pps: false,
        },
        // Variable / battery supplies: report the max V / current pair.
        _ => Pdo {
            mv: ((pdo >> 20) & 0x3FF) * 50,
            min_mv: ((pdo >> 10) & 0x3FF) * 50,
            ma: (pdo & 0x3FF) * 10,
            is_pps: false,
        },
    }
}

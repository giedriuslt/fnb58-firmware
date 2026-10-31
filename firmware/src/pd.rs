//! Passive USB-PD listener: FUSB302B (I2C 0x22) receive-only driver.
//!
//! The meter sits on the pass-through CC lines between source and DUT, so
//! it must stay electrically invisible: no AUTO_CRC (no GoodCRC — the real
//! sink answers), no Rd/VCONN switches, only the high-impedance measure/
//! receiver tap on one CC line at a time.
//!
//! Register map per the FUSB302B datasheet (matches the Linux fusb302
//! driver and the kws-x1 `fusb302-pd` crate, which cites the DS tables):
//! POWER 0x0B, RESET 0x0C, CONTROL1 0x07, STATUS1 0x41, FIFO 0x43
//! (auto-pops one byte per read).
//!
//! Receiving SOP messages needs three things wired up (DS): power to the
//! bandgap/receiver/oscillator, ENSOP1 in CONTROL1, and the measure block
//! routed to a CC line (SWITCHES0 MEAS_CC1/MEAS_CC2). The receiver only
//! hears the routed line, so the poll alternates lines while they look
//! quiet and sticks to one that shows BMC activity (I_ACTIVITY latch).

use embassy_time::Timer;
use embassy_at32::i2c::{Error, I2c};
use embedded_hal_async::i2c::I2c as _;
use embedded_hal_async::i2c::Operation;

const FUSB: u8 = 0x22;

const REG_DEVICE_ID: u8 = 0x01;
const REG_SWITCHES0: u8 = 0x02;
const REG_CONTROL1: u8 = 0x07;
const REG_POWER: u8 = 0x0b;
const REG_RESET: u8 = 0x0c;
const REG_STATUS1: u8 = 0x41;
const REG_INTERRUPT: u8 = 0x42;
const REG_FIFOS: u8 = 0x43;

/// POWER (0x0B): bandgap+wake, receiver, internal oscillator.
const POWER_ON: u8 = 0x01 | 0x02 | 0x08;
/// RESET (0x0C): reset the PD transmitter/receiver logic; self-clearing.
const PD_RESET: u8 = 0x02;
/// CONTROL1 (0x07): receive SOP packets / flush the RX FIFO (self-clearing).
const ENSOP1: u8 = 0x01;
const RX_FLUSH: u8 = 0x04;
/// SWITCHES0 (0x02): route CC1/CC2 to the measure block + BMC receiver.
const MEAS_CC1: u8 = 0x04;
const MEAS_CC2: u8 = 0x08;
/// INTERRUPT (0x42), read-clear: BMC activity started/stopped on the
/// measured CC line.
const I_ACTIVITY: u8 = 0x40;
/// STATUS1 (0x41): the RX FIFO holds a complete message.
const RX_EMPTY: u8 = 0x20;

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

/// Sticky state for the CC-line alternation between polls.
pub struct Listener {
    /// CC line currently routed to the receiver: 1 or 2.
    cc: u8,
    /// Consecutive polls with no BMC activity on the routed line.
    quiet: u32,
}

impl Default for Listener {
    fn default() -> Self {
        Self { cc: 1, quiet: 0 }
    }
}

/// Polls with no activity before trying the other CC line.
const QUIET_SWITCH_AFTER: u32 = 2;

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

/// Power up and configure the FUSB302 as a passive listener. Returns the
/// device id (0x9x for an FUSB302B).
pub async fn init(i2c: &mut I2c) -> Result<u8, Error> {
    // Reset the PD logic first so the config below starts from known
    // defaults: SWITCHES1 = 0 means AUTO_CRC off and no transmit driver —
    // the listener must stay silent on the wire.
    wr(i2c, REG_RESET, PD_RESET).await?;
    Timer::after_millis(1).await;
    wr(i2c, REG_POWER, POWER_ON).await?;
    // Route CC1 to the receiver (poll() re-routes as needed) and enable SOP
    // reception; the self-clearing flush drops anything from before reset.
    wr(i2c, REG_SWITCHES0, MEAS_CC1).await?;
    wr(i2c, REG_CONTROL1, ENSOP1 | RX_FLUSH).await?;
    let mut id = [0u8];
    rd(i2c, REG_DEVICE_ID, &mut id).await?;
    Ok(id[0])
}

/// Poll for a received message; parse Source_Capabilities into `caps`.
/// Ok(true) = new source capabilities were stored.
pub async fn poll(
    i2c: &mut I2c,
    caps: &mut SourceCaps,
    lis: &mut Listener,
) -> Result<bool, Error> {
    // Read-clear interrupt latch: BMC activity on the routed CC line.
    let mut int = [0u8];
    rd(i2c, REG_INTERRUPT, &mut int).await?;
    if int[0] & I_ACTIVITY != 0 {
        lis.quiet = 0;
    } else {
        lis.quiet += 1;
        if lis.quiet > QUIET_SWITCH_AFTER {
            // Nothing heard on this line; try the other one.
            lis.cc = if lis.cc == 1 { 2 } else { 1 };
            lis.quiet = 0;
            let meas = if lis.cc == 1 { MEAS_CC1 } else { MEAS_CC2 };
            wr(i2c, REG_SWITCHES0, meas).await?;
        }
    }

    let mut st1 = [0u8];
    rd(i2c, REG_STATUS1, &mut st1).await?;
    if st1[0] & RX_EMPTY != 0 {
        return Ok(false);
    }

    // FIFO order: SOP token, 16-bit header, N data objects, CRC32.
    let mut head = [0u8; 3];
    rd(i2c, REG_FIFOS, &mut head).await?;
    // The token's top three bits address the SOP* variant (DS Table 42);
    // 7 = SOP. Anything else is a non-SOP frame or a desynced FIFO.
    if head[0] >> 5 != 7 {
        wr(i2c, REG_CONTROL1, ENSOP1 | RX_FLUSH).await?;
        return Ok(false);
    }
    let header = u16::from_le_bytes([head[1], head[2]]);
    let n = ((header >> 12) & 0x7) as usize;
    let mut rest = [0u8; 28 + 4];
    rd(i2c, REG_FIFOS, &mut rest[..n * 4 + 4]).await?;

    // SOP data message type 1 with payload = Source_Capabilities; GoodCRC
    // and other control messages (n = 0) are ignored.
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

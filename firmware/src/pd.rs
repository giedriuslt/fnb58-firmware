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
const REG_STATUS0: u8 = 0x40;
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
/// INTERRUPT (0x42), read-clear: a received packet's CRC was checked.
const I_CRC_CHK: u8 = 0x10;
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
    /// PD spec revision of the message (header bits 7:6).
    pub spec_rev: u8,
    /// Bumped on every new Source_Capabilities message.
    pub updates: u32,
}

/// Sticky state for the CC-line scanning listener.
pub struct Listener {
    /// CC line currently routed to the receiver: 1 or 2.
    cc: u8,
    /// Ticks of lock remaining on the active CC; latched activity re-arms
    /// it. Scanning alternates lines only while unlocked.
    lock_ticks: u32,
    /// Latched I_ACTIVITY per CC since the last diagnostic print.
    activity: [bool; 2],
    /// Last STATUS0/STATUS1 read while that CC was routed.
    st0: [u8; 2],
    st1: [u8; 2],
    /// Last poll error as a one-digit diagnostic code (0 = none).
    err: u8,
    /// The CRC-check latch has vouched for the FIFO contents; frames may
    /// be parsed. Cleared when the FIFO drains empty.
    validated: bool,
    /// Ticks with non-empty, unvalidated FIFO bytes (stale-flush counter).
    stale: u32,
}

impl Default for Listener {
    fn default() -> Self {
        Self {
            cc: 1,
            lock_ticks: 0,
            activity: [false; 2],
            st0: [0; 2],
            st1: [0; 2],
            err: 0,
            validated: false,
            stale: 0,
        }
    }
}

/// Lock ticks after latched activity (1 s at the 100 ms poll cadence).
const LOCK_TICKS: u32 = 10;

/// Ticks of non-empty, unvalidated FIFO bytes before a flush (500 ms).
const STALE_FLUSH_TICKS: u32 = 5;

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

/// Poll one tick (100 ms cadence); parse Source_Capabilities into `caps`.
/// Scans both CC lines and locks onto one that shows latched BMC
/// activity. Ok(true) = new source capabilities were stored.
pub async fn poll(
    i2c: &mut I2c,
    caps: &mut SourceCaps,
    lis: &mut Listener,
) -> Result<bool, Error> {
    match poll_inner(i2c, caps, lis).await {
        Ok(v) => {
            lis.err = 0;
            Ok(v)
        }
        Err(e) => {
            lis.err = err_code(&e);
            Err(e)
        }
    }
}

fn err_code(e: &Error) -> u8 {
    match e {
        Error::Timeout => 1,
        Error::ArbitrationLoss => 2,
        Error::Bus => 3,
        Error::Overrun => 4,
        Error::ClockTimeout => 5,
        Error::NoAcknowledge(_) => 6,
    }
}

async fn poll_inner(
    i2c: &mut I2c,
    caps: &mut SourceCaps,
    lis: &mut Listener,
) -> Result<bool, Error> {
    // Read-clear interrupt latch: BMC activity and CRC validation on the
    // routed CC line, both latched since the last tick.
    let mut int = [0u8];
    rd(i2c, REG_INTERRUPT, &mut int).await?;
    if int[0] & I_ACTIVITY != 0 {
        let idx = (lis.cc - 1) as usize;
        lis.activity[idx] = true;
        lis.lock_ticks = LOCK_TICKS;
    } else if lis.lock_ticks > 0 {
        lis.lock_ticks -= 1;
    } else {
        // Scanning: dwell one tick per CC, then swap.
        lis.cc = if lis.cc == 1 { 2 } else { 1 };
        let meas = if lis.cc == 1 { MEAS_CC1 } else { MEAS_CC2 };
        wr(i2c, REG_SWITCHES0, meas).await?;
    }
    if int[0] & I_CRC_CHK != 0 {
        // A frame completed with a hardware CRC check; the FIFO now
        // holds at least one validated frame (possibly queued behind
        // older bytes).
        lis.validated = true;
    }

    // Snapshot the receiver state for the currently routed line.
    let idx = (lis.cc - 1) as usize;
    let mut st0 = [0u8];
    rd(i2c, REG_STATUS0, &mut st0).await?;
    let mut st1 = [0u8];
    rd(i2c, REG_STATUS1, &mut st1).await?;
    lis.st0[idx] = st0[0];
    lis.st1[idx] = st1[0];
    if st1[0] & RX_EMPTY != 0 {
        lis.stale = 0;
        lis.validated = false;
        return Ok(false);
    }
    if !lis.validated {
        // FIFO bytes that no CRC check has vouched for: a frame still
        // arriving, or one that failed validation. Give it a few ticks,
        // then flush so the FIFO can never wedge.
        lis.stale += 1;
        if lis.stale > STALE_FLUSH_TICKS {
            wr(i2c, REG_CONTROL1, ENSOP1 | RX_FLUSH).await?;
            lis.stale = 0;
        }
        return Ok(false);
    }
    lis.stale = 0;

    // Drain validated frames; Source_Capabilities may sit anywhere in
    // the queue behind GoodCRC responses.
    let mut caps_found = false;
    for _ in 0..4 {
        let mut st1b = [0u8];
        rd(i2c, REG_STATUS1, &mut st1b).await?;
        if st1b[0] & RX_EMPTY != 0 {
            lis.validated = false;
            break;
        }
        // FIFO order: SOP token, 16-bit header, N data objects, CRC32.
        let mut head = [0u8; 3];
        rd(i2c, REG_FIFOS, &mut head).await?;
        // The token's top three bits address the SOP* variant (DS Table
        // 42); 7 = SOP. Anything else is a non-SOP frame or a desynced
        // FIFO: flush so the next frame starts aligned.
        if head[0] >> 5 != 7 {
            wr(i2c, REG_CONTROL1, ENSOP1 | RX_FLUSH).await?;
            lis.validated = false;
            return Ok(caps_found);
        }
        let header = u16::from_le_bytes([head[1], head[2]]);
        let n = ((header >> 12) & 0x7) as usize;
        let mut rest = [0u8; 28 + 4];
        rd(i2c, REG_FIFOS, &mut rest[..n * 4 + 4]).await?;
        // SOP data message type 1 with payload = Source_Capabilities.
        // Rejected: extended messages (B15 set — Source_Capabilities_
        // Extended shares type 1 and would parse as garbage PDOs) and
        // control messages such as GoodCRC (n = 0).
        if header & 0x8000 != 0 || n == 0 || (header & 0x1F) != 1 {
            continue;
        }
        caps.n = n.min(7);
        // Specification revision is header bits 7:6 (00 = PD 2.0,
        // 01 = PD 3.0); bits 15:14 are the Extended flag plus the top
        // object-count bit.
        caps.spec_rev = ((header >> 6) & 0x3) as u8;
        for i in 0..caps.n {
            let o = &rest[i * 4..i * 4 + 4];
            let pdo = u32::from_le_bytes([o[0], o[1], o[2], o[3]]);
            caps.pdos[i] = parse_pdo(pdo);
        }
        caps.updates = caps.updates.wrapping_add(1);
        caps_found = true;
    }
    Ok(caps_found)
}

fn hex8(buf: &mut [u8], w: usize, v: u8) -> usize {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    buf[w] = HEX[(v >> 4) as usize];
    buf[w + 1] = HEX[(v & 0xF) as usize];
    w + 2
}

impl Listener {
    /// Diagnostic line: per-CC last STATUS0/STATUS1 (hex), the latched
    /// activity flag (a0/a1) and the last poll error code. STATUS0/
    /// STATUS1 reflect the line as of the last tick it was routed.
    pub fn write_diag(&self, buf: &mut [u8]) -> usize {
        let mut w = 0;
        for &b in b"[pd] " {
            buf[w] = b;
            w += 1;
        }
        for (i, cc) in [1u8, 2].iter().enumerate() {
            if i == 1 {
                buf[w] = b' ';
                w += 1;
            }
            buf[w] = b'c';
            buf[w + 1] = b'0' + cc;
            buf[w + 2] = b'=';
            w += 3;
            w = hex8(buf, w, self.st0[i]);
            buf[w] = b'/';
            w += 1;
            w = hex8(buf, w, self.st1[i]);
            buf[w] = b',';
            buf[w + 1] = b'a';
            buf[w + 2] = if self.activity[i] { b'1' } else { b'0' };
            buf[w + 3] = b',';
            buf[w + 4] = b'e';
            buf[w + 5] = b'0' + self.err;
            w += 6;
        }
        buf[w] = b'\r';
        buf[w + 1] = b'\n';
        w + 2
    }
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

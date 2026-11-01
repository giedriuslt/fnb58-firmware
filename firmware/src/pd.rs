//! USB-PD driver: FUSB302B (I2C 0x22). Receives Source_Capabilities and
//! participates as a sink: auto-GoodCRC (hardware), Get_Source_Cap and
//! Request messages sent from software.
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
//!
//! CAVEAT: AUTO_CRC acks EVERY incoming SOP message. With a DUT inline
//! that acks traffic meant for the DUT and corrupts its contract — this
//! driver is for standalone use (the meter IS the sink) until a passive
//! mode lands.

use embassy_time::Timer;
use embassy_at32::i2c::{Error, I2c};
use embedded_hal_async::i2c::I2c as _;
use embedded_hal_async::i2c::Operation;

const FUSB: u8 = 0x22;

const REG_DEVICE_ID: u8 = 0x01;
const REG_SWITCHES0: u8 = 0x02;
const REG_SWITCHES1: u8 = 0x03;
const REG_CONTROL1: u8 = 0x07;
const REG_CONTROL0: u8 = 0x06;
const REG_CONTROL3: u8 = 0x09;
const REG_POWER: u8 = 0x0b;
const REG_RESET: u8 = 0x0c;
const REG_STATUS0: u8 = 0x40;
const REG_STATUS1: u8 = 0x41;
const REG_INTERRUPT: u8 = 0x42;
const REG_INTERRUPTB: u8 = 0x3f;
const REG_INTERRUPTA: u8 = 0x3e;
const REG_FIFOS: u8 = 0x43;

/// SWITCHES1 (0x03): auto-GoodCRC for received packets, BMC transmit
/// driver on CC1/CC2, GoodCRC spec revision (SPECREV[1:0] = 01 at bits
/// 6:5 = revision 2.0 — without it we ack a 2.0/3.0 source with a
/// revision-1.0 GoodCRC and it ignores us).
const AUTO_CRC: u8 = 0x04;
const TXCC1_EN: u8 = 0x01;
const TXCC2_EN: u8 = 0x02;
const SPECREV_2_0: u8 = 0x20;
/// CONTROL3 (0x09): automatic retransmission while no GoodCRC arrives,
/// N_RETRIES[1:0] = 3 retries.
const AUTO_RETRY: u8 = 0x01;
/// CONTROL0 (0x06): flush the TX FIFO (self-clearing) — drop stale
/// bytes a previous attempt left behind.
const TX_FLUSH: u8 = 0x40;
/// INTERRUPTA (0x3E): a sent packet was GoodCRC'd / retries exhausted.
const I_TXSENT: u8 = 0x04;
const I_RETRYFAIL: u8 = 0x10;
/// STATUS0 (0x40): live BMC activity on the measured CC (never TX into
/// an incoming carrier).
const ST0_ACTIVITY: u8 = 0x40;

/// TX FIFO tokens (DS Table 41, via the kws-x1 fusb302-pd register map).
const TK_SYNC1: u8 = 0x12;
const TK_SYNC2: u8 = 0x13;
const TK_SYNC3: u8 = 0x1b;
const TK_JAM_CRC: u8 = 0xff;
const TK_EOP: u8 = 0x14;
const TK_TXOFF: u8 = 0xfe;
const TK_TXON: u8 = 0xa1;

/// POWER (0x0B): bandgap+wake, receiver, MEASURE block, oscillator —
/// set BEFORE the CC orientation read (kws-x1 never measures through an
/// unpowered measure block; r90 measured orientation at 0x09 and picked
/// the wrong CC).
const POWER_FULL: u8 = 0x01 | 0x02 | 0x04 | 0x08;
/// SWITCHES0 (0x02): device pull-downs = Rd, kept on BOTH CC lines at
/// all times (kws-x1: "keep Rd on both CCs"), plus the measure routing.
const PDWN1: u8 = 0x01;
const PDWN2: u8 = 0x02;
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
/// INTERRUPT (0x42), read-clear: a transmit collided with live CC
/// traffic (the packet is then NOT sent).
const I_COLLISION: u8 = 0x02;
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
    /// Ticks with non-empty, carrier-idle FIFO bytes before a flush.
    stale: u32,
    /// MessageID for the next transmitted message (wraps 0..=7).
    tx_id: u8,
    /// Spec revision bits for OUR message headers (header 7:6): 0x40 =
    /// 2.0, 0x80 = 3.0. The GoodCRC SPECREV in SWITCHES1 stays 2.0
    /// (kws-x1 keeps it fixed; GoodCRC echoes the received revision).
    tx_rev: u16,
    /// Last auto-scan outcome: true = the source acked our
    /// Get_Source_Cap. None until the first scan.
    last_scan: Option<bool>,
    /// Last transmission outcome: 0 = acked, 1 = RETRYFAIL (the TX left
    /// the chip and no GoodCRC came back), 2 = timeout with neither flag
    /// (the TX may not even have started). The r91 'no reply' hid this.
    last_tx: u8,
    /// The hardware auto-GoodCRC'ed at least one RECEIVED frame
    /// (INTERRUPTB I_GCRCSENT) — proof the source talked to us.
    gcrc: bool,
    /// A transmit collided with CC activity (INTERRUPT I_COLLISION).
    collided: bool,
    /// BC_LVL of both lines at the last orientation measure ([CC1, CC2]).
    last_meas: [u8; 2],
    /// Idle poll ticks since the last activity latch (idle re-check).
    idle: u32,
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
            stale: 0,
            tx_id: 0,
            tx_rev: 0x40,
            last_scan: None,
            last_tx: 0,
            gcrc: false,
            collided: false,
            last_meas: [0; 2],
            idle: 0,
        }
    }
}

/// Lock ticks after latched activity (1 s at the 100 ms poll cadence).
const LOCK_TICKS: u32 = 10;

/// Ticks of non-empty, unvalidated FIFO bytes before a flush (500 ms).
const STALE_FLUSH_TICKS: u32 = 5;

/// Control message types (USB PD Table 6-13 / kernel pd.h): 5 is Ping,
/// 7 is Get_Source_Cap — r88-r98 sent Pings, which the PHY GoodCRC'd
/// and the policy engine rightly ignored.
const CTRL_GET_SOURCE_CAP: u16 = 7;
/// Soft_Reset returns the source's protocol layer to ready (Accept
/// comes back, hardware-acked) and re-arms its caps announce.
const CTRL_SOFT_RESET: u16 = 13;
/// Data message types (USB PD Table 6-6).
const DATA_REQUEST: u16 = 2;

/// Transmit one message (header + optional 4-byte object) and wait for
/// the hardware to report the GoodCRC (TXSENT) or exhausted retries
/// (RETRYFAIL). The TX driver routing and AUTO_CRC come from init().
/// Ok(true) = acknowledged.
async fn tx_message(
    i2c: &mut I2c,
    lis: &mut Listener,
    msg_type: u16,
    payload: &[u8],
) -> Result<bool, Error> {
    // Never transmit into an incoming carrier (garbles both directions).
    for _ in 0..20 {
        let mut st0 = [0u8];
        rd(i2c, REG_STATUS0, &mut st0).await?;
        if st0[0] & ST0_ACTIVITY == 0 {
            break;
        }
        Timer::after_millis(1).await;
    }
    // Drop any stale bytes a previous attempt left in the TX FIFO.
    wr(i2c, REG_CONTROL0, TX_FLUSH).await?;

    // r99: clear stale INTERRUPTA flags first — a late TXSENT from an
    // earlier timed-out attempt would otherwise be read as THIS
    // message's ack.
    let mut stale = [0u8; 1];
    let _ = rd(i2c, REG_INTERRUPTA, &mut stale).await;

    // Header: type, Number of Data Objects (r99: the field was never
    // set — a Request went out as a 0-object GotoMin with 4 stray
    // bytes), MessageID, sink power role, spec revision (bits 7:6:
    // 0x40 = 2.0, 0x80 = 3.0).
    let ndo = (payload.len() / 4) as u16;
    let header: u16 = msg_type
        | (ndo << 12)
        | ((lis.tx_id as u16 & 0x7) << 9)
        | lis.tx_rev;
    lis.tx_id = lis.tx_id.wrapping_add(1);
    let [h0, h1] = header.to_le_bytes();
    // TX FIFO: SOP tokens (USB PD: SOP = three Sync-1 + one Sync-2 —
    // kws-x1 phy.rs &[SOP1, SOP1, SOP1, SOP2]; SYNC2/SYNC3 in the
    // preamble was NEVER a valid SOP — the source silently ignored every
    // frame, the whole r84-r90 'no reply'), PACKSYM (byte count = header
    // + payload), message bytes, CRC jam, EOP, driver off, transmit.
    let mut fifo = [0u8; 16];
    let mut w = 0;
    for &b in &[
        TK_SYNC1, TK_SYNC1, TK_SYNC1, TK_SYNC2,
        0x80 | (2 + payload.len() as u8),
    ] {
        fifo[w] = b;
        w += 1;
    }
    fifo[w] = h0;
    fifo[w + 1] = h1;
    w += 2;
    for &b in payload {
        fifo[w] = b;
        w += 1;
    }
    for &b in &[TK_JAM_CRC, TK_EOP, TK_TXOFF, TK_TXON] {
        fifo[w] = b;
        w += 1;
    }
    wrb(i2c, REG_FIFOS, &fifo[..w]).await?;

    // Wait for the hardware to report the GoodCRC (TXSENT) or exhausted
    // retries (RETRYFAIL); record WHICH outcome — 'no reply' hid the
    // difference (retryfail = TX left the chip, timeout = it may not
    // even have started).
    lis.last_tx = 2;
    for _ in 0..50 {
        let mut a = [0u8; 1];
        rd(i2c, REG_INTERRUPTA, &mut a).await?;
        if a[0] & I_TXSENT != 0 {
            lis.last_tx = 0;
            return Ok(true);
        }
        if a[0] & I_RETRYFAIL != 0 {
            lis.last_tx = 1;
            return Ok(false);
        }
        Timer::after_millis(5).await;
    }
    Ok(false)
}

/// Send a Request for the first fixed PDO in `caps` at its full maximum
/// current. RDO (usbpd crate layout): object position 28..=31,
/// no_usb_suspend bit 24, operating current 10..=19, max operating
/// current 0..=9 — both in 10 mA units.
async fn request_first_fixed(
    i2c: &mut I2c,
    caps: &SourceCaps,
    lis: &mut Listener,
) -> Result<bool, Error> {
    if let Some(i) = (0..caps.n).find(|&i| !caps.pdos[i].is_pps) {
        let ma10 = (caps.pdos[i].ma / 10) as u16;
        let rdo: u32 = ((i as u32 + 1) << 28)
            | (1 << 25) // no_usb_suspend — RDO_NO_USB_SUSP is BIT(25);
            // we set BIT(24) = Unchunked_Extended_Messages, a PD 3.0
            // field a 2.0 source can reject the Request over (r101)
            | ((ma10 as u32) << 10)
            | (ma10 as u32);
        let bytes = rdo.to_le_bytes();
        return tx_message(i2c, lis, DATA_REQUEST, &bytes).await;
    }
    Ok(false)
}

/// Tester-initiated discovery: Get_Source_Cap, capture the response
/// caps, then Request the first fixed PDO so the source establishes a
/// contract and stops hard-resetting VBUS (a compliant source power-cycles
/// an unresponsive sink after nCapsCount retries). Requires the PD switch
/// ON (hardware Rd) and no DUT inline (our auto-GoodCRC would ack traffic
/// meant for the DUT). Ok(true) = caps captured (and requested, when a
/// fixed PDO exists).
/// Capture window: drain complete frames (FIFO non-empty AND the CC
/// carrier ended — STATUS0 ACTIVITY) until Source_Capabilities shows
/// up; store it and send the Request. Idle sleeps are 2 ms: after the
/// source's caps are GoodCRC'd, the Request must land within
/// tSenderResponse (24-30 ms). Returns true when caps were stored.
async fn capture_window(
    i2c: &mut I2c,
    caps: &mut SourceCaps,
    lis: &mut Listener,
    iters: u32,
) -> Result<bool, Error> {
    for _ in 0..iters {
        let mut st1 = [0u8; 1];
        rd(i2c, REG_STATUS1, &mut st1).await?;
        let mut st0 = [0u8; 1];
        rd(i2c, REG_STATUS0, &mut st0).await?;
        if st1[0] & RX_EMPTY != 0 || st0[0] & ST0_ACTIVITY != 0 {
            Timer::after_millis(2).await;
            continue;
        }
        // Carrier idle, complete frame queued: drain one frame.
        let mut head = [0u8; 3];
        rd(i2c, REG_FIFOS, &mut head).await?;
        if head[0] >> 5 != 7 {
            wr(i2c, REG_CONTROL1, ENSOP1 | RX_FLUSH).await?;
            return Ok(false);
        }
        let header = u16::from_le_bytes([head[1], head[2]]);
        let n = ((header >> 12) & 0x7) as usize;
        let mut rest = [0u8; 28 + 4];
        rd(i2c, REG_FIFOS, &mut rest[..n * 4 + 4]).await?;
        // GoodCRC and Accept are control messages (n = 0) — keep
        // draining until the caps appear.
        if header & 0x8000 != 0 || n == 0 || (header & 0x1F) != 1 {
            continue;
        }

        caps.n = n.min(7);
        caps.spec_rev = ((header >> 6) & 0x3) as u8;
        for i in 0..caps.n {
            let o = &rest[i * 4..i * 4 + 4];
            let pdo = u32::from_le_bytes([o[0], o[1], o[2], o[3]]);
            caps.pdos[i] = parse_pdo(pdo);
        }
        caps.updates = caps.updates.wrapping_add(1);

        // Request the first fixed PDO (usually 5 V); the source answers
        // Accept + PS_RDY, our hardware acks those automatically, and the
        // contract stands at the PDO's voltage.
        let _ = request_first_fixed(i2c, caps, lis).await?;
        return Ok(true);
    }
    Ok(false)
}

pub async fn discover(
    i2c: &mut I2c,
    caps: &mut SourceCaps,
    lis: &mut Listener,
) -> Result<bool, Error> {
    // r103: re-measure the orientation first — the r102 session saw the
    // port with NO Rp at all (charger detached); the scan must follow
    // the Rp wherever it is right now.
    let (meas, cc) = measure_cc(i2c).await?;
    lis.last_meas = meas;
    let _ = route(i2c, cc).await;
    lis.set_cc(cc);

    // r104: Soft_Reset first (user suggestion + spec): the source's
    // protocol layer returns to ready, its Accept is hardware-acked,
    // and MessageIDs reset to zero on both sides — the r103 session
    // showed the FIRST scan after a fresh init works and everything
    // after that fails to transmit (r=2), so every scan now rebuilds
    // the whole state (init in main.rs) and re-syncs the protocol.
    let _ = tx_message(i2c, lis, CTRL_SOFT_RESET, &[]).await?;
    lis.tx_id = 0;
    Timer::after_millis(10).await;
    let acked = tx_message(i2c, lis, CTRL_GET_SOURCE_CAP, &[]).await?;
    lis.last_scan = Some(acked);
    if !acked {
        return Ok(false);
    }

    // First window: an alert source answers a real Get_Source_Cap
    // directly.
    if capture_window(i2c, caps, lis, 250).await? {
        return Ok(true);
    }

    // r101: no answer — force a fresh attach. A source whose policy
    // engine slept after an unanswered announce (r100: the receiver
    // caught exactly ONE spontaneous caps frame during the user's
    // switch toggles) ignores Get_Source_Cap forever; the spec's way
    // back is a NEW ATTACH. Removing our internal Rd for a moment is a
    // real detach when the board's switch-gated Rd is OFF (the source
    // drops VBUS, then re-attaches and announces into our armed
    // receiver); with the switch ON the board Rd keeps the attach and
    // the pulse is a no-op.
    let meas = if lis.cc == 1 { MEAS_CC1 } else { MEAS_CC2 };
    wr(i2c, REG_SWITCHES0, meas).await?;
    Timer::after_millis(400).await;
    wr(i2c, REG_SWITCHES0, PDWN1 | PDWN2 | meas).await?;

    // Second window: the fresh attach announces within a few hundred ms.
    capture_window(i2c, caps, lis, 600).await
}

async fn wr(i2c: &mut I2c, reg: u8, val: u8) -> Result<(), Error> {
    let mut buf = [reg, val];
    i2c.transaction(FUSB, &mut [Operation::Write(&mut buf)]).await
}

/// Block write (TX FIFO and other multi-byte registers). Buffer sized
/// for the largest frame: SOP tokens x4 + PACKSYM + 2-byte header + 7
/// objects (28) + JAM/EOP/TXOFF/TXON = 40 — a Request (15 bytes) PANICked
/// the old 13-byte buffer the moment TX started working.
async fn wrb(i2c: &mut I2c, reg: u8, data: &[u8]) -> Result<(), Error> {
    let mut buf = [0u8; 41];
    assert!(data.len() <= 40);
    buf[0] = reg;
    buf[1..1 + data.len()].copy_from_slice(data);
    i2c.transaction(
        FUSB,
        &mut [Operation::Write(&mut buf[..1 + data.len()])],
    )
    .await
}

async fn rd(i2c: &mut I2c, reg: u8, out: &mut [u8]) -> Result<(), Error> {
    let mut reg_buf = [reg];
    i2c.transaction(
        FUSB,
        &mut [Operation::Write(&mut reg_buf), Operation::Read(out)],
    )
    .await
}

/// Measure BC_LVL on both CC lines (internal Rd on both, 10 ms settle
/// each) and pick the source-driven line. Returns ([CC1, CC2] levels,
/// pick). r103: shared by init, every scan and the idle re-check — the
/// r102 session saw a port with NO Rp anywhere (charger detached); a
/// source that comes back on the other line must be followed.
pub(crate) async fn measure_cc(i2c: &mut I2c) -> Result<([u8; 2], u8), Error> {
    let mut st = [0u8; 1];
    wr(i2c, REG_SWITCHES0, PDWN1 | PDWN2 | MEAS_CC1).await?;
    Timer::after_millis(10).await;
    rd(i2c, REG_STATUS0, &mut st).await?;
    let cc1 = st[0] & 0x03;
    wr(i2c, REG_SWITCHES0, PDWN1 | PDWN2 | MEAS_CC2).await?;
    Timer::after_millis(10).await;
    rd(i2c, REG_STATUS0, &mut st).await?;
    let cc2 = st[0] & 0x03;
    let cc = if cc1 != 0 || cc2 == 0 { 1 } else { 2 };
    Ok(([cc1, cc2], cc))
}

/// Power up and configure the FUSB302, kws-x1-faithful (phy.rs sink
/// start): full POWER (incl. the measure block) before anything is
/// routed, CC orientation MEASURED (Rp on the source-driven line) with
/// the internal Rd on both lines, then the BMC TX driver + auto-GoodCRC
/// on the SELECTED line only. Returns (device id, CC).
pub async fn init(i2c: &mut I2c) -> Result<(u8, u8), Error> {
    // Reset the PD logic first so the config below starts from known
    // defaults.
    wr(i2c, REG_RESET, PD_RESET).await?;
    Timer::after_millis(1).await;
    wr(i2c, REG_POWER, POWER_FULL).await?;

    let (levels, cc) = measure_cc(i2c).await?;

    // Route the (now powered) measure block and arm the sink: auto-CRC
    // rev 2.0 GoodCRC + hardware retries, BMC TX driver on the selected
    // line ONLY.
    let meas = if cc == 1 { MEAS_CC1 } else { MEAS_CC2 };
    wr(i2c, REG_SWITCHES0, PDWN1 | PDWN2 | meas).await?;
    let txc = if cc == 1 { TXCC1_EN } else { TXCC2_EN };
    wr(i2c, REG_SWITCHES1, AUTO_CRC | SPECREV_2_0 | txc).await?;
    wr(i2c, REG_CONTROL3, AUTO_RETRY | (0b11 << 1)).await?;
    wr(i2c, REG_POWER, POWER_FULL).await?;
    // Receive SOP packets; the flush drops anything from before the
    // receiver was powered.
    wr(i2c, REG_CONTROL1, ENSOP1 | RX_FLUSH).await?;
    let mut id = [0u8; 1];
    rd(i2c, REG_DEVICE_ID, &mut id).await?;
    Ok((id[0], cc))
}

/// One-shot receiver snapshot: (STATUS0, STATUS1). Read-only — no
/// transmit, no CC re-routing; whatever CC line init() routed is shown.
pub async fn status(i2c: &mut I2c) -> Result<(u8, u8), Error> {
    let mut st0 = [0u8];
    rd(i2c, REG_STATUS0, &mut st0).await?;
    let mut st1 = [0u8];
    rd(i2c, REG_STATUS1, &mut st1).await?;
    Ok((st0[0], st1[0]))
}

/// Route the receiver AND the BMC TX driver to one CC line (1 or 2),
/// kws-x1 SWITCHES0/1 form: internal Rd stays on both, measure block on
/// `cc`, TX driver on `cc`, RX FIFO flushed for the new line. r93: each
/// scan picks its line, because the r91/r92 evidence (self-TX heard on
/// CC1 when both drivers were armed, silence on CC2 with TXCC2 alone)
/// leaves the TX electrical path as the open question.
pub async fn route(i2c: &mut I2c, cc: u8) -> Result<(), Error> {
    let meas = if cc == 1 { MEAS_CC1 } else { MEAS_CC2 };
    wr(i2c, REG_SWITCHES0, PDWN1 | PDWN2 | meas).await?;
    let txc = if cc == 1 { TXCC1_EN } else { TXCC2_EN };
    wr(i2c, REG_SWITCHES1, AUTO_CRC | SPECREV_2_0 | txc).await?;
    wr(i2c, REG_CONTROL1, ENSOP1 | RX_FLUSH).await?;
    Ok(())
}

/// Raw-drain up to 8 RX FIFO bytes as hex into `out` (returns the chars
/// written; 0 = the FIFO was empty), then flush the FIFO. Diagnostic for
/// a scan that acked without capturing caps: shows whether the source
/// sent ANYTHING and what (raw bytes: SOP token, header lo/hi, objects).
pub async fn rx_dump(i2c: &mut I2c, out: &mut [u8]) -> usize {
    let mut st1 = [0u8; 1];
    if rd(i2c, REG_STATUS1, &mut st1).await.is_err() || st1[0] & RX_EMPTY != 0 {
        return 0;
    }
    let mut raw = [0u8; 8];
    if rd(i2c, REG_FIFOS, &mut raw).await.is_err() {
        return 0;
    }
    let mut w = 0;
    for &b in raw.iter() {
        if w + 2 <= out.len() {
            w = hex8(out, w, b);
        }
    }
    let _ = wr(i2c, REG_CONTROL1, ENSOP1 | RX_FLUSH).await;
    w
}

/// Poll one tick (100 ms cadence in r85+ / 10 ms now); parse
/// Source_Capabilities into `caps`. The receiver stays on the CC line
/// init() selected. Ok(true) = new source capabilities were stored.
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
    // routed CC line, both latched since the last tick. I_COLLISION = a
    // transmit ran into live CC traffic (the packet is then NOT sent).
    let mut int = [0u8; 1];
    rd(i2c, REG_INTERRUPT, &mut int).await?;
    if int[0] & I_COLLISION != 0 {
        lis.collided = true;
    }
    if int[0] & I_ACTIVITY != 0 {
        let idx = (lis.cc - 1) as usize;
        lis.activity[idx] = true;
        lis.lock_ticks = LOCK_TICKS;
        lis.idle = 0;
    } else if lis.lock_ticks > 0 {
        lis.lock_ticks -= 1;
    }
    // No CC alternation: init() measured the orientation and the
    // receiver stays on that line (kws-x1 never toggles after selection;
    // swapping away from the source's line only loses ticks).
    // I_GCRCSENT (read-clear): the hardware sent a GoodCRC for an
    // incoming frame — the source DID talk to us at least once.
    let mut intb = [0u8; 1];
    rd(i2c, REG_INTERRUPTB, &mut intb).await?;
    if intb[0] & 0x01 != 0 {
        lis.gcrc = true;
    }

    // Snapshot the receiver state for the currently routed line.
    let idx = (lis.cc - 1) as usize;
    let mut st0 = [0u8; 1];
    rd(i2c, REG_STATUS0, &mut st0).await?;
    let mut st1 = [0u8; 1];
    rd(i2c, REG_STATUS1, &mut st1).await?;
    lis.st0[idx] = st0[0];
    lis.st1[idx] = st1[0];

    // r103: while nothing has EVER been captured, re-check the
    // orientation every ~1 s of idle — the poll and the INT pin only
    // hear the routed line, so a source that re-attaches on the OTHER
    // line must be followed (the r102 session listened on a dead line).
    if caps.updates == 0 {
        lis.idle += 1;
        if lis.idle >= 100 {
            lis.idle = 0;
            let (meas, cc) = measure_cc(i2c).await?;
            lis.last_meas = meas;
            if cc != lis.cc {
                lis.cc = cc;
                let m = if cc == 1 { MEAS_CC1 } else { MEAS_CC2 };
                wr(i2c, REG_SWITCHES0, PDWN1 | PDWN2 | m).await?;
            }
        }
    }

    if st1[0] & RX_EMPTY != 0 {
        lis.stale = 0;
        return Ok(false);
    }
    if st0[0] & ST0_ACTIVITY != 0 {
        // A frame is still arriving — only drain complete ones (r100:
        // the old I_CRC_CHK 'validated' latch never cleared and could
        // admit a half-received frame).
        return Ok(false);
    }
    lis.stale += 1;
    if lis.stale > STALE_FLUSH_TICKS {
        // Non-empty with no carrier for many ticks: a wedged FIFO.
        wr(i2c, REG_CONTROL1, ENSOP1 | RX_FLUSH).await?;
        lis.stale = 0;
        return Ok(false);
    }

    // Drain complete frames; Source_Capabilities may sit anywhere in
    // the queue behind GoodCRC responses.
    let mut caps_found = false;
    for _ in 0..4 {
        let mut st1b = [0u8];
        rd(i2c, REG_STATUS1, &mut st1b).await?;
        if st1b[0] & RX_EMPTY != 0 {
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
        caps.spec_rev = ((header >> 6) & 0x3) as u8;
        for i in 0..caps.n {
            let o = &rest[i * 4..i * 4 + 4];
            let pdo = u32::from_le_bytes([o[0], o[1], o[2], o[3]]);
            caps.pdos[i] = parse_pdo(pdo);
        }
        caps.updates = caps.updates.wrapping_add(1);
        caps_found = true;
        // Sink protocol reply: EVERY Source_Capabilities gets a Request
        // within tSenderResponse (~24-30 ms) or the source hard-resets
        // VBUS. r100: report CAPS FOUND regardless of the Request's ack
        // status — the old code returned the Request outcome and the
        // caller then never displayed a captured caps frame.
        let _ = request_first_fixed(i2c, caps, lis).await?;
        return Ok(true);
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
    /// Point the CC scan at the line init() measured as the source side
    /// (1 or 2); the scan then only alternates while the line is quiet.
    pub fn set_cc(&mut self, cc: u8) {
        self.cc = cc;
    }

    /// The last transmission outcome code (0 acked / 1 RETRYFAIL /
    /// 2 never started) for the per-scan console line.
    pub fn last_tx_code(&self) -> u8 {
        self.last_tx
    }

    /// Whether the hardware ever auto-GoodCRC'ed an incoming frame.
    pub fn gcrc_seen(&self) -> bool {
        self.gcrc
    }

    /// BC_LVL of both lines at the last orientation measure ([CC1, CC2]).
    pub fn meas(&self) -> [u8; 2] {
        self.last_meas
    }

    /// Whether any transmit collided with live CC traffic.
    pub fn collided(&self) -> bool {
        self.collided
    }

    /// Diagnostic line: per-CC last STATUS0/STATUS1 (hex), the latched
    /// activity flag (a0/a1) and the last poll error code. STATUS0/
    /// STATUS1 reflect the line as of the last tick it was routed.
    pub fn write_diag(&self, buf: &mut [u8], pos: usize) -> usize {
        let mut w = pos;
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
        // Last auto-scan outcome: 1 = source acked, 0 = no reply, -= never.
        buf[w] = b',';
        buf[w + 1] = b's';
        buf[w + 2] = match self.last_scan {
            Some(true) => b'1',
            Some(false) => b'0',
            None => b'-',
        };
        w += 3;
        // TX outcome (0 acked / 1 retryfail / 2 never started), whether
        // the hardware ever GoodCRC'ed an incoming frame (g), and
        // whether any transmit collided with live CC traffic (x).
        buf[w] = b',';
        buf[w + 1] = b'r';
        buf[w + 2] = b'=';
        buf[w + 3] = b'0' + self.last_tx;
        buf[w + 4] = b',';
        buf[w + 5] = b'g';
        buf[w + 6] = if self.gcrc { b'1' } else { b'0' };
        buf[w + 7] = b',';
        buf[w + 8] = b'x';
        buf[w + 9] = if self.collided { b'1' } else { b'0' };
        w += 10;
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

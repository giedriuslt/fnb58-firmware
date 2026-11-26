//! The PD sink session task: starts the CC diagnostic and the sink session
//! at boot (kws-x1's order — the charger's caps window opens at attach),
//! then serves OK presses with further one-shot sessions, all from the
//! vendored `fusb302-pd` + `pd-protocol` stack, and publishes a Copy view
//! for the UI.
//!
//! The I2C1 bus is shared with the meter through `BUS` (one Mutex lease per
//! transaction; each transfer spins to completion inside its lease, so the
//! session's SenderResponseTimer survives even a 31-byte FIFO read).

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU16, AtomicU32, AtomicUsize, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::Timer;
use fusb302_pd::observation::{Observation, Phase};
use fusb302_pd::startup::Report;
use pd_protocol::pd_sniff as _pd_sniff_types;
use pd_protocol::pd_spr;

use crate::console::PD_STAGE as STAGE;
use crate::pd_phy::{self, FnbLimits, MEASURE, SharedBus, wait_for_rx_idle};

/// The HAL I2c handle, boxed for the shared bus. `I2c` is `!Send`/`!Sync`
/// (svd2rust register blocks contain UnsafeCell); single-core firmware
/// reaches it only under the critical-section mutex below, so the
/// `unsafe impl Sync` on the bus wrapper is sound.
pub struct BusCell(pub Option<embassy_at32::i2c::I2c>);
/// The shared I2C1 bus: created once in main, leased per transaction (the
/// meter gets turns between PD transfers).
pub struct Bus {
    inner: Mutex<CriticalSectionRawMutex, BusCell>,
}
// SAFETY: single core, every access under the critical-section mutex.
unsafe impl Sync for Bus {}
impl Bus {
    pub const fn new() -> Self {
        Self {
            inner: Mutex::new(BusCell(None)),
        }
    }
    pub async fn lock(&'static self) -> embassy_sync::mutex::MutexGuard<'static, CriticalSectionRawMutex, BusCell> {
        self.inner.lock().await
    }
}
pub static BUS: Bus = Bus::new();
/// OK on the PD screen asks the task to run a scan + sink session.
pub static START: Signal<CriticalSectionRawMutex, ()> = Signal::new();
/// Pressed-button bitmask from the main loop (bit 0 = BACK held).
pub static BTN: AtomicU8 = AtomicU8::new(0);

/// r171: the user-selected request target, cycled with LEFT/RIGHT on the
/// PPS screen (pd_task::CHOICES_MV). The boot session arms it as-is, so
/// the boot contract is whatever was last chosen (default 5 V).
pub static DESIRED_MV: AtomicU16 = AtomicU16::new(5000);

/// r235: the PPS request the PPS screen commits — the operating current
/// and the PPS-kind flag (DESIRED_MV carries the voltage). The control
/// closure builds `Target { mv, ma, Supply::Pps(pos) }` from them;
/// preview() validates against the engine's own offered range.
pub static DESIRED_MA: AtomicU16 = AtomicU16::new(1000);
pub static DESIRED_PPS: AtomicBool = AtomicBool::new(false);

/// r229: the advertised FIXED profiles of the last caps (the PD
/// PROFILES screen's rows — the PD-native equivalents of the QC
/// target list). Populated at the caps receive site below; the fixed
/// objects only (PPS/APB objects have no `fixed` pair and are skipped).
pub static CAPS_N: AtomicUsize = AtomicUsize::new(0);
pub static CAPS_MV: [AtomicU16; 6] = [
    AtomicU16::new(0),
    AtomicU16::new(0),
    AtomicU16::new(0),
    AtomicU16::new(0),
    AtomicU16::new(0),
    AtomicU16::new(0),
];
pub static CAPS_MA: [AtomicU16; 6] = [
    AtomicU16::new(0),
    AtomicU16::new(0),
    AtomicU16::new(0),
    AtomicU16::new(0),
    AtomicU16::new(0),
    AtomicU16::new(0),
];
/// r235: the advertised PPS ranges of the last caps (the PPS screen) —
/// one entry per SPR PPS APDO, with its object position (1..=7, the
/// engine's Supply::Pps argument). Populated at the caps store site
/// from the raw PDO word, kws-x1's describe_pdo bit layout verbatim
/// (min bits 8..15 and max bits 17..24 in 100 mV units, current bits
/// 0..6 in 50 mA units, APDO kind 0b1100 in bits 31..28).
pub static PPS_N: AtomicUsize = AtomicUsize::new(0);
pub static PPS_OBJ: [AtomicU8; 4] = [AtomicU8::new(0), AtomicU8::new(0), AtomicU8::new(0), AtomicU8::new(0)];
pub static PPS_MIN_MV: [AtomicU16; 4] = [AtomicU16::new(0), AtomicU16::new(0), AtomicU16::new(0), AtomicU16::new(0)];
pub static PPS_MAX_MV: [AtomicU16; 4] = [AtomicU16::new(0), AtomicU16::new(0), AtomicU16::new(0), AtomicU16::new(0)];
pub static PPS_MAX_MA: [AtomicU16; 4] = [AtomicU16::new(0), AtomicU16::new(0), AtomicU16::new(0), AtomicU16::new(0)];

/// r236: the caps evidence — the observation's count plus EVERY object
/// slot's raw word (filled regardless of count, so a dropped APDO is
/// visible), dumped by the console 'q' command.
pub static CAPS_COUNT: AtomicU8 = AtomicU8::new(0);
pub static CAPS_RAW: [core::sync::atomic::AtomicU32; 7] = [
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
];

/// The selectable fixed targets (the PD screen cycles through these).
pub const CHOICES_MV: [u16; 5] = [5000, 9000, 12000, 15000, 20000];

/// Cycle [`DESIRED_MV`] by one step in `dir` (+1 right / -1 left), with
/// wraparound. Returns the new value for the console line.
pub fn cycle_desired(dir: i16) -> u16 {
    let current = DESIRED_MV.load(Ordering::Relaxed);
    let mut index = CHOICES_MV
        .iter()
        .position(|&mv| mv == current)
        .unwrap_or(0);
    index = (index as i16 + dir).rem_euclid(CHOICES_MV.len() as i16) as usize;
    let next = CHOICES_MV[index];
    DESIRED_MV.store(next, Ordering::Relaxed);
    next
}

/// r173: set by the PD screen's OK-commit. The engine's control callback
/// then previews+confirms the target on its next poll — a plain Request
/// from Ready (the PD-native voltage change; PPS re-requests the same
/// way). NO Hard Reset, NO session restart. Browsing (LEFT/RIGHT) never
/// transmits — that is the Selection API's own contract ("browsing alone
/// never schedules transmission").
pub static COMMIT: AtomicBool = AtomicBool::new(false);

/// Arm a FIXED re-request (the PD PROFILES screen's OK). Clears the PPS
/// kind so the control closure's fixed match runs.
pub fn commit_desired() {
    DESIRED_PPS.store(false, Ordering::Relaxed);
    COMMIT.store(true, Ordering::Relaxed);
}

/// r235: arm a PPS request (the PPS screen's OK) — mv within an
/// advertised range, ma the operating current. preview() validates both
/// against the engine's own range (20 mV / 50 mA quanta, 1 A minimum).
pub fn commit_pps(mv: u16, ma: u16) {
    DESIRED_MV.store(mv, Ordering::Relaxed);
    DESIRED_MA.store(ma, Ordering::Relaxed);
    DESIRED_PPS.store(true, Ordering::Relaxed);
    COMMIT.store(true, Ordering::Relaxed);
}

// --- r238: the CABLE screen (kws-x1's Detail::Cable page) -------------
// The vendored stack already carries the whole feature — the engine's
// Selection::request_cable/request_cable_mode arming, the FUSB302's SOP'
// inspection (Soft_Reset sync, Discover Identity with the PD3/PD2
// fallbacks, Discover SVIDs, Get_Manufacturer_Info, the cable and SOP''
// statuses — pd_discovery behind phy::discover_cable) and the pd_svids
// bitmap store; the FnbPlatform's INSPECTION_CC/INSPECTION_SWITCHES0 were
// in place since the crate copy. This is the trigger + view wiring.

/// The cable's SVID set (kws-x1's CABLE_SVIDS): the vendored discovery
/// fills it from Discover SVIDs replies; the page and the scroll-driven
/// mode requests read it back. 8 KiB of bitmap.
static CABLE_SVIDS: embassy_sync::blocking_mutex::Mutex<
    CriticalSectionRawMutex,
    RefCell<pd_protocol::pd_svids::Svids>,
> = embassy_sync::blocking_mutex::Mutex::new(RefCell::new(
    pd_protocol::pd_svids::Svids::EMPTY,
));
/// The `pd_svids::Store` handle the session config gets (kws-x1's
/// CableSvidStore).
pub struct CableSvidStore;
impl pd_protocol::pd_svids::Store for CableSvidStore {
    fn clear(&self) {
        CABLE_SVIDS.lock(|cell| cell.borrow_mut().clear());
    }
    fn response(&self, objects: &[u32]) -> Option<pd_protocol::pd_svids::Progress> {
        CABLE_SVIDS.lock(|cell| cell.borrow_mut().response(objects))
    }
    fn at(&self, index: u16) -> Option<u16> {
        CABLE_SVIDS.lock(|cell| cell.borrow().at(index))
    }
    fn contains(&self, svid: u16) -> bool {
        CABLE_SVIDS.lock(|cell| cell.borrow().contains(svid))
    }
}
/// The store handle the cable page reads the MODES row's names through.
pub(crate) static CABLE_SVID_STORE: CableSvidStore = CableSvidStore;

/// The browse mode's OK asked for a cable read (kws-x1's long-OK →
/// PdCommand::QueryCable; fnb58 has no long-press, the r226 energy-reset
/// precedent). Consumed by the control closure.
pub static CABLE_ASK: AtomicBool = AtomicBool::new(false);
/// An ask happened since boot (kws-x1's CABLE_ASKED): a refused query
/// explains itself only after one.
pub static CABLE_ASKED: AtomicBool = AtomicBool::new(false);
/// The ask waits for the 5 V contract it requested first (kws-x1's
/// Selection::after_5v — the engine only inspects from fixed 5 V).
static CABLE_AFTER_5V: AtomicBool = AtomicBool::new(false);
/// When the waiting 5 V contract was first seen, ms (kws-x1's
/// five_volts_since; FIVE_VOLT_SETTLE_MS = 300).
static CABLE_5V_MS: AtomicU32 = AtomicU32::new(0);
/// An inspection window the closure armed (identity or modes): BUSY stays
/// up until the engine reports it done, so the ~230 ms SPI draws cannot
/// stall the SOP' exchanges (the r160/r176 answer-window rule — the
/// inspection is one long answer window).
static CABLE_WINDOW: AtomicBool = AtomicBool::new(false);
/// The cable page's section on view (0 = not browsing, else section+1):
/// the closure requests the MODES read of the SVID the user scrolled to
/// (kws-x1's control_cable_selection driven by detail_page()).
pub static CABLE_SECTION: AtomicU8 = AtomicU8::new(0);
/// The mode cursor (kws-x1's Selection::mode_cursor), as index+1; 0 none.
static CABLE_MODE_CURSOR: AtomicU8 = AtomicU8::new(0);
/// The published cable page data (written at every publish — the
/// inspection's breadcrumbs each fire one — read by draw_cable). A
/// blocking-mutex RefCell (embassy 0.8's `lock` hands out `&T`, the
/// RefCell carries the mutation — the CABLE_SVIDS shape); the critical
/// section only covers the ~150-byte copy.
pub static CABLE: embassy_sync::blocking_mutex::Mutex<
    CriticalSectionRawMutex,
    RefCell<CableData>,
> = embassy_sync::blocking_mutex::Mutex::new(RefCell::new(CableData::EMPTY));

/// The CABLE screen's ask (its browse-mode OK; kws-x1's long-OK).
pub fn ask_cable() {
    CABLE_ASKED.store(true, Ordering::Relaxed);
    CABLE_ASK.store(true, Ordering::Relaxed);
}

/// The mode cursor's SVID — the page's selected alt mode (kws-x1's draw
/// passes `svids.at(mode_cursor())`).
pub fn cable_cursor_svid() -> Option<u16> {
    let cursor = CABLE_MODE_CURSOR.load(Ordering::Relaxed);
    (cursor != 0)
        .then(|| CABLE_SVIDS.lock(|cell| cell.borrow().at(u16::from(cursor - 1))))
        .flatten()
}

/// What the CABLE static publishes: a Copy snapshot of the observation's
/// SOP' state, refreshed at every publish (the inspection's breadcrumbs
/// each fire one). Lives here — not in cable_page — so the pdmin bin,
/// which includes this file without the UI modules, still builds.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct CableData {
    /// The last inspection's report (`None` while none has landed, or a
    /// new one is running).
    pub report: Option<pd_protocol::pd_cable::CableReport>,
    /// The SVID/modes/extended discovery state beside the report.
    pub modes: pd_protocol::pd_cable_modes::Discovery,
    /// An inspection is running right now.
    pub active: bool,
    /// The observation carries an SPR section at all (kws-x1's
    /// `let Some(spr) = observation.spr` — without one the page says
    /// NO DISCOVERY YET whatever `asked` holds).
    pub spr: bool,
    /// OK asked for a cable read since boot (so a refusal can say why).
    pub asked: bool,
    /// The PD session ended in failure (kws-x1's `Phase::Failed` check).
    pub failed: bool,
    /// The peer's PD revision (3 = PD 3.0), as the observation records it.
    pub peer_revision: Option<u8>,
    /// The request the engine would send now (the safe-fixed-5V gate).
    pub supply: pd_spr::Supply,
    pub target_mv: u16,
}

impl CableData {
    /// A state with no session data at all.
    pub const EMPTY: Self = Self {
        report: None,
        modes: pd_protocol::pd_cable_modes::Discovery::EMPTY,
        active: false,
        spr: false,
        asked: false,
        failed: false,
        peer_revision: None,
        supply: pd_spr::Supply::Fixed,
        target_mv: 0,
    };
    /// No live SPR observation (`spr` was `None` in the publish): the
    /// page's NO DISCOVERY state, whatever an old report said.
    pub fn no_session() -> Self {
        Self {
            peer_revision: None,
            ..Self::EMPTY
        }
    }
}
/// r160: true for the whole diagnostic+session+restore span. The main
/// loop freezes ALL screen draws, meter/accel reads and other SPI/I2C
/// side-work while this is set: Spi::write_busy is a SYNCHRONOUS loop
/// that blocks the cooperative executor for the whole transfer (a
/// draw_pd fill ~230 ms at 4 MHz SPI), and a mid-session blackout
/// starves the engine's transmit chain past the charger's ~77 ms
/// caps->Hard-Reset window (r159: f=6, the HR caught at the pre-TX
/// poll, Request built but never written - q=1 v=5000).
pub static SESSION_ACTIVE: AtomicBool = AtomicBool::new(false);
/// r172: true only while the session is actively NEGOTIATING (diagnostic
/// + caps wait + request chain). False once the contract lands (PS_RDY)
/// or the session ends. The main loop gates the ~230 ms SPI draws and the
/// meter/accel I2C on THIS, not on SESSION_ACTIVE: a persistent contract
/// keeps SESSION_ACTIVE true forever, which starved the PPS screen of
/// every redraw (the r171 'PPS screen was empty') and froze the meter.
pub static BUSY: AtomicBool = AtomicBool::new(false);
const BACK_BIT: u8 = 0x01;

/// What the UI renders on the PD screen. Atomics: the task writes, the main
/// loop reads, no locks.
pub struct View {
    /// `phase_code` value (Waiting 0, Failed 2, Negotiating 10,
    /// Contracted 11, Hold 12, ...).
    pub phase: AtomicU8,
    pub selected_mv: AtomicU16,
    pub selected_ma: AtomicU16,
    pub accept: AtomicBool,
    pub psrdy: AtomicBool,
    pub contract: AtomicBool,
    /// PD spec revision of the peer, 0 = unknown.
    pub peer_rev: AtomicU8,
    pub rx_packets: AtomicU16,
    /// `pd_phy::Error` code, 0 = none.
    pub error: AtomicU16,
    pub vbus_mv: AtomicU16,
    /// CC diagnostic: BC_LVL of both lines at the last attempt.
    pub cc1: AtomicU8,
    pub cc2: AtomicU8,
    /// The diagnostic's listen verdict (0 none, 1 traffic seen).
    pub listen: AtomicU8,
    /// Furthest engine/PHY step (trace_steps).
    pub boot_step: AtomicU8,
    /// The session wrote TX bytes (the engine sent a Message).
    pub tx_written: AtomicBool,
    /// The engine's rx_error code (service-layer receive fault).
    pub rx_error: AtomicU8,
}

pub static VIEW: View = View {
    phase: AtomicU8::new(0),
    selected_mv: AtomicU16::new(0),
    selected_ma: AtomicU16::new(0),
    accept: AtomicBool::new(false),
    psrdy: AtomicBool::new(false),
    contract: AtomicBool::new(false),
    peer_rev: AtomicU8::new(0),
    rx_packets: AtomicU16::new(0),
    error: AtomicU16::new(0),
    vbus_mv: AtomicU16::new(0),
    cc1: AtomicU8::new(0),
    cc2: AtomicU8::new(0),
    listen: AtomicU8::new(0),
    boot_step: AtomicU8::new(0),
    tx_written: AtomicBool::new(false),
    rx_error: AtomicU8::new(0),
};

fn phase_code(phase: Phase) -> u8 {
    match phase {
        Phase::Waiting => 0,
        Phase::Passive => 1,
        Phase::Failed => 2,
        Phase::Skipped => 3,
        Phase::Measuring => 4,
        Phase::Guarded => 5,
        Phase::Verify => 6,
        Phase::Receiving => 7,
        Phase::Captured => 8,
        Phase::NoCapabilities => 9,
        Phase::Negotiating => 10,
        Phase::Contracted => 11,
        Phase::Hold => 12,
    }
}

/// r179: the contract voltage of the last logged PS_RDY (0 = none yet).
/// The line fires on every CHANGE — the boot 5 V and each chosen voltage —
/// instead of once per attempt (the r178 log: the 9 V re-contract was
/// console-silent because the boot 5 V had consumed the once-flag).
static LAST_PSRDY_MV: AtomicU16 = AtomicU16::new(0);

/// r234/r236: store the advertised caps for the PD PROFILES and PPS
/// screens — fixed profiles (object.fixed) and SPR PPS APDOs (decoded
/// from the raw word with kws-x1's describe_pdo bit layout: kind 0b1100
/// in bits 31..28, min bits 8..15, max bits 17..24 in 100 mV units,
/// current bits 0..6 in 50 mA units). Called LIVE from publish() — the
/// post-session site is unreachable while a persistent contract lives
/// (the r236 'q' dump proved count=0 at ph=11).
fn store_caps(caps: &pd_protocol::pd_rx::Advertisements) {
    let mut fixed_n = 0usize;
    let mut pps_n = 0usize;
    for (i, object) in caps
        .objects
        .iter()
        .take(usize::from(caps.count))
        .enumerate()
    {
        if let Some((mv, ma)) = object.fixed {
            if fixed_n < CAPS_MV.len() {
                CAPS_MV[fixed_n].store(mv, Ordering::Relaxed);
                CAPS_MA[fixed_n].store(ma, Ordering::Relaxed);
                fixed_n += 1;
            }
        } else if object.raw >> 28 == 0b1100 && pps_n < PPS_OBJ.len() {
            // The SPR PPS APDO.
            PPS_OBJ[pps_n].store(i as u8 + 1, Ordering::Relaxed);
            PPS_MIN_MV[pps_n].store(((object.raw >> 8) & 0xff) as u16 * 100, Ordering::Relaxed);
            PPS_MAX_MV[pps_n].store(((object.raw >> 17) & 0xff) as u16 * 100, Ordering::Relaxed);
            PPS_MAX_MA[pps_n].store((object.raw & 0x7f) as u16 * 50, Ordering::Relaxed);
            pps_n += 1;
        }
    }
    CAPS_N.store(fixed_n, Ordering::Relaxed);
    PPS_N.store(pps_n, Ordering::Relaxed);
    // The evidence dump — count + every raw slot.
    CAPS_COUNT.store(caps.count, Ordering::Relaxed);
    for (slot, object) in CAPS_RAW.iter().zip(caps.objects.iter()) {
        slot.store(object.raw, Ordering::Relaxed);
    }
}

fn publish(observation: Observation) {
    // r236: the caps store runs LIVE — the moment caps are observed,
    // reachable for persistent contracts too.
    if let Some(caps) = observation.capabilities {
        store_caps(&caps);
    }
    let spr = observation.spr;
    VIEW.phase.store(phase_code(observation.phase), Ordering::Relaxed);
    VIEW.rx_packets
        .store(observation.rx_packets, Ordering::Relaxed);
    if let Some(spr) = spr {
        VIEW.selected_mv
            .store(spr.selected_mv.unwrap_or(0), Ordering::Relaxed);
        VIEW.selected_ma
            .store(spr.selected_ma.unwrap_or(0), Ordering::Relaxed);
        VIEW.accept.store(spr.accept_seen, Ordering::Relaxed);
        VIEW.psrdy.store(spr.psrdy_seen, Ordering::Relaxed);
        VIEW.contract.store(spr.contract_seen, Ordering::Relaxed);
        VIEW.peer_rev
            .store(spr.peer_revision.unwrap_or(0), Ordering::Relaxed);
        VIEW.vbus_mv
            .store(spr.vbus_live_mv.unwrap_or(0), Ordering::Relaxed);
        if spr.psrdy_seen {
            // r178 log: the re-contract's PS_RDY left BUSY stuck true (it
            // lived inside the once-per-attempt gate) and froze the PD
            // screen + meter. The exchange window closes on EVERY publish
            // with the contract up, and the phase follows.
            BUSY.store(false, Ordering::Relaxed);
            VIEW.phase
                .store(phase_code(Phase::Contracted), Ordering::Relaxed);
            let mv = spr.selected_mv.unwrap_or(0);
            if LAST_PSRDY_MV.swap(mv, Ordering::Relaxed) != mv {
                let mut d = [0u8; 40];
                let mut n = 0;
                for &b in b"[pd] psrdy v=" {
                    d[n] = b;
                    n += 1;
                }
                n = crate::push_dec(&mut d, n, u32::from(mv));
                for &b in b"mV ma=" {
                    d[n] = b;
                    n += 1;
                }
                n = crate::push_dec(&mut d, n, u32::from(spr.selected_ma.unwrap_or(0)));
                d[n] = b'\r';
                d[n + 1] = b'\n';
                crate::pd_log(&d[..n + 2]);
            }
        }
    }
    VIEW.boot_step.store(observation.boot_step, Ordering::Relaxed);
    VIEW.tx_written
        .store(observation.spr.is_some_and(|spr| spr.tx_written), Ordering::Relaxed);
    VIEW.rx_error.store(observation.rx_error, Ordering::Relaxed);
    // r238: the cable page's data — a live copy of the SOP' state so the
    // screen follows the inspection's breadcrumb publishes (each mark
    // fires one; the report lands at mark 28, the modes fill between
    // marks 11..25).
    CABLE.lock(|cell| {
        *cell.borrow_mut() = match observation.spr {
            Some(spr) => CableData {
                report: spr.cable_report,
                modes: spr.cable_modes,
                active: spr.cable_active,
                spr: true,
                asked: CABLE_ASKED.load(Ordering::Relaxed),
                failed: observation.phase == Phase::Failed,
                peer_revision: spr.peer_revision,
                supply: spr.supply,
                target_mv: spr.target_mv,
            },
            // No SPR observation at all: the kws-x1 page's NO DISCOVERY
            // state, whatever an old report said.
            None => CableData::no_session(),
        };
    });
}

fn stop_requested() -> bool {
    BTN.load(Ordering::Relaxed) & BACK_BIT != 0 || restart_requested()
}

/// Write the power-on configuration back: a session that dies mid-startup
/// (Hard Reset received, bus error) leaves SWITCHES0/1 and POWER in session
/// state, and every later attempt would fail the crate's Baseline gate
/// (the r113 first-run err=0x102 = register 0x02). DEVICE_ID is read-only
/// and skipped. r145: also DRAIN the read-clear INTERRUPT/A/B latches
/// (0x3e/0x3f/0x42) — a session that ends without its own final poll
/// leaves flags latched there, and the next Session::begin boundary gate
/// refuses on them (the r144 'start err=99' = TransmissionFault). Reading
/// clears; a real fault re-latches and the next begin still catches it.
/// r167: also the pdmin probe's per-attempt bring-up step.
pub(crate) async fn restore_baseline(bus: &mut embassy_at32::i2c::I2c) -> Result<(), ()> {
    use fusb302_pd::phy::Platform as _;
    let baseline = <crate::pd_phy::FnbPlatform as fusb302_pd::phy::Platform>::BASELINE;
    // REGISTERS is 13 wide (the two dynamic STATUS bytes trail); the
    // BASELINE covers the first 11, and DEVICE_ID (read-only) is skipped.
    use embedded_hal_async::i2c::I2c as _;
    for (reg, value) in fusb302_pd::snapshot::REGISTERS
        .iter()
        .skip(1)
        .take(10)
        .zip(baseline.iter().skip(1))
    {
        bus.write(0x22, &[*reg, *value]).await.map_err(|_| ())?;
    }
    // Drain the latched interrupt flags (read-clear registers).
    for reg in fusb302_pd::register::INTERRUPTS {
        let mut flag = [0u8];
        bus.write_read(fusb302_pd::snapshot::ADDRESS, &[reg], &mut flag)
            .await
            .map_err(|_| ())?;
    }
    Ok(())
}


/// The session/PHY error mapped to a diagnostic code for the PD screen.
/// r167: shared with the pdmin probe's log lines.
/// r239f probe: the I2C error kind, as a small code for the log line
/// (1 Timeout, 2 ArbitrationLoss, 3 Bus, 4 Overrun, 5 ClockTimeout,
/// 6.. NAK by source: 6 Generic, 7 Address, 8 Data, 9 Acknowledge).
/// r250: the standard I2C bus unwedge — a slave stuck mid-
/// transaction (the FUSB302's I2C interface locks up after an
/// unanswered Hard-Reset transmission, the boot dead-line wake's
/// case) holds SDA low and dead-ends EVERY device on I2C1, the
/// meter at 0x40 included. The pins are re-driven as GPIO open-
/// drain; if SDA is held low, up to 9 SCL pulses release it, then
/// a STOP condition. The caller re-muxes the pins to their
/// alternate function afterwards (and holds `BUS` at runtime, so
/// no transaction can interleave). Harmless on a healthy bus: the
/// pulses only fire when SDA is actually held.
/// Moved here from main.rs (r250): the PD task's park arms run it
/// themselves — the FUSB302-only wedge never trips the meter arm's
/// read transition (the TPA keeps answering), so on charger-only
/// power the unwedge never happened and the bootlooped.
pub(crate) unsafe fn i2c_unwedge() {
    use embassy_at32::gpio::{Drive, OutputPin, Port};
    const HALF: u32 = 48; // ~5 us at 192 MHz
    // r254: a trace row either side of the dance — the lines as found
    // and as left (read before the pins become GPIOs; IDT shows the line
    // in either mode). Trace builds only.
    #[cfg(feature = "i2c-trace")]
    let found = embassy_at32::i2c::trace::pins();
    let mut scl =
        unsafe { OutputPin::new_open_drain(Port::B, 6, true, Drive::Moderate) }
            .expect("i2c scl gpio");
    let mut sda =
        unsafe { OutputPin::new_open_drain(Port::B, 7, true, Drive::Moderate) }
            .expect("i2c sda gpio");
    cortex_m::asm::delay(HALF);
    if !sda.is_set() {
        for _ in 0..9 {
            scl.set_low();
            cortex_m::asm::delay(HALF);
            scl.set_high();
            cortex_m::asm::delay(HALF);
            if sda.is_set() {
                break;
            }
        }
    }
    // A STOP condition: SDA low while SCL high, then SDA high.
    sda.set_low();
    cortex_m::asm::delay(HALF);
    sda.set_high();
    cortex_m::asm::delay(HALF);
    #[cfg(feature = "i2c-trace")]
    embassy_at32::i2c::trace::mark(1, found, embassy_at32::i2c::trace::pins());
}

pub(crate) fn i2c_src_code(e: embassy_at32::i2c::Error) -> u32 {
    use embassy_at32::i2c::Error as I;
    match e {
        I::Timeout => 1,
        I::ArbitrationLoss => 2,
        I::Bus => 3,
        I::Overrun => 4,
        I::ClockTimeout => 5,
        I::NoAcknowledge(source) => match source {
            embassy_at32::i2c::NoAcknowledgeSource::Address => 7,
            embassy_at32::i2c::NoAcknowledgeSource::Data => 8,
        },
    }
}

pub(crate) fn error_code(error: &pd_phy::Error<embassy_at32::i2c::Error>) -> u16 {
    use pd_phy::Error as E;
    match error {
        E::Orientation => 2,
        E::InputVoltage { .. } => 3,
        E::Config(_) => 4,
        E::Bus { .. } => 5,
        E::Boundary { .. } => 6,
        E::Rx(fusb302_pd::service::Error::HardReset) => 7,
        E::Rx(fusb302_pd::service::Error::PhyFault) => 8,
        E::Rx(fusb302_pd::service::Error::RxFull) => 9,
        E::Rx(fusb302_pd::service::Error::Frame(_)) => 10,
        E::Stopped => 11,
        E::StopRequested => 12,
        E::UnexpectedAck => 13,
        E::Request => 14,
        E::QueueFull => 15,
        E::Budget => 16,
        E::TxTimeout => 17,
        E::Completion => 18,
        E::PpsDeadline => 19,
        E::HardResetNeeded => 20,
        E::NotAcknowledged => 21,
        E::Peer => 22,
        // r144: name these instead of the anonymous 99 - HardResetVbus is
        // the expected restart outcome with the micro-B PC attached (its
        // 5 V holds the shared VBUS rail through the charger's dip).
        E::HardResetVbus => 23,
        E::BistCarrierVbus => 24,
        E::Deferred => 25,
        _ => 99,
    }
}

/// The ENGINE's error mapped to kws-x1's rx_error numbering (32..=44) —
/// r114's log ended sessions silently because these were dropped.
fn engine_error_code(failure: &pd_spr::Error) -> u16 {
    match failure {
        pd_spr::Error::Stopped => 32,
        pd_spr::Error::UnsupportedPhy => 33,
        pd_spr::Error::UnsafeCapabilities => 34,
        pd_spr::Error::Peer => 35,
        pd_spr::Error::ReceiveDiscarded => 36,
        pd_spr::Error::HardResetReceived => 37,
        pd_spr::Error::HardResetSent => 44,
        pd_spr::Error::Transmit(_) => 38,
        pd_spr::Error::Declined(_) => 39,
        pd_spr::Error::RecoveryBlocked => 40,
        pd_spr::Error::TransmissionBlocked => 41,
        pd_spr::Error::Engine(_) => 42,
    }
}

struct PdClock;
impl fusb302_pd::phy::Clock for PdClock {
    fn now_ms(&self) -> u64 {
        embassy_time::Instant::now().as_millis()
    }
}
impl pd_spr::Timer for PdClock {
    async fn after_millis(ms: u64) {
        embassy_time::Timer::after_millis(ms).await;
    }
    async fn after_timer(timer: pd_spr::TimerType, ms: u64) {
        embassy_time::Timer::after_millis(ms).await;
        // These Ready selects and answer waits must not cancel an active
        // I2C receive future mid-transfer.
        if matches!(
            timer,
            pd_spr::TimerType::SinkPPSPeriodic
                | pd_spr::TimerType::SinkEPRKeepAlive
                | pd_spr::TimerType::SinkEPREnter
                | pd_spr::TimerType::SenderResponse
                | pd_spr::TimerType::VDMResponse
        ) {
            wait_for_rx_idle(&mut embassy_time::Delay).await;
        }
    }
}

/// Restart requested by an OK press (the session stops, the charger is
/// restarted with a Hard Reset, and the firmware reboots — kws-x1's exact
/// restart sequence).
static RESTART: AtomicBool = AtomicBool::new(false);

pub fn request_restart() {
    RESTART.store(true, Ordering::Relaxed);
    START.signal(());
}

fn restart_requested() -> bool {
    RESTART.load(Ordering::Relaxed)
}

// r233: the passthrough publish — the SNIFF screen's head line ("SNIFF
// CC1 RP3.0A M:9") and the message rows (pd_protocol's describe() renders
// each entry to 24 chars), plus the message count the screen's redraw
// gate watches. All behind embassy mutexes (the pd_task writes, the UI
// reads).
pub static SNIFF_HEAD: Mutex<CriticalSectionRawMutex, [u8; 24]> = Mutex::new([0u8; 24]);
pub static SNIFF_ROWS: Mutex<CriticalSectionRawMutex, [[u8; 24]; 7]> =
    Mutex::new([[0u8; 24]; 7]);
pub static SNIFF_MESSAGES: AtomicU16 = AtomicU16::new(0);

/// The Rp label bytes (kws-x1's pd.rs head).
fn rp_label(rp: Option<pd_protocol::pd_sniff::Rp>) -> &'static [u8] {
    match rp {
        Some(pd_protocol::pd_sniff::Rp::Default) => b" RPDEF ",
        Some(pd_protocol::pd_sniff::Rp::Current1A5) => b" RP1.5A",
        Some(pd_protocol::pd_sniff::Rp::Current3A0) => b" RP3.0A",
        None => b" RP--  ",
    }
}

/// Render + publish the sniff state (the head and the message rows,
/// oldest first).
async fn publish_sniff(
    log: &pd_protocol::pd_sniff::Log,
    cc: Option<u8>,
    rp: Option<pd_protocol::pd_sniff::Rp>,
) {
    let mut head = [0u8; 24];
    let mut n = 0;
    for &b in b"SNIFF CC" {
        head[n] = b;
        n += 1;
    }
    match cc {
        Some(c) => {
            head[n] = b'0' + c;
            n += 1;
        }
        None => {
            head[n] = b'-';
            n += 1;
        }
    }
    for &b in rp_label(rp) {
        head[n] = b;
        n += 1;
    }
    for &b in b" M:" {
        head[n] = b;
        n += 1;
    }
    n = crate::push_dec(&mut head, n, log.messages.min(9999));
    head[n] = 0;
    let mut rows = [[0u8; 24]; 7];
    let len = log.entries().len();
    let skip = len.saturating_sub(7);
    for (i, entry) in log.entries().enumerate() {
        if i < skip {
            continue;
        }
        rows[i - skip] = pd_protocol::pd_sniff::describe(entry);
    }
    *SNIFF_HEAD.lock().await = head;
    *SNIFF_ROWS.lock().await = rows;
    SNIFF_MESSAGES.store(log.messages.min(u16::MAX as u32) as u16, Ordering::Relaxed);
}

/// The PPS heuristic (kws-x1's uncaptured_request): a Request seen before
/// any Source_Capabilities is a re-request (the meter booted from the same
/// VBUS after the negotiation); a second one for the same object is PPS
/// (a fixed sink only answers new capabilities, which the sniffer would
/// have captured).
fn uncaptured_request(uncaptured: &mut Option<usize>, rdo: u32) -> pd_protocol::pd_sniff::Supply {
    use pd_protocol::pd_sniff::Supply;
    const RDO_POSITION_SHIFT: u32 = 28;
    const PPS_RDO_VOLTAGE_SHIFT: u32 = 9;
    const PPS_RDO_VOLTAGE_MASK: u32 = 0xfff;
    const PPS_RDO_VOLTAGE_UNIT_MV: u32 = 20;
    const PPS_RDO_CURRENT_MASK: u32 = 0x7f;
    const PPS_RDO_CURRENT_UNIT_MA: u32 = 50;
    let position = (rdo >> RDO_POSITION_SHIFT) as usize;
    let repeated = *uncaptured == Some(position);
    *uncaptured = Some(position);
    if repeated {
        Supply::Pps {
            mv: ((rdo >> PPS_RDO_VOLTAGE_SHIFT) & PPS_RDO_VOLTAGE_MASK)
                * PPS_RDO_VOLTAGE_UNIT_MV,
            ma: (rdo & PPS_RDO_CURRENT_MASK) * PPS_RDO_CURRENT_UNIT_MA,
        }
    } else {
        Supply::Raw
    }
}

/// r233/r238 FIX: the passthrough task entry — main spawns THIS instead
/// of run() when the boot-chosen setting is ON (the receive-only sniffer
/// replaces the sink session). The dispatch was a casualty of the r233
/// rebuild — sniff() existed but nothing called it, so a pass=ON boot
/// still ran the sink session and the SNIFF screen waited forever;
/// caught while wiring the r238 cable page (whose passthrough notice
/// assumes the sink is off in that mode).
#[embassy_executor::task]
pub async fn sniff_task() {
    sniff().await;
}

/// r233: the passthrough sniff loop (kws-x1's sniff()): the CC pull-downs
/// come off in the sniffer's listening configuration within milliseconds
/// of boot (before the charger's PD negotiation), the RX stays on and
/// auto-CRC stays off — the meter never sends, never acknowledges, never
/// resets. Every heard frame lands in the Log for the SNIFF screen. Any
/// error stops it for good (parked; the screen keeps the last state).
async fn sniff() {
    const SNIFF_FULL_POLL: embassy_time::Duration = embassy_time::Duration::from_millis(10);
    crate::pd_log(b"[pd] passthrough: listen only\r\n");
    let mut log = pd_protocol::pd_sniff::Log::new();
    let mut caps = [0u32; 7];
    let mut caps_n = 0usize;
    let mut uncaptured: Option<usize> = None;
    let mut rp: Option<pd_protocol::pd_sniff::Rp> = None;
    let mut cc: Option<u8> = None;
    loop {
        STAGE.store(2, Ordering::Relaxed);
        crate::dbg_stage(11);
        SESSION_ACTIVE.store(false, Ordering::Relaxed);
        BUSY.store(false, Ordering::Relaxed);
        // The released snapshot: the power-on configuration restored and
        // verified (the sniffer's start removes the CC pull-downs).
        let started: Option<fusb302_pd::sniffer::ActiveSniffer> = {
            let mut guard = BUS.lock().await;
            let started_inner = match guard.0.as_mut() {
                Some(bus) => {
                    if restore_baseline(bus).await.is_err() {
                        crate::pd_log(b"[pd] baseline FAIL\r\n");
                        None
                    } else {
                        match fusb302_pd::snapshot::snapshot(bus).await {
                            Err(failure) => {
                                let mut d = [0u8; 24];
                                let mut n = 0;
                                for &b in b"[pd] snapshot FAIL reg=" {
                                    d[n] = b;
                                    n += 1;
                                }
                                n = crate::push_hex8(&mut d, n, u32::from(failure.register));
                                d[n] = b'\r';
                                d[n + 1] = b'\n';
                                crate::pd_log(&d[..n + 2]);
                                None
                            }
                            Ok(snapshot) => {
                                let mut delay = embassy_time::Delay;
                                match fusb302_pd::sniffer::Sniffer::once(
                                    <crate::pd_phy::FnbPlatform
                                        as fusb302_pd::phy::Platform>::BASELINE,
                                    MEASURE,
                                )
                                .start(bus, &mut delay, snapshot)
                                .await
                                {
                                    Ok(sniffer) => {
                                        cc = Some(sniffer.selected());
                                        Some(sniffer)
                                    }
                                    Err(_) => {
                                        crate::pd_log(b"[pd] sniff start FAIL\r\n");
                                        None
                                    }
                                }
                            }
                        }
                    }
                }
                None => {
                    crate::pd_log(b"[pd] no bus\r\n");
                    None
                }
            };
            started_inner
        };
        let Some(mut sniffer) = started else {
            // The sniffer is once-per-boot (kws-x1's semantics): park.
            crate::pd_log(b"[pd] sniff parked\r\n");
            loop {
                Timer::after_secs(3600).await;
            }
        };
        publish_sniff(&log, cc, rp).await;
        let mut next_check =
            embassy_time::Instant::now() + embassy_time::Duration::from_secs(1);
        let mut next_poll = embassy_time::Instant::now();
        loop {
            Timer::after_millis(1).await;
            let mut guard = BUS.lock().await;
            let Some(bus) = guard.0.as_mut() else {
                continue;
            };
            let now = embassy_time::Instant::now();
            // kws-x1's poll cadence: the full FIFO poll every 10 ms,
            // a STATUS1 pending check between, the config check 1/s.
            let result = if now >= next_check {
                next_check += embassy_time::Duration::from_secs(1);
                sniffer.check(bus).await.map(|()| None)
            } else {
                let due = if now >= next_poll {
                    Ok(true)
                } else {
                    sniffer.pending(bus).await
                };
                match due {
                    Ok(true) => {
                        next_poll = now + SNIFF_FULL_POLL;
                        sniffer.poll(bus).await.map(Some)
                    }
                    Ok(false) => Ok(None),
                    Err(error) => Err(error),
                }
            };
            let mut changed = false;
            match result {
                Ok(Some((outcome, status0))) => {
                    // BC_LVL is only meaningful between BMC transmissions.
                    if fusb302_pd::pd_sniff::quiet(status0)
                        && let Some(new_rp) = fusb302_pd::pd_sniff::rp(status0)
                        && Some(new_rp) != rp
                    {
                        rp = Some(new_rp);
                        changed = true;
                    }
                    if let fusb302_pd::service::Outcome::Packet(packet) = outcome {
                        let frame = packet.frame();
                        // Source_Capabilities replace the stored PDOs
                        // first, so a later Request decodes against the
                        // newest offer (kws-x1's record()).
                        if frame.header() & fusb302_pd::header::EXTENDED == 0
                            && let Ok(objects) = frame.source_capabilities()
                        {
                            caps_n = objects.len().min(7);
                            for (slot, raw) in caps[..caps_n].iter_mut().zip(objects) {
                                *slot = raw;
                            }
                        }
                        let entry = pd_protocol::pd_sniff::Entry::classify(
                            frame.header(),
                            frame.data(),
                            &caps[..caps_n],
                        );
                        if let Some(supply) = entry.supply {
                            let supply = match supply {
                                pd_protocol::pd_sniff::Supply::Raw if caps_n == 0 => {
                                    uncaptured_request(&mut uncaptured, entry.word)
                                }
                                supply => supply,
                            };
                            let _ = supply;
                        }
                        log.push(entry);
                        changed = true;
                    }
                }
                Ok(None) => {}
                Err(_) => {
                    crate::pd_log(b"[pd] sniff stop\r\n");
                    // Parked for good: any error ends the sniffer without
                    // cleanup writes (kws-x1's semantics).
                    loop {
                        Timer::after_secs(3600).await;
                    }
                }
            }
            if changed {
                publish_sniff(&log, cc, rp).await;
            }
        }
    }
}

#[embassy_executor::task]
pub async fn run() {
    // r171: the boot session starts IMMEDIATELY and unconditionally, and
    // it now COMPLETES a contract: with the r170 I2C fix the Request is on
    // the wire ~10 ms after the caps' GoodCRC, inside every charger's
    // request window. The r138 3 s USB gate and the r142 20 s charger-only
    // deferral existed only because the Request never left (acked caps ->
    // Hard Reset -> VBUS dip); their reason is gone. The boot contract
    // targets DESIRED_MV (default 5 V); OK on the PPS screen re-requests
    // the chosen voltage. The boot LOG waits for nothing: the 4096-byte
    // ring (console.rs BootLog) flushes when the host connects.
    STAGE.store(1, Ordering::Relaxed);
    crate::pd_log(b"[pd] boot session\r\n");
    let mut first = true;
    // r236: the boot dead-line wake fires once — a soft reset (the
    // bootloader's RESET, a PC replug) never drops the CC pull-downs, so
    // an attached charger still sees the same sink and never re-announces
    // its caps; the engine dies having heard nothing. One automatic
    // OK-press equivalent (the r147 restart flow) wakes it.
    //
    // r250: the wake's role re-established. The caps announce at attach
    // is NEVER heard by any build — the FUSB302's receiver is configured
    // only ~600 ms into boot, and the source's announce cycle finishes
    // before that (the diag's listen reports li=0 in every build,
    // r239g included). The engine's own Get_Source_Cap recovery does not
    // fire either, and once the source has exhausted nCapsCount without
    // an ACK it disables its PD — no caps ever again without a trigger.
    // The wake's HR is that trigger: it makes the source re-announce
    // AFTER the receiver is ready, which is how r239g contracted.
    //
    // The wake's cost: per the r239f finding the FUSB302's I2C interface
    // locks up after an unanswered HR transmission, and on charger-only
    // power the charger then cuts VBUS with no console back-feed to hold
    // the rail — the r249 standalone bootloop. The mitigation is the
    // r250b dance in the failure arms below: every HR is followed by the
    // bus unwedge, and the retry reruns the session on the healed bus
    // with the wake re-armed for one more attempt.
    let mut boot_wake = true;
    // r239: the boot diagnostic's failure count — a wedged FUSB302
    // (I2C NAKs after an MCU reset hit it mid-transaction) used to
    // retry SILENTLY forever behind BUSY=true, freezing the UI. Three
    // failures park the task loudly instead; an OK press retries.
    let mut diag_fails = 0u32;
    // r239g: the first diagnostic-failure park retries ONCE after the
    // main task's bus unwedge (the wake-HR's wedge heals ~1.5 s after
    // BUSY clears; the meter transition fires the pin dance). The
    // retry restores the r236-style automatic negotiation through the
    // VBUS race (the charger cycles VBUS vs the console's back-feed).
    let mut retried = false;
    loop {
        if !first {
            STAGE.store(6, Ordering::Relaxed); // parked, waiting for OK
            START.wait().await;
        }
        first = false;
        // r145: RESTART is consumed below once the restart has been
        // performed; every OK press is a restart (fresh session on the
        // fresh attach our own Hard Reset creates).
        STAGE.store(2, Ordering::Relaxed); // CC diagnostic
        crate::dbg_stage(11);
        SESSION_ACTIVE.store(true, Ordering::Relaxed);
        BUSY.store(true, Ordering::Relaxed);
        // r145: fresh view per attempt — a stale err=45/40/st= from the
        // previous attempt mislabeled the next one on the LCD.
        VIEW.error.store(0, Ordering::Relaxed);
        VIEW.phase.store(phase_code(Phase::Waiting), Ordering::Relaxed);
        VIEW.boot_step.store(0, Ordering::Relaxed);
        VIEW.tx_written.store(false, Ordering::Relaxed);
        VIEW.rx_packets.store(0, Ordering::Relaxed);
        VIEW.rx_error.store(0, Ordering::Relaxed);
        VIEW.listen.store(0, Ordering::Relaxed);

        // One-shot CC diagnostic on the power-on configuration: measures
        // both CCs with Rd on both and listens for the source's traffic;
        // its report feeds the session start.
        let report: Option<Report> = {
            let mut guard = BUS.lock().await;
            let Some(bus) = guard.0.as_mut() else {
                // r145: loud, not silent (a skipped boot session used to
                // leave no trace at all).
                VIEW.error.store(26, Ordering::Relaxed);
                crate::pd_log(b"[pd] no bus\r\n");
                diag_fails = diag_fails.saturating_add(1);
                if diag_fails >= 3 {
                    diag_fails = 0;
                    SESSION_ACTIVE.store(false, Ordering::Relaxed);
                    BUSY.store(false, Ordering::Relaxed);
                    if !retried {
                        // r239g: the ONE automatic retry - BUSY is down now,
                        // so the main task's meter transition runs the pin
                        // dance within a tick; the delay gives it the window,
                        // then the session reruns on the healed bus (the
                        // wake's HR already woke the charger - its caps
                        // arrive into THIS session).
                        //
                        // r250: the dance now runs HERE, from the PD side —
                        // the FUSB302-only wedge never trips the meter arm's
                        // read transition (the TPA keeps answering), so on
                        // charger-only power the unwedge never happened and
                        // the bootlooped. The dance's SDA gate makes it
                        // harmless when the wedge is the NAK-latch type.
                        retried = true;
                        crate::pd_log(
                            b"[pd] diag FAIL x3 - retry after unwedge\r\n",
                        );
                        {
                            let mut guard = BUS.lock().await;
                            unsafe { i2c_unwedge() };
                        }
                        first = true; // the loop top skips its own wait
                        boot_wake = false; // no second auto-wake
                        Timer::after_millis(1500).await;
                        continue;
                    }
                    first = false;
                    continue;
                }
                VIEW.phase.store(phase_code(Phase::Failed), Ordering::Relaxed);
                SESSION_ACTIVE.store(false, Ordering::Relaxed);
                BUSY.store(false, Ordering::Relaxed);
                continue;
            };
            // Restore the power-on configuration FIRST: a previous session
            // that died mid-startup leaves the registers in session state
            // and the crate's Baseline gate would fail every later attempt.
            if restore_baseline(bus).await.is_err() {
                VIEW.error.store(0x300, Ordering::Relaxed);
                crate::pd_log(b"[pd] baseline FAIL (bus a=22)\r\n");
                diag_fails = diag_fails.saturating_add(1);
                if diag_fails >= 3 {
                    diag_fails = 0;
                    SESSION_ACTIVE.store(false, Ordering::Relaxed);
                    BUSY.store(false, Ordering::Relaxed);
                    if !retried {
                        // r239g: the ONE automatic retry - BUSY is down now,
                        // so the main task's meter transition runs the pin
                        // dance within a tick; the delay gives it the window,
                        // then the session reruns on the healed bus (the
                        // wake's HR already woke the charger - its caps
                        // arrive into THIS session).
                        //
                        // r250: the dance now runs HERE, from the PD side —
                        // the FUSB302-only wedge never trips the meter arm's
                        // read transition (the TPA keeps answering), so on
                        // charger-only power the unwedge never happened and
                        // the bootlooped. The dance's SDA gate makes it
                        // harmless when the wedge is the NAK-latch type.
                        retried = true;
                        crate::pd_log(
                            b"[pd] diag FAIL x3 - retry after unwedge\r\n",
                        );
                        {
                            let mut guard = BUS.lock().await;
                            unsafe { i2c_unwedge() };
                        }
                        first = true; // the loop top skips its own wait
                        boot_wake = false; // no second auto-wake
                        Timer::after_millis(1500).await;
                        continue;
                    }
                    first = false;
                    continue;
                }
                VIEW.phase.store(phase_code(Phase::Failed), Ordering::Relaxed);
                SESSION_ACTIVE.store(false, Ordering::Relaxed);
                BUSY.store(false, Ordering::Relaxed);
                continue;
            }
            let snapshot = match fusb302_pd::snapshot::snapshot(bus).await {
                Ok(snapshot) => snapshot,
                Err(failure) => {
                    VIEW.error.store(u16::from(failure.register) | 0x100, Ordering::Relaxed);
                    crate::pd_log(b"[pd] snapshot FAIL\r\n");
                    diag_fails = diag_fails.saturating_add(1);
                    if diag_fails >= 3 {
                        diag_fails = 0;
                        SESSION_ACTIVE.store(false, Ordering::Relaxed);
                        BUSY.store(false, Ordering::Relaxed);
                        if !retried {
                            // r239g: the ONE automatic retry - BUSY is down now,
                            // so the main task's meter transition runs the pin
                            // dance within a tick; the delay gives it the window,
                            // then the session reruns on the healed bus (the
                            // wake's HR already woke the charger - its caps
                            // arrive into THIS session).
                            //
                            // r250: the dance runs HERE (the BUS is held by
                            // this arm) — the FUSB302-only wedge never trips
                            // the meter arm's read transition (the TPA keeps
                            // answering), so on charger-only power the
                            // unwedge never happened and the bootlooped.
                            retried = true;
                            crate::pd_log(
                                b"[pd] diag FAIL x3 - retry after unwedge\r\n",
                            );
                            unsafe { i2c_unwedge() };
                            first = true; // the loop top skips its own wait
                            boot_wake = false; // no second auto-wake
                            Timer::after_millis(1500).await;
                            continue;
                        }
                        first = false;
                        continue;
                    }
                VIEW.phase.store(phase_code(Phase::Failed), Ordering::Relaxed);
                SESSION_ACTIVE.store(false, Ordering::Relaxed);
                BUSY.store(false, Ordering::Relaxed);
                continue;
                }
            };
            VIEW.phase.store(phase_code(Phase::Measuring), Ordering::Relaxed);
            let mut delay = embassy_time::Delay;
            match fusb302_pd::startup::Diagnostic::once(
                <crate::pd_phy::FnbPlatform as fusb302_pd::phy::Platform>::BASELINE,
                MEASURE,
            )
            .run(bus, &mut delay, snapshot)
            .await
            {
                Ok(report) => {
                    diag_fails = 0;
                    VIEW.cc1.store(report.status[0], Ordering::Relaxed);
                    VIEW.cc2.store(report.status[1], Ordering::Relaxed);
                    VIEW.listen.store(
                        u8::from(
                        report.listen.is_some_and(|l| l.traffic.is_some()),
                    ),
                        Ordering::Relaxed,
                    );
                    Some(report)
                }
                Err(failure) => {
                    // Baseline/verify failures report the offending
                    // register in the low byte.
                    let code: u16 = match failure.error {
                        fusb302_pd::config::Error::Baseline { register, .. }
                        | fusb302_pd::config::Error::Verify { register, .. } => {
                            u16::from(register) | 0x100
                        }
                        fusb302_pd::config::Error::Bus { register, .. } => {
                            u16::from(register) | 0x200
                        }
                    };
                    VIEW.error.store(code, Ordering::Relaxed);
                    // r239: the failure used to retry silently forever
                    // (BUSY latched, the UI frozen) — log, retry 3x,
                    // then park loudly (an OK press starts a fresh
                    // attempt; a power cycle unwedges the chip).
                    diag_fails = diag_fails.saturating_add(1);
                    let mut d = [0u8; 40];
                    let mut n = 0;
                    for &b in b"[pd] diag FAIL n=" {
                        d[n] = b;
                        n += 1;
                    }
                    n = crate::push_dec(&mut d, n, diag_fails);
                    for &b in b" err=" {
                        d[n] = b;
                        n += 1;
                    }
                    n = crate::push_dec(&mut d, n, u32::from(code));
                    for &b in b"\r\n" {
                        d[n] = b;
                        n += 1;
                    }
                    crate::pd_log(&d[..n]);
                    if diag_fails >= 3 {
                        diag_fails = 0;
                        SESSION_ACTIVE.store(false, Ordering::Relaxed);
                        BUSY.store(false, Ordering::Relaxed);
                        if !retried {
                            // r239g: the ONE automatic retry - BUSY is down now,
                            // so the main task's meter transition runs the pin
                            // dance within a tick; the delay gives it the window,
                            // then the session reruns on the healed bus (the
                            // wake's HR already woke the charger - its caps
                            // arrive into THIS session).
                            //
                            // r250: the dance runs HERE (the BUS is held by
                            // this arm) — the FUSB302-only wedge never trips
                            // the meter arm's read transition (the TPA keeps
                            // answering), so on charger-only power the
                            // unwedge never happened and the bootlooped.
                            retried = true;
                            crate::pd_log(
                                b"[pd] diag FAIL x3 - retry after unwedge\r\n",
                            );
                            unsafe { i2c_unwedge() };
                            first = true; // the loop top skips its own wait
                            boot_wake = false; // no second auto-wake
                            Timer::after_millis(1500).await;
                            continue;
                        }
                        first = false;
                        continue;
                    }
                VIEW.phase.store(phase_code(Phase::Failed), Ordering::Relaxed);
                SESSION_ACTIVE.store(false, Ordering::Relaxed);
                BUSY.store(false, Ordering::Relaxed);
                continue;
                }
            }
        };
        let Some(report) = report else { continue };
        {
            // r139: the diagnostic's verdict into the console (PD_LOG).
            const HEX: &[u8; 16] = b"0123456789abcdef";
            let mut d = [0u8; 40];
            let mut n = 0;
            for &b in b"[pd] diag cc=" {
                d[n] = b;
                n += 1;
            }
            for &status in &report.status {
                d[n] = HEX[(status >> 4) as usize];
                d[n + 1] = HEX[(status & 0xF) as usize];
                d[n + 2] = b',';
                n += 3;
            }
            d[n - 1] = b' ';
            for &b in b"li=" {
                d[n] = b;
                n += 1;
            }
            // r145: same meaning as VIEW.listen (traffic seen), not
            // merely "the listen ran".
            d[n] = b'0'
                + u8::from(report.listen.is_some_and(|l| l.traffic.is_some()));
            n += 1;
            d[n] = b'\r';
            d[n + 1] = b'\n';
            crate::pd_log(&d[..n + 2]);
        }

        // --- The sink session on the fresh attach: the engine selects the
        // target and drives Requests; our Platform supplies VBUS and
        // policy. The diagnostic above ran ONCE (kws-x1's flow; before
        // r138 a second run repeated it, delaying the session start by up
        // to ~640 ms of listen windows).
        let selection = pd_spr::Selection::<FnbLimits>::new();
        // r149: ARM the first Request ourselves (the shared engine NEVER
        // auto-requests — kws-x1's product sends PdCommand::Confirm from
        // its selector UI). r173: the arm happens (a) at boot — the first
        // caps arm DESIRED_MV so the boot contract is 5 V — and (b) on an
        // explicit COMMIT (the PD screen's OK): preview+confirm wakes the
        // engine and it sends a plain Request FROM READY — the PD-native
        // voltage change, no Hard Reset, no restart. Browsing never
        // transmits; a commit for the already-established voltage is a
        // no-op; a voltage the charger does not offer is logged, not
        // silently ignored. confirm() refuses after a Reject until fresh
        // caps — also logged.
        let control =
            |observation: &mut Observation, store: Option<&dyn pd_protocol::pd_svids::Store>| {
                use pd_protocol::pd_spr::Supply;
                // r238: the cable inspection window CLOSED — the engine
                // finished (or refused) its SOP' work. BUSY reopens the
                // draws here (the r176 outcome side; publish() cannot own
                // this: between arm and start the trace still carries the
                // PREVIOUS report, and the CC2-refused inspection returns
                // without a closing publish). Exactly one close per armed
                // window, so this is also where the outcome logs.
                if CABLE_WINDOW.load(Ordering::Relaxed)
                    && !selection.cable_active()
                    && !selection.has_pending()
                {
                    CABLE_WINDOW.store(false, Ordering::Relaxed);
                    BUSY.store(false, Ordering::Relaxed);
                    // 0 Identity, 1 Nak, 2 Busy, 3 Timeout, 4 Deferred.
                    let (status, count, rev) = observation
                        .spr
                        .and_then(|spr| spr.cable_report)
                        .map_or((9u32, 0u32, 0u32), |report| {
                            (
                                match report.status {
                                    pd_protocol::pd_cable::CableStatus::Identity => 0,
                                    pd_protocol::pd_cable::CableStatus::Nak => 1,
                                    pd_protocol::pd_cable::CableStatus::Busy => 2,
                                    pd_protocol::pd_cable::CableStatus::Timeout => 3,
                                    pd_protocol::pd_cable::CableStatus::Deferred => 4,
                                },
                                u32::from(report.count),
                                u32::from(report.revision),
                            )
                        });
                    let mut d = [0u8; 40];
                    let mut n = 0;
                    for &b in b"[cable] report s=" {
                        d[n] = b;
                        n += 1;
                    }
                    n = crate::push_dec(&mut d, n, status);
                    for &b in b" n=" {
                        d[n] = b;
                        n += 1;
                    }
                    n = crate::push_dec(&mut d, n, count);
                    for &b in b" rev=" {
                        d[n] = b;
                        n += 1;
                    }
                    n = crate::push_dec(&mut d, n, rev);
                    for &b in b"\r\n" {
                        d[n] = b;
                        n += 1;
                    }
                    crate::pd_log(&d[..n]);
                }
                if selection.has_pending() || selection.request_in_flight() {
                    return; // a latched COMMIT waits for the next idle poll
                }
                // r238: the CABLE screen's ask (kws-x1's long-OK →
                // query_at_5v). The engine only inspects from a PD 3.0
                // fixed-5V Ready (Selection::request_cable's own gate), so
                // a contract on another voltage first re-requests 5 V and
                // the query waits for it; a PD 2.0 peer never drops the
                // contract for a query that cannot follow (kws-x1's
                // `!pd3` drop).
                let pd3 = observation
                    .spr
                    .and_then(|spr| spr.peer_revision)
                    == Some(3);
                if CABLE_ASK.swap(false, Ordering::Relaxed) {
                    if !pd3 {
                        crate::pd_log(b"[cable] ask: needs PD3.0\r\n");
                    } else if selection.request_cable() {
                        CABLE_WINDOW.store(true, Ordering::Relaxed);
                        BUSY.store(true, Ordering::Relaxed);
                        crate::pd_log(b"[cable] ask armed\r\n");
                    } else {
                        // Not at fixed 5 V: request 5 V first, the query
                        // follows once it has held (kws-x1's after_5v).
                        let five = (0..selection.option_count())
                            .filter_map(|index| selection.option(index))
                            .find(|option| {
                                matches!(option.supply, Supply::Fixed) && option.mv == 5_000
                            });
                        if five.is_some_and(|five| selection.preview(five) && selection.confirm())
                        {
                            // The 5 V exchange window (the existing PS_RDY
                            // publish reopens BUSY when it lands; the
                            // resume below re-arms it for the read).
                            BUSY.store(true, Ordering::Relaxed);
                            CABLE_AFTER_5V.store(true, Ordering::Relaxed);
                            crate::pd_log(b"[cable] ask: 5V first\r\n");
                        } else {
                            crate::pd_log(b"[cable] ask: no 5V contract\r\n");
                        }
                    }
                }
                // The waiting query fires once its 5 V contract has held
                // for kws-x1's FIVE_VOLT_SETTLE_MS (their resume_5v_query;
                // selection.now_ms() is the engine's own clock).
                if CABLE_AFTER_5V.load(Ordering::Relaxed) {
                    let at_5v = selection
                        .established_target()
                        .is_some_and(|target| target.mv == 5_000 && matches!(target.supply, Supply::Fixed));
                    if !at_5v {
                        CABLE_5V_MS.store(0, Ordering::Relaxed);
                    } else {
                        let now = selection.now_ms().min(u64::from(u32::MAX)) as u32;
                        // The FIRST sighting latches (kws-x1's
                        // `unwrap_or(now)`); a sliding timestamp would
                        // never reach the settle.
                        let since = match CABLE_5V_MS.compare_exchange(
                            0,
                            now,
                            Ordering::Relaxed,
                            Ordering::Relaxed,
                        ) {
                            Ok(_) => now,
                            Err(previous) => previous,
                        };
                        if now.saturating_sub(since) >= 300 && selection.request_cable() {
                            CABLE_AFTER_5V.store(false, Ordering::Relaxed);
                            CABLE_5V_MS.store(0, Ordering::Relaxed);
                            CABLE_WINDOW.store(true, Ordering::Relaxed);
                            BUSY.store(true, Ordering::Relaxed);
                            crate::pd_log(b"[cable] ask armed at 5V\r\n");
                        }
                    }
                }
                // r238: the SVID whose MODES section is on view gets its
                // Discover Modes read (kws-x1's control_cable_selection,
                // driven there by their detail page's section).
                if pd3
                    && !selection.cable_active()
                    && !selection.has_pending()
                    && observation
                        .spr
                        .is_some_and(|spr| {
                            spr.cable_modes.status == pd_protocol::pd_cable_modes::Status::Ack
                        })
                {
                    let section = CABLE_SECTION.load(Ordering::Relaxed);
                    let page = u32::from(section).saturating_sub(1);
                    let data = observation.spr.map(|spr| CableData {
                        report: spr.cable_report,
                        modes: spr.cable_modes,
                        active: spr.cable_active,
                        spr: true,
                        asked: false,
                        failed: false,
                        peer_revision: spr.peer_revision,
                        supply: spr.supply,
                        target_mv: spr.target_mv,
                    });
                    let index = data
                        .as_ref()
                        .and_then(|data| crate::cable_page::svid_index(data, page));
                    if let Some(index) = index {
                        let cursor = index.saturating_add(1).min(255) as u8;
                        if CABLE_MODE_CURSOR.load(Ordering::Relaxed) != cursor {
                            let svid = store.and_then(|store| store.at(index));
                            let current = observation
                                .spr
                                .map(|spr| spr.cable_modes.mode.svid)
                                .unwrap_or(0);
                            if svid == Some(current) && current != 0 {
                                CABLE_MODE_CURSOR.store(cursor, Ordering::Relaxed);
                            } else if svid.is_some_and(|svid| selection.request_cable_mode(svid)) {
                                // request_cable_mode armed it; svid is
                                // Some (the predicate only ran then).
                                let svid = svid.unwrap_or_default();
                                CABLE_MODE_CURSOR.store(cursor, Ordering::Relaxed);
                                CABLE_WINDOW.store(true, Ordering::Relaxed);
                                BUSY.store(true, Ordering::Relaxed);
                                if let Some(spr) = observation.spr.as_mut() {
                                    spr.cable_modes.mode = pd_protocol::pd_cable_modes::Modes {
                                        svid,
                                        ..Default::default()
                                    };
                                }
                                let mut d = [0u8; 32];
                                let mut n = 0;
                                for &b in b"[cable] mode svid=" {
                                    d[n] = b;
                                    n += 1;
                                }
                                for shift in [12u16, 8, 4, 0] {
                                    d[n] = b"0123456789ABCDEF"[((svid >> shift) & 15) as usize];
                                    n += 1;
                                }
                                for &b in b"\r\n" {
                                    d[n] = b;
                                    n += 1;
                                }
                                crate::pd_log(&d[..n]);
                            }
                        }
                    }
                }
                let established = selection.established_target();
                let commit = COMMIT.swap(false, Ordering::Relaxed);
                let desired = DESIRED_MV.load(Ordering::Relaxed);
                let desired_ma = DESIRED_MA.load(Ordering::Relaxed);
                let pps = DESIRED_PPS.load(Ordering::Relaxed);
                let due = match established {
                    // boot: arm the fixed default before any contract
                    // (a PPS request only ever comes from the screen).
                    None => !pps,
                    Some(current) => {
                        commit
                            && (current.mv != desired
                                || matches!(current.supply, Supply::Pps(_)) != pps
                                || (pps && current.ma != desired_ma))
                    }
                };
                if !due {
                    return;
                }
                // r235: a PPS request — the object whose advertised range
                // holds the desired voltage (the position stored at the
                // caps site); preview() then validates mv/ma against the
                // ENGINE's own view of the offer, so stale caps only log.
                let target = if pps {
                    let mut range = None;
                    for k in 0..PPS_N.load(Ordering::Relaxed).min(PPS_OBJ.len()) {
                        let min = PPS_MIN_MV[k].load(Ordering::Relaxed);
                        let max = PPS_MAX_MV[k].load(Ordering::Relaxed);
                        if desired >= min && desired <= max {
                            range = Some(PPS_OBJ[k].load(Ordering::Relaxed));
                            break;
                        }
                    }
                    let Some(object) = range else {
                        crate::pd_log(b"[pd] pps: no range holds mv\r\n");
                        return;
                    };
                    pd_spr::Target {
                        mv: desired,
                        ma: desired_ma,
                        supply: Supply::Pps(object),
                    }
                } else {
                    let mut match_ = None;
                    for index in 0..selection.option_count() {
                        if let Some(target) = selection.option(index)
                            && matches!(target.supply, Supply::Fixed)
                            && target.mv == desired
                        {
                            match_ = Some(target);
                            break;
                        }
                    }
                    let Some(target) = match_ else {
                        if commit {
                            crate::pd_log(b"[pd] arm: not offered\r\n");
                        }
                        return;
                    };
                    target
                };
                if selection.preview(target) && selection.confirm() {
                    // r176: the exchange window opens HERE, not at session
                    // start: the answer wait (Request -> Accept -> PS_RDY,
                    // or Get_Source_Cap -> caps -> Request -> ...) must run
                    // without a main-task blackout. The PE's answer select
                    // polls the TIMEOUT future first, so a single ~55 ms
                    // draw_pd inside the window loses the Accept (already
                    // in our FIFO) to the SenderResponse timer — the r173
                    // log's code=44 at +63 ms. publish() clears BUSY at
                    // PS_RDY; the session end clears it on failure.
                    BUSY.store(true, Ordering::Relaxed);
                    crate::pd_log(b"[pd] arm ok\r\n");
                } else {
                    // confirm() refuses while rejected/waiting — fresh
                    // caps re-enable it.
                    crate::pd_log(b"[pd] arm refused\r\n");
                }
            };
        // r171: persistent sessions again (kws-x1's shape). The r120
        // one-shot decision existed because a session that acked caps
        // without ever requesting them looped Hard Resets; with the
        // Request working, a contract must be MAINTAINABLE: Ready-state
        // messages answered, contract_seen set, BACK finishing cleanly.
        let config = pd_phy::Config {
            persistent: true,
            selection: Some(&selection),
            // r149: the arming callback above (was None — the engine
            // never transmitted without it).
            control: Some(&control),
            // r238: a fresh identity inspection invalidates the mode
            // cursor (kws-x1's identity_started → set_mode_cursor(None)).
            identity_started: Some(&|| {
                CABLE_MODE_CURSOR.store(0, Ordering::Relaxed);
            }),
            publish: Some(&publish),
            trace_steps: true,
            // r238: the cable breadcrumbs too (each mark publishes the
            // live SOP' state — the CABLE screen follows the read).
            trace_cable: true,
            stop_requested: Some(stop_requested),
            svids: Some(&CABLE_SVID_STORE),
            emarker: false,
        };
        let mut bus = SharedBus;
        let clock = PdClock;
        let mut delay = embassy_time::Delay;
        let mut observation = Observation::new();
        observation.phase = Phase::Negotiating;
        // r155: session wall-clock, for the engine-end d= field (a ~465 ms
        // death = the SinkWaitCap timer, ~1 s+ = the engine's Hard Reset
        // spiral; instant = a begin/boundary failure).
        let session_t0 = embassy_time::Instant::now();
        let session_t0_ms = session_t0.as_millis() as u32;
        fusb302_pd::phy::TX_ENTER_MS.store(0, Ordering::Relaxed);
        fusb302_pd::phy::PRE_TX_MS.store(0, Ordering::Relaxed);
        STAGE.store(3, Ordering::Relaxed); // session start
        crate::dbg_stage(12);
        let (mut phy, started) = pd_phy::Session::start(
            &mut bus,
            &mut delay,
            &clock,
            config,
            observation,
            report,
        )
        .await;
        let mut error = started.err();
        let mut rerun = false; // r144: fresh-attach re-run after our HR
        let mut clean_stop = false; // r145: BACK stop finished the contract
        {
            let code = error.as_ref().map_or(0, error_code);
            let mut d = [0u8; 48];
            let mut n = 0;
            for &b in b"[pd] start err=" {
                d[n] = b;
                n += 1;
            }
            n = crate::push_dec(&mut d, n, u32::from(code));
            // r239f probe: WHICH device NAKed and why.
            if let Some(pd_phy::Error::Bus {
                address,
                register,
                source,
            }) = error
            {
                for &b in b" bus a=" {
                    d[n] = b;
                    n += 1;
                }
                n = crate::push_hex8(&mut d, n, u32::from(address));
                for &b in b" r=" {
                    d[n] = b;
                    n += 1;
                }
                n = crate::push_hex8(&mut d, n, u32::from(register));
                for &b in b" src=" {
                    d[n] = b;
                    n += 1;
                }
                n = crate::push_dec(&mut d, n, i2c_src_code(source));
            }
            d[n] = b'\r';
            d[n + 1] = b'\n';
            crate::pd_log(&d[..n + 2]);
        }
        let mut engine_code = 0u16;
        // r154/r159: fresh refusal + DPM debug markers per attempt.
        fusb302_pd::phy::LAST_TX_REFUSAL.store(0, Ordering::Relaxed);
        pd_protocol::pd_spr::DEBUG_LAST_FAIL.store(0, Ordering::Relaxed);
        pd_protocol::pd_spr::DEBUG_REQ_GATE.store(0, Ordering::Relaxed);
        pd_protocol::pd_spr::DEBUG_REQ_OK.store(0, Ordering::Relaxed);
        pd_protocol::pd_spr::DEBUG_REQ_MV.store(0, Ordering::Relaxed);
        pd_protocol::pd_spr::DEBUG_STAGE.store(0, Ordering::Relaxed);
                // r238: a stale after-5V wait or inspection window dies with
                // the session that armed it (the attempt-end BUSY clear
                // already reopened the draws). The ASK itself stays
                // latched — a deliberate deviation from kws-x1, which
                // drains its PD_COMMANDS queue at session start: fnb58's
                // ask can arrive while no session runs (main.rs may arm a
                // restart for it), and the fresh session's control closure
                // consumes it once the PD 3.0 verdict is known.
                CABLE_AFTER_5V.store(false, Ordering::Relaxed);
                CABLE_5V_MS.store(0, Ordering::Relaxed);
                CABLE_WINDOW.store(false, Ordering::Relaxed);
        if restart_requested() && error.is_none() {
            // r147: a restart press never runs the engine — kws-x1's
            // restart shape (Session::start, then the Hard Reset at once).
            // r172: a LIVE contract (or any heard source) needs the full
            // phy.restart() — restart_unheard REFUSES once anything was
            // received (rx_packets != 0), which made every OK press during
            // the boot contract a silent no-op ("HR NOT sent").
            crate::pd_log(b"[pd] restart: HR now\r\n");
            STAGE.store(4, Ordering::Relaxed); // busy in session
            crate::dbg_stage(15);
            error = if phy.observation.rx_packets != 0
                || phy
                    .observation
                    .spr
                    .is_some_and(|s| s.peer_revision.is_some())
            {
                phy.restart().await.err()
            } else {
                phy.restart_unheard().await.err()
            };
            // r250: the post-HR unwedge — the FUSB302's I2C interface
            // locks up after an unanswered HR transmission (the r239f
            // finding), and every I2C access after it would fail
            // (err=768-era). The dance is SDA-gated: harmless when the
            // bus is healthy, clears the held-SDA lockup before the
            // transition's reads run.
            unsafe { i2c_unwedge() };
            if phy.hard_reset_sent() {
                RESTART.store(false, Ordering::Relaxed); // performed
                match phy.transition_to_default().await {
                    // r171: NO reboot on OK. The sys_reset wiped DESIRED_MV
                    // (the chosen voltage reverted to 5 V) and made every OK
                    // press a full power cycle on charger-only boards;
                    // transition_to_default already re-baselined the PHY,
                    // so rerun a fresh session instead.
                    Ok(()) => {
                        rerun = true;
                        crate::pd_log(b"[pd] restart: default, rerun\r\n");
                    }
                    // r144: with the micro-B PC attached its 5 V holds the
                    // shared VBUS rail up through the charger's HR dip, so
                    // the spec's vSafe0V -> vSafe5V ceremony can never pass
                    // (HardResetVbus = 23). The HR WAS sent and honored -
                    // which is the point: re-run a session at once, inside
                    // the source's fresh announce window, no reboot needed.
                    Err(failure) => {
                        VIEW.error.store(error_code(&failure), Ordering::Relaxed);
                        rerun = true;
                        crate::dbg_stage(16);
                        crate::pd_log(b"[pd] restart: vbus held, rerun\r\n");
                    }
                }
            } else {
                crate::pd_log(b"[pd] restart: HR NOT sent\r\n");
                if error.is_none() {
                    // nothing happened; drop the request so it cannot loop
                    RESTART.store(false, Ordering::Relaxed);
                    START.reset();
                }
            }
        } else if error.is_none() {
            let trace = RefCell::new(pd_spr::MaintainedTrace::default());
            let mut trial = pd_spr::Trial::maintained();
            STAGE.store(4, Ordering::Relaxed); // engine running
            crate::dbg_stage(13);
            match trial
                .run_selectable::<_, PdClock>(&mut phy, &trace, &selection)
                .await
            {
                // The engine loop ended without a stop request: kws-x1
                // codes this 43 (should not happen for a persistent run).
                Ok(()) => engine_code = 43,
                Err(failure) => {
                    engine_code = engine_error_code(&failure);
                    error = phy.take_error();
                    if error.is_none() && matches!(failure, pd_spr::Error::Stopped) {
                        // A requested stop: BACK finishes the contract
                        // cleanly; an OK restart continues below with the
                        // Hard Reset path.
                        if !restart_requested() {
                            let contract = trace.borrow().contract.clone();
                            if let Some(contract) = contract {
                                if let Err(f) = phy.finish(&contract).await {
                                    error = Some(f);
                                } else {
                                    // r145: a clean BACK stop is not a
                                    // failure (it used to report err 32).
                                    clean_stop = true;
                                }
                            }
                        }
                    }
                }
            }
            // r153: what the engine actually asked for — the reviewed RDO
            // from the maintained trace (offer vs request, finally both
            // visible in one log).
            if let Some(contract) = trace.borrow().contract.clone() {
                let mut d = [0u8; 32];
                let mut n = 0;
                for &b in b"[pd] rdo=" {
                    d[n] = b;
                    n += 1;
                }
                for shift in [24u32, 16, 8, 0] {
                    n = crate::push_hex8(&mut d, n, (contract.request.0 >> shift) & 0xFF);
                }
                d[n] = b'\r';
                d[n + 1] = b'\n';
                n += 2;
                crate::pd_log(&d[..n]);
            }
        }
        if error.is_none() {
            error = phy.take_error();
        }
        // r156: the offer FIRST, the engine line LAST — the engine line
        // carries the diagnosis, so it must be the newest ring entry, not
        // the one a following burst pushes out. All six objects now.
        if let Some(caps) = phy.observation.capabilities {
            let mut d = [0u8; 24];
            let mut n = 0;
            for &b in b"[pd] caps n=" {
                d[n] = b;
                n += 1;
            }
            n = crate::push_dec(&mut d, n, u32::from(caps.count));
            d[n] = b'\r';
            d[n + 1] = b'\n';
            n += 2;
            crate::pd_log(&d[..n]);
            // r236: the store now runs LIVE in publish() (a persistent
            // contract never reaches this post-session logging); this
            // call covers the one-shot paths.
            store_caps(&caps);
            for (i, object) in caps.objects.iter().take(6).enumerate() {
                if i >= usize::from(caps.count) {
                    break;
                }
                // r153: the PARSED fixed values too — raw alone left the
                // offer ambiguous by hand.
                let mut d = [0u8; 48];
                let mut n = 0;
                for &b in b"[pd] p" {
                    d[n] = b;
                    n += 1;
                }
                d[n] = b'1' + i as u8;
                n += 1;
                d[n] = b'=';
                n += 1;
                for shift in [24u32, 16, 8, 0] {
                    n = crate::push_hex8(&mut d, n, (object.raw >> shift) & 0xFF);
                }
                if let Some((mv, ma)) = object.fixed {
                    d[n] = b' ';
                    n += 1;
                    n = crate::push_dec(&mut d, n, u32::from(mv));
                    for &b in b"mV" {
                        d[n] = b;
                        n += 1;
                    }
                    n = crate::push_dec(&mut d, n, u32::from(ma));
                    for &b in b"mA" {
                        d[n] = b;
                        n += 1;
                    }
                }
                d[n] = b'\r';
                d[n + 1] = b'\n';
                n += 2;
                crate::pd_log(&d[..n]);
            }
        }
        {
            let code = if engine_code != 0 {
                engine_code
            } else {
                error.as_ref().map_or(0, error_code)
            };
            // r148: the SprObservation trace — WHY nothing was sent.
            // w = tx_written, g = tx GoodCRC seen, c = sink_tx_cause (the
            // phy's refusal char: G SinkTxOK gate, C source message won,
            // Q quiet override, V cable-safe-5V, '-' none), r = request
            // MessageID, and on a TX timeout its STATUS1 + INTERRUPTA.
            // r155: h/n/s/d appended. r172: 96 -> 160 bytes — push_dec is
            // uncapped by design, and the FIRST successful contract's line
            // (e=132 p=133 v=5000 st=3 ...) crossed 96 and PANICKED
            // mid-write (pd_task.rs:913, the r171 hw log) — the same
            // growth-kills-the-line class as the r157/r161 silent drops,
            // now as a crash.
            let mut d = [0u8; 160];
            let mut n = 0;
            for &b in b"[pd] engine code=" {
                d[n] = b;
                n += 1;
            }
            n = crate::push_dec(&mut d, n, u32::from(code));
            if let Some(spr) = phy.observation.spr {
                for &b in b" w=" {
                    d[n] = b;
                    n += 1;
                }
                d[n] = b'0' + u8::from(spr.tx_written);
                n += 1;
                for &b in b" g=" {
                    d[n] = b;
                    n += 1;
                }
                d[n] = b'0' + u8::from(spr.tx_goodcrc);
                n += 1;
                for &b in b" c=" {
                    d[n] = b;
                    n += 1;
                }
                d[n] = if spr.sink_tx_cause.is_ascii_alphabetic() {
                    spr.sink_tx_cause
                } else {
                    b'-'
                };
                n += 1;
                for &b in b" r=" {
                    d[n] = b;
                    n += 1;
                }
                match spr.request_id {
                    Some(id) => d[n] = b'0' + (id & 7),
                    None => d[n] = b'-',
                }
                n += 1;
                // r154: the phy's last refused-transmission gate
                // (see fusb302-pd phy.rs LAST_TX_REFUSAL).
                for &b in b" x=" {
                    d[n] = b;
                    n += 1;
                }
                d[n] = b'0' + fusb302_pd::phy::LAST_TX_REFUSAL.load(Ordering::Relaxed);
                n += 1;
                if let Some(timeout) = spr.tx_timeout {
                    for &b in b" t=1 s1=" {
                        d[n] = b;
                        n += 1;
                    }
                    n = crate::push_hex8(&mut d, n, u32::from(timeout.status1));
                    for &b in b" ia=" {
                        d[n] = b;
                        n += 1;
                    }
                    n = crate::push_hex8(&mut d, n, u32::from(timeout.flags));
                }
            }
            // r155: the four facts the x=0 sessions left unanswered.
            // h = last received header (the PHY's decode vs what the
            // ENGINE got), n = packets, s = an HR was SENT (the engine's
            // own SinkWaitCap-timeout Hard Reset is invisible to x=),
            // d = session wall-clock ms (465 = SinkWaitCap death,
            // 1000+ = the engine HR spiral, instant = begin failure).
            for &b in b" h=" {
                d[n] = b;
                n += 1;
            }
            match phy.observation.rx_last_header {
                Some(h) => {
                    for shift in [12u16, 8, 4, 0] {
                        n = crate::push_hex8(&mut d, n, u32::from((h >> shift) & 0xF));
                    }
                }
                None => {
                    for &b in b"----" {
                        d[n] = b;
                        n += 1;
                    }
                }
            }
            for &b in b" n=" {
                d[n] = b;
                n += 1;
            }
            n = crate::push_dec(&mut d, n, u32::from(phy.observation.rx_packets));
            for &b in b" s=" {
                d[n] = b;
                n += 1;
            }
            d[n] = b'0' + u8::from(phy.hard_reset_sent());
            n += 1;
            // r158: the caps-arrival timestamp (boot_step_ms = the step-5
            // ms from session start) - the hardware race decider: caps
            // late in a ~130 ms session means the charger's announce->HR
            // window beat us; caps early means the engine stalled.
            for &b in b" m=" {
                d[n] = b;
                n += 1;
            }
            n = crate::push_dec(&mut d, n, u32::from(phy.observation.boot_step_ms));
            // r159: the vendored crate's debug markers, straight from the
            // [patch]ed pd-protocol-debug build: f=last fail code (3
            // UnsafeCapabilities, 10 RecoveryBlocked, 11 TransmissionBlocked,
            // 4 Peer, 12 Engine), g=request_fixed gate (60..=64), q=the DPM's
            // target.request ok flag, v=the requested target mV. On the host
            // repro f=0 q=1 g=0 with caps delivered; hardware divergence
            // names the exact predicate.
            for &b in b" f=" {
                d[n] = b;
                n += 1;
            }
            n = crate::push_dec(
                &mut d,
                n,
                u32::from(pd_protocol::pd_spr::DEBUG_LAST_FAIL.load(Ordering::Relaxed)),
            );
            for &b in b" g=" {
                d[n] = b;
                n += 1;
            }
            n = crate::push_dec(
                &mut d,
                n,
                u32::from(pd_protocol::pd_spr::DEBUG_REQ_GATE.load(Ordering::Relaxed)),
            );
            for &b in b" q=" {
                d[n] = b;
                n += 1;
            }
            n = crate::push_dec(
                &mut d,
                n,
                u32::from(pd_protocol::pd_spr::DEBUG_REQ_OK.load(Ordering::Relaxed)),
            );
            for &b in b" v=" {
                d[n] = b;
                n += 1;
            }
            n = crate::push_dec(
                &mut d,
                n,
                pd_protocol::pd_spr::DEBUG_REQ_MV.load(Ordering::Relaxed),
            );
            // r161: where the 77 ms went. st= the vendored crate's stage
            // (1 DPM::request entered, 2 Borrowed::transmit entered,
            // 3 request done); e= Session::transmit entry, p= the pre-TX
            // poll - both as ms after session start (0xffffffff/0 = the
            // step never ran).
            for &b in b" st=" {
                d[n] = b;
                n += 1;
            }
            n = crate::push_dec(
                &mut d,
                n,
                u32::from(pd_protocol::pd_spr::DEBUG_STAGE.load(Ordering::Relaxed)),
            );
            for &b in b" e=" {
                d[n] = b;
                n += 1;
            }
            {
                let at = fusb302_pd::phy::TX_ENTER_MS.load(Ordering::Relaxed);
                if at == 0 {
                    d[n] = b'-';
                    n += 1;
                } else {
                    n = crate::push_dec(&mut d, n, at.wrapping_sub(session_t0_ms));
                }
            }
            for &b in b" p=" {
                d[n] = b;
                n += 1;
            }
            {
                let at = fusb302_pd::phy::PRE_TX_MS.load(Ordering::Relaxed);
                if at == 0 {
                    d[n] = b'-';
                    n += 1;
                } else {
                    n = crate::push_dec(&mut d, n, at.wrapping_sub(session_t0_ms));
                }
            }
            for &b in b" d=" {
                d[n] = b;
                n += 1;
            }
            n = crate::push_dec(
                &mut d,
                n,
                embassy_time::Instant::now()
                    .saturating_duration_since(session_t0)
                    .as_millis()
                    .min(99_999) as u32,
            );
            d[n] = b'\r';
            d[n + 1] = b'\n';
            crate::pd_log(&d[..n + 2]);
        }
        if let Some(failure) = &error {
            let code = error_code(failure);
            VIEW.error.store(code, Ordering::Relaxed);
            // A Hard Reset received mid-session means the charger reset
            // VBUS: give it the tSrcRecover + tSrcTurnOn window (with
            // margin) before the next attempt can start.
            if code == 7 {
                embassy_time::Timer::after_millis(1_600).await;
            }
        }
        // r141: a finished-with-failure session reports Failed, not a
        // stale Negotiating (kws-x1's wrapper does the same) — on
        // charger-only power the LCD is the only readout, and a dead
        // session showing NEGOTIATING forever misled it. r145: a clean
        // BACK stop is exempt (not a failure).
        if (error.is_some() || engine_code != 0) && !clean_stop {
            phy.observation.phase = Phase::Failed;
        }
        publish(phy.observation);
        if error.is_none() && engine_code != 0 && !clean_stop {
            VIEW.error.store(engine_code, Ordering::Relaxed);
        }

        // r236: the boot dead-line wake — nothing heard, no caps, but the
        // diagnostic saw a source attached (BC_LVL on either CC): the
        // charger is asleep in the no-ack regime. Fire the r147 restart
        // flow once (clearing `error` so the restart branch below runs —
        // the exact equivalent of the user pressing OK on a dead line).
        if boot_wake
            && phy.observation.capabilities.is_none()
            && phy.observation.rx_packets == 0
            && (VIEW.cc1.load(Ordering::Relaxed) | VIEW.cc2.load(Ordering::Relaxed)) & 0x03 != 0
        {
            boot_wake = false;
            crate::pd_log(b"[pd] boot dead line: HR wake\r\n");
            RESTART.store(true, Ordering::Relaxed);
            error = None;
            rerun = true;
        }

        // r145/r147: a failed restart must not chain (a still-set RESTART
        // plus any pending START signal would loop Hard Reset attempts) —
        // and unlike r146 it is never silent.
        if restart_requested() && error.is_some() {
            crate::pd_log(b"[pd] restart FAIL\r\n");
            RESTART.store(false, Ordering::Relaxed);
            START.reset();
        }
        if !phy.stopped() {
            phy.stop();
        }
        STAGE.store(5, Ordering::Relaxed); // finish / restore
        crate::dbg_stage(14);
        // r139: a failed session must not leave the FUSB302 acking caps —
        // the r138 charger-only reboot loop: the auto-GoodCRC keeps the
        // source waiting for a Request that never comes, its Hard Reset
        // dips VBUS. Blind-restore the power-on baseline exactly like the
        // session-start path (r114); the next attempt re-verifies anyway.
        if error.is_some() || engine_code != 0 {
            let mut guard = BUS.lock().await;
            if let Some(bus) = guard.0.as_mut() {
                let ok = restore_baseline(bus).await.is_ok();
                crate::pd_log(if ok {
                    b"[pd] baseline restored\r\n"
                } else {
                    b"[pd] baseline restore FAIL\r\n"
                });
            }
        }
        if rerun {
            // r144: our Hard Reset was sent and honored; run the next
            // session immediately (the engine's caps wait catches the
            // source's fresh announce whenever it comes). The flag stays
            // set: the rerun starts at once.
            START.signal(());
        } else {
            SESSION_ACTIVE.store(false, Ordering::Relaxed);
        }
        BUSY.store(false, Ordering::Relaxed);
    }
}

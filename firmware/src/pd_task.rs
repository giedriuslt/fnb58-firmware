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
use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU16, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use fusb302_pd::observation::{Observation, Phase};
use fusb302_pd::startup::Report;
use pd_protocol::pd_spr;

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
/// r139 stall-watch breadcrumb: 1 = USB gate, 2 = CC diagnostic,
/// 3 = session start, 4 = engine, 5 = finish/restore, 6 = parked on OK.
pub static STAGE: AtomicU8 = AtomicU8::new(0);
/// r160: true for the whole diagnostic+session+restore span. The main
/// loop freezes ALL screen draws, meter/accel reads and other SPI/I2C
/// side-work while this is set: Spi::write_busy is a SYNCHRONOUS loop
/// that blocks the cooperative executor for the whole transfer (a
/// draw_pd fill ~230 ms at 4 MHz SPI), and a mid-session blackout
/// starves the engine's transmit chain past the charger's ~77 ms
/// caps->Hard-Reset window (r159: f=6, the HR caught at the pre-TX
/// poll, Request built but never written - q=1 v=5000).
pub static SESSION_ACTIVE: AtomicBool = AtomicBool::new(false);
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

fn publish(observation: Observation) {
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
    }
    VIEW.boot_step.store(observation.boot_step, Ordering::Relaxed);
    VIEW.tx_written
        .store(observation.spr.is_some_and(|spr| spr.tx_written), Ordering::Relaxed);
    VIEW.rx_error.store(observation.rx_error, Ordering::Relaxed);
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
async fn restore_baseline(bus: &mut embassy_at32::i2c::I2c) -> Result<(), ()> {
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
fn error_code(error: &pd_phy::Error<embassy_at32::i2c::Error>) -> u16 {
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

#[embassy_executor::task]
pub async fn run() {
    // r139: the boot session runs on PC power — USB configured within a
    // BOUNDED 3 s (hosts finish enumeration in ~1 s, still well inside
    // the source's ~10 s caps window). r138 started it unconditionally
    // and charger-only power reboot-looped: the session's automatic
    // GoodCRCs string the source along, its Hard Reset dips VBUS, the
    // board browns out and the next boot acks fresh caps again (the r119
    // wall; the HR is the charger's, so one-shot mode cannot prevent it).
    // The boot LOG waits for nothing either way: the 4096-byte ring
    // (main.rs BootLog) captures everything and flushes on connection.
    // STAGE breadcrumbs feed the SysTick stall watch in main.rs.
    unsafe extern "C" {
        fn fnb58_usb_configured() -> u32;
    }
    let boot_ms = embassy_time::Instant::now().as_millis();
    STAGE.store(1, Ordering::Relaxed); // 1 = USB gate
    let mut powered = false;
    for _ in 0..30 {
        if unsafe { fnb58_usb_configured() } == 1 {
            powered = true;
            break;
        }
        embassy_time::Timer::after_millis(100).await;
    }
    // r142: charger-only OK presses INSIDE the source's post-attach caps
    // window (~15 s: nCapsCount + NoResponse HR + one re-announce) string
    // the source along with our auto-GoodCRC, and its Hard Reset dips
    // VBUS - the r119 brownout, once per press (r141 hw result). A session
    // that starts AFTER the window is harmless: the idle source never
    // transmits, nothing is acked, no HR. Defer charger-only sessions
    // until 20 s past boot; the LCD shows FAIL 45 meanwhile.
    let charger_only = !powered;
    crate::pd_log(if powered {
        b"[pd] pc power: boot session\r\n"
    } else {
        b"[pd] no usb 3s: OK-only\r\n"
    });
    let mut first = powered;
    loop {
        if !first {
            STAGE.store(6, Ordering::Relaxed); // parked, waiting for OK
            START.wait().await;
        }
        first = false;
        // r145: RESTART is NO LONGER wiped here. request_restart() is the
        // only START signaler, so wiping it turned every OK press into a
        // plain session (the r144 log: no HR, no rerun). The flag is
        // consumed below once the restart has been performed; every OK
        // press is a restart now.
        // r142 source-window deferral (charger-only): see `charger_only`.
        // Error 45 = "source announce window, wait and press again".
        // r147: a PC arriving LATE (past the 3 s gate — the cause of a
        // confusing 'no usb 3s' boot with a working console) still counts:
        // its 5 V backs the board through HR dips, so from that moment
        // the deferral is unnecessary.
        if charger_only && unsafe { fnb58_usb_configured() } != 1 {
            const SRC_WINDOW_MS: u64 = 20_000;
            let elapsed = embassy_time::Instant::now().as_millis() - boot_ms;
            if elapsed < SRC_WINDOW_MS {
                VIEW.error.store(45, Ordering::Relaxed);
                VIEW.phase.store(phase_code(Phase::Failed), Ordering::Relaxed);
                crate::pd_log(b"[pd] src window, wait\r\n");
                continue;
            }
        }
        STAGE.store(2, Ordering::Relaxed); // CC diagnostic
        crate::dbg_stage(11);
        SESSION_ACTIVE.store(true, Ordering::Relaxed);
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
                continue;
            };
            // Restore the power-on configuration FIRST: a previous session
            // that died mid-startup leaves the registers in session state
            // and the crate's Baseline gate would fail every later attempt.
            if restore_baseline(bus).await.is_err() {
                VIEW.error.store(0x300, Ordering::Relaxed);
                continue;
            }
            let snapshot = match fusb302_pd::snapshot::snapshot(bus).await {
                Ok(snapshot) => snapshot,
                Err(failure) => {
                    VIEW.error.store(u16::from(failure.register) | 0x100, Ordering::Relaxed);
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
                    VIEW.error.store(
                        match failure.error {
                            fusb302_pd::config::Error::Baseline { register, .. }
                            | fusb302_pd::config::Error::Verify { register, .. } => {
                                u16::from(register) | 0x100
                            }
                            fusb302_pd::config::Error::Bus { register, .. } => {
                                u16::from(register) | 0x200
                            }
                        },
                        Ordering::Relaxed,
                    );
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
        // r149: ARM the first Request ourselves. The shared engine NEVER
        // auto-requests — kws-x1's product sends PdCommand::Confirm from
        // its selector UI ("nothing is transmitted before Confirm",
        // firmware-core buttons.rs), and our config had control: None, so
        // the selection's pending stayed None forever: caps arrived and
        // were auto-GoodCRC'd, no Request followed, and the source
        // Hard-Reset us (the r147/r148 st=5/0/1 code 37/40 wall — the
        // same shape as r115's mystery). Once caps are advertised, request
        // the highest offered fixed target.
        let control =
            |_observation: &mut Observation, _store: Option<&dyn pd_protocol::pd_svids::Store>| {
                use pd_protocol::pd_spr::Supply;
                if selection.established_target().is_none()
                    && !selection.has_pending()
                    && !selection.request_in_flight()
                {
                    let mut best = None;
                    for index in 0..selection.option_count() {
                        if let Some(target) = selection.option(index)
                            && matches!(target.supply, Supply::Fixed)
                            && best.is_none_or(|current: pd_protocol::pd_spr::Target| {
                                target.mv > current.mv
                            })
                        {
                            best = Some(target);
                        }
                    }
                    if let Some(target) = best
                        && selection.preview(target)
                    {
                        selection.confirm();
                    }
                }
            };
        // r120: ONE-SHOT sessions. A persistent engine loops Hard Resets
        // when the charger refuses to acknowledge the Request, and every
        // Hard Reset dips VBUS - a brownout on charger-only power. One
        // clean attempt per OK press; the result lands in the view.
        let config = pd_phy::Config {
            persistent: false,
            selection: Some(&selection),
            // r149: the arming callback above (was None — the engine
            // never transmitted without it).
            control: Some(&control),
            identity_started: None,
            publish: Some(&publish),
            trace_steps: true,
            trace_cable: false,
            stop_requested: Some(stop_requested),
            svids: None,
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
            let mut d = [0u8; 32];
            let mut n = 0;
            for &b in b"[pd] start err=" {
                d[n] = b;
                n += 1;
            }
            n = crate::push_dec(&mut d, n, u32::from(code));
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
        if restart_requested() && error.is_none() {
            // r147: a restart press never runs the engine — kws-x1's
            // restart_unheard shape (Session::start, then the Hard Reset
            // at once, against a source never heard). The r143-r146
            // post-engine phy.restart() was refused silently three rounds
            // in a row (the engine's failure leaves the session unusable
            // for it; the old no-chain guard swallowed the error without
            // a log line), so the HR never went out.
            crate::pd_log(b"[pd] restart: HR now\r\n");
            STAGE.store(4, Ordering::Relaxed); // busy in session
            crate::dbg_stage(15);
            error = phy.restart_unheard().await.err();
            if phy.hard_reset_sent() {
                RESTART.store(false, Ordering::Relaxed); // performed
                match phy.transition_to_default().await {
                    Ok(()) => {
                        cortex_m::peripheral::SCB::sys_reset();
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
            // r155: h/n/s/d appended - the worst-case line is ~64 chars.
            let mut d = [0u8; 96];
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
    }
}

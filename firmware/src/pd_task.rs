//! The PD sink session task: waits for the OK request on the PD screen,
//! runs the one-shot CC diagnostic and the persistent sink session from the
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

use crate::pd_phy::{
    self, Clock, FnbLimits, MEASURE, SharedBus, spin, wait_for_rx_idle,
};

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
const BACK_BIT: u8 = 0x01;

/// What the UI renders on the PD screen. Atomics: the task writes, the main
/// loop reads, no locks.
pub struct View {
    /// `Phase` code (Waiting 0, Negotiating 2, Contracted 3, Failed 4, ...).
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

/// The engine's error mapped to a diagnostic code for the PD screen.
/// Write the power-on configuration back: a session that dies mid-startup
/// (Hard Reset received, bus error) leaves SWITCHES0/1 and POWER in session
/// state, and every later attempt would fail the crate's Baseline gate
/// (the r113 first-run err=0x102 = register 0x02). DEVICE_ID is read-only
/// and skipped.
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
        let _ = bus.write(0x22, &[*reg, *value]).await.map_err(|_| ())?;
    }
    Ok(())
}


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
    // The BOOT session starts immediately: the charger is still announcing
    // its capabilities (nCapsCount: up to ~50 announcements over ~10 s) and
    // the engine's passive wait (~16 s budget) catches them — kws-x1's
    // exact boot flow. Later sessions wait for an OK request.
    let mut first = true;
    loop {
        if !first {
            START.wait().await;
        }
        first = false;
        RESTART.store(false, Ordering::Relaxed);

        // One-shot CC diagnostic on the power-on configuration: measures
        // both CCs with Rd on both and listens for the source's traffic;
        // its report feeds the session start.
        let report: Option<Report> = {
            let mut guard = BUS.lock().await;
            let Some(bus) = guard.0.as_mut() else {
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

        // --- Phase B: the persistent sink session on the fresh attach.
        // The engine selects the target and drives Requests; our Platform
        // supplies VBUS and policy.
        let selection = pd_spr::Selection::<FnbLimits>::new();
        let config = pd_phy::Config {
            persistent: true,
            selection: Some(&selection),
            control: None,
            identity_started: None,
            publish: Some(&publish),
            trace_steps: true,
            trace_cable: false,
            stop_requested: Some(stop_requested),
            svids: None,
            emarker: false,
        };
        let report = {
            let mut guard = BUS.lock().await;
            let Some(bus) = guard.0.as_mut() else {
                continue;
            };
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
                        u8::from(report.listen.is_some_and(|l| l.traffic.is_some())),
                        Ordering::Relaxed,
                    );
                    Some(report)
                }
                Err(failure) => {
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
                    None
                }
            }
        };
        let Some(report) = report else { continue };

        let selection = pd_spr::Selection::<FnbLimits>::new();
        let mut bus = SharedBus;
        let clock = PdClock;
        let mut delay = embassy_time::Delay;
        let mut observation = Observation::new();
        observation.phase = Phase::Negotiating;
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
        let mut engine_code = 0u16;
        if error.is_none() {
            let trace = RefCell::new(pd_spr::MaintainedTrace::default());
            let mut trial = pd_spr::Trial::maintained();
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
                        // A requested stop: finish the contract cleanly
                        // when one exists (BACK stop; a restart continues
                        // below with the Hard Reset path instead).
                        if restart_requested() {
                            RESTART.store(false, Ordering::Relaxed);
                            if phy.hard_reset_sent() {
                                match phy.transition_to_default().await {
                                    Ok(()) => {
                                        cortex_m::peripheral::SCB::sys_reset();
                                    }
                                    Err(f) => error = Some(f),
                                }
                            }
                        } else {
                            let contract = trace.borrow().contract.clone();
                            if let Some(contract) = contract {
                                if let Err(f) = phy.finish(&contract).await {
                                    error = Some(f);
                                }
                            }
                        }
                    }
                }
            }
        }
        if error.is_none() {
            error = phy.take_error();
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
        publish(phy.observation);
        if error.is_none() && engine_code != 0 {
            VIEW.error.store(engine_code, Ordering::Relaxed);
        }

        // kws-x1's restart: an OK-pressed stop restarts the charger with a
        // Hard Reset, transitions to default, and reboots the firmware so
        // the boot session starts against the fresh attach.
        if restart_requested() && error.is_none() && engine_code == 32 && phy.hard_reset_sent() {
            match phy.transition_to_default().await {
                Ok(()) => {
                    cortex_m::peripheral::SCB::sys_reset();
                }
                Err(failure) => {
                    VIEW.error.store(error_code(&failure), Ordering::Relaxed);
                }
            }
        }
        if !phy.stopped() {
            phy.stop();
        }
    }
}

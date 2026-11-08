//! QC trigger task (r189 QC2.0, r196 QC3.0 continuous): drives the data
//! lines through the DAC board layer (dac.rs), verifies the driven
//! levels with the PA2/PA3 ADC taps, and follows the output on the
//! TPA626 bus voltage the main loop publishes. The protocol state
//! machines are the vendored qc-protocol crate (pure logic,
//! host-tested); this file is the board layer.
//!
//! kws-x1 parity: firmware/src/qc_drive.rs is the reference consumer —
//! the state shapes mirror its Off/Releasing/Entering/Active/Stepping
//! flow minus the capability probe. Entry is guarded the same way: only
//! from a measured ~5 V output with the PD stack idle. Every transition
//! logs a [qc] line — no silent branches.
//!
//! Two screens share this task: the QC2 screen commits Fixed voltages,
//! the QC3 screen commits a Continuous target (mV). The mode rides with
//! the commit (MODE stored before COMMIT), and a commit from the other
//! screen switches flows mid-session — Fixed→Continuous enters stepping
//! from the current output; Continuous→Fixed drives the 5 V pair held
//! (which exits continuous mode) and re-requests after the settle.
//!
//! Coordination (the pd_task convention): atomics + the executor only.
//! No I2C here — the main loop's meter read publishes VBUS_MV.

use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU16, Ordering};
use embassy_time::{block_for, Duration, Instant, Timer};
use qc_protocol::detect::{Follow, Outcome};
use qc_protocol::qc2::{Failure, Handshake, Level, Progress, Session, Voltage, REQUEST_SETTLE_MS};
use qc_protocol::qc3::Continuous;

use crate::adc;
use crate::dac::{self, Line};

// --- UI-facing state (atomics only) ---

/// Selected QC2 target: an index into Voltage::ALL (0 = V5 .. 3 = V20).
/// Default 9 V — the reason the QC2 screen exists.
pub static TARGET: AtomicU8 = AtomicU8::new(1);
/// Selected QC3 target in mV (the QC3 screen cycles it in 200 mV steps,
/// clamped to qc3::MIN_MV..=20000).
pub static M3_TARGET: AtomicU16 = AtomicU16::new(9000);
pub static COMMIT: AtomicBool = AtomicBool::new(false);
/// TPA626 bus voltage in mV, published by the main loop's meter read.
pub static VBUS_MV: AtomicU16 = AtomicU16::new(0);
/// Measured line taps in mV (PA3 = D+, PA2 = D−), for the QC screens.
pub static DP_MV: AtomicU16 = AtomicU16::new(0);
pub static DM_MV: AtomicU16 = AtomicU16::new(0);
/// 0 Off, 1 Releasing, 2 Entering, 3 Active (fixed), 4 Failed,
/// 5 Stepping (continuous), 6 Exiting (leaving continuous).
pub static STATE: AtomicU8 = AtomicU8::new(0);
/// 0 none, 1 Waiting, 2 Followed, 3 Ignored.
pub static OUTCOME: AtomicU8 = AtomicU8::new(0);
/// 0 none, 1 NotDcp, 2 NoEntry.
pub static FAILURE: AtomicU8 = AtomicU8::new(0);
/// The continuous-mode set point in mV (the crate's own count), for the
/// QC3 screen.
pub static SET_MV: AtomicU16 = AtomicU16::new(0);

const STATE_OFF: u8 = 0;
const STATE_RELEASING: u8 = 1;
const STATE_ENTERING: u8 = 2;
const STATE_ACTIVE: u8 = 3;
const STATE_FAILED: u8 = 4;
const STATE_STEPPING: u8 = 5;

/// The selectable QC2 targets in cycle order (5 V = the stop/restore).
pub const TARGETS_MV: [u16; 4] = [5000, 9000, 12000, 20000];
/// The QC3 target range (the crate floors stepping at qc3::MIN_MV).
pub const M3_MIN_MV: u16 = 3600;
pub const M3_MAX_MV: u16 = 20_000;

/// Cycle [`TARGET`] one step in `dir` (+1 right / −1 left), wraparound.
/// Returns the new target in mV for the console line.
pub fn cycle_target(dir: i16) -> u16 {
    let current = TARGET.load(Ordering::Relaxed) as usize % TARGETS_MV.len();
    let next = (current as i16 + dir).rem_euclid(TARGETS_MV.len() as i16) as usize;
    TARGET.store(next as u8, Ordering::Relaxed);
    TARGETS_MV[next]
}

/// Cycle the QC3 target by one 200 mV step in `dir`, clamped to the
/// range. Returns the new target in mV for the console line.
pub fn cycle_m3_target(dir: i16) -> u16 {
    let step = i32::from(qc_protocol::qc3::STEP_MV) * i32::from(dir);
    let next = (i32::from(M3_TARGET.load(Ordering::Relaxed)) + step)
        .clamp(i32::from(M3_MIN_MV), i32::from(M3_MAX_MV));
    let next = next as u16;
    M3_TARGET.store(next, Ordering::Relaxed);
    next
}

/// The QC2 screen's OK in selection mode: a fixed-voltage request.
pub fn commit_fixed() {
    MODE.store(MODE_FIXED, Ordering::Relaxed);
    COMMIT.store(true, Ordering::Relaxed);
}

/// The QC3 screen's OK in selection mode: a continuous-mode target.
pub fn commit_continuous() {
    MODE.store(MODE_CONTINUOUS, Ordering::Relaxed);
    COMMIT.store(true, Ordering::Relaxed);
}

fn log(line: &[u8]) {
    crate::pd_log(line);
}

/// kws-x1's DEFAULT_VBUS_MV guard (single_wire_drive/status.rs): QC is
/// only entered from a measured plain-5 V output.
const ENTRY_VBUS_WINDOW: core::ops::RangeInclusive<u32> = 4500..=5500;
/// Release settle before the handshake starts (kws-x1 RELEASE_MS).
const RELEASE_MS: u64 = 100;

fn now_ms() -> u64 {
    Instant::now().as_millis()
}

/// Lines released (the charger's own dividers own the lines again).
fn release_lines() {
    dac::drive(Line::Dp, None);
    dac::drive(Line::Dm, None);
}

fn release_lines_logged() {
    release_lines();
    log(b"[qc] lines released\r\n");
}

fn log_dec(prefix: &[u8], values: &[u32], infixes: &[&[u8]]) {
    let mut d = [0u8; 96];
    let mut n = 0;
    for &b in prefix {
        d[n] = b;
        n += 1;
    }
    for (i, v) in values.iter().enumerate() {
        n = crate::push_dec(&mut d, n, *v);
        for &b in infixes[i] {
            d[n] = b;
            n += 1;
        }
    }
    log(&d[..n]);
}

// --- commit plumbing (mode rides with the commit) ---

const MODE_FIXED: u8 = 0;
const MODE_CONTINUOUS: u8 = 1;
static MODE: AtomicU8 = AtomicU8::new(MODE_FIXED);

/// What the handshake is FOR, captured at the starting commit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Request {
    Fixed(Voltage),
    Continuous(u16),
}

fn request_from_statics() -> Request {
    match MODE.load(Ordering::Relaxed) {
        MODE_CONTINUOUS => Request::Continuous(M3_TARGET.load(Ordering::Relaxed)),
        _ => Request::Fixed(
            Voltage::ALL[TARGET.load(Ordering::Relaxed) as usize % Voltage::ALL.len()],
        ),
    }
}

fn request_mv(request: Request) -> u32 {
    match request {
        Request::Fixed(v) => u32::from(v.millivolts()),
        Request::Continuous(t) => u32::from(t),
    }
}

#[embassy_executor::task]
pub async fn run() {
    dac::init();
    log(b"[qc] task up\r\n");
    let mut state = State::Off;
    loop {
        let now = now_ms();
        let request = request_from_statics();
        let commit = COMMIT.swap(false, Ordering::Relaxed);
        // PA3 = D+ tap, PA2 = D− tap (user trace: 1 kΩ series each).
        let dp_mv = adc::read_line_mv(3);
        let dm_mv = adc::read_line_mv(2);
        if let Some(v) = dp_mv {
            DP_MV.store(v, Ordering::Relaxed);
        }
        if let Some(v) = dm_mv {
            DM_MV.store(v, Ordering::Relaxed);
        }
        let vbus_mv = u32::from(VBUS_MV.load(Ordering::Relaxed));

        state = update(state, now, request, commit, dp_mv, dm_mv, vbus_mv);
        Timer::after_millis(50).await;
    }
}

enum State {
    Off,
    Releasing(u64),
    /// The handshake plus the request entry is FOR (captured at the
    /// starting commit; a later commit retargets it).
    Entering(Handshake, Request),
    /// The fixed-voltage flow: a request held and its output followed.
    Active {
        session: Session,
        voltage: Voltage,
        follow: Option<Follow>,
        decided: bool,
    },
    /// The continuous flow: the mode was taken, steps run toward the
    /// target one pulse per settle window.
    Stepping {
        session: Session,
        continuous: Continuous,
        target: u16,
        last_pulse: u64,
    },
    /// The 5 V pair is held — it exits continuous mode (sustained levels
    /// are not a pulse) and requests 5 V — while the next request
    /// settles behind it.
    Exiting {
        since: u64,
        session: Session,
        request: Request,
    },
    /// A 5 V stop request settles before the lines release.
    Stopping(u64),
}

fn update(
    state: State,
    now: u64,
    request: Request,
    commit: bool,
    _dp_mv: Option<u16>,
    dm_mv: Option<u16>,
    vbus_mv: u32,
) -> State {
    match state {
        State::Off => {
            if !commit {
                return State::Off;
            }
            if request == Request::Fixed(Voltage::V5) {
                // 5 V is the restore request; with the lines released
                // there is nothing to drive.
                log(b"[qc] off: released (5v default)\r\n");
                return State::Off;
            }
            let in_window = ENTRY_VBUS_WINDOW.contains(&vbus_mv);
            let pd_idle = !crate::pd_task::SESSION_ACTIVE.load(Ordering::Relaxed)
                && !crate::pd_task::BUSY.load(Ordering::Relaxed);
            if !in_window || !pd_idle {
                // Loud, with the reason: no silent refusal.
                log_dec(
                    b"[qc] blocked vbus=",
                    &[vbus_mv, u32::from(pd_idle)],
                    &[b"mV pd_idle=", b"\r\n"],
                );
                return State::Off;
            }
            log(b"[qc] releasing lines\r\n");
            release_lines();
            STATE.store(STATE_RELEASING, Ordering::Relaxed);
            State::Releasing(now)
        }
        State::Releasing(since) => {
            if now.saturating_sub(since) < RELEASE_MS {
                return State::Releasing(since);
            }
            // D+ held at 0.6 V, D− released — the charger drives it now
            // (shorts it to 0.6 V, then pulls it low after entry).
            dac::drive(Line::Dp, Some(Level::Mid));
            dac::drive(Line::Dm, None);
            // r190: the t= stamp makes the entry→fail elapsed visible in
            // the console (NotDcp cannot legally fire before 1000 ms).
            log_dec(
                b"[qc] t=",
                &[now as u32 % 100_000],
                &[b" entering dp=600mV dm=release\r\n"],
            );
            STATE.store(STATE_ENTERING, Ordering::Relaxed);
            FAILURE.store(0, Ordering::Relaxed);
            State::Entering(Handshake::start(now), request)
        }
        State::Entering(handshake, entry_request) => {
            // A commit during entry retargets it (the handshake itself is
            // level-only: D+ mid, D− released — retargeting is free).
            let entry_request = if commit && request != entry_request {
                log_dec(
                    b"[qc] entry retarget ",
                    &[request_mv(request)],
                    &[b"mV\r\n"],
                );
                request
            } else {
                entry_request
            };
            // A failed scan keeps waiting (kws-x1: same). D− readings of
            // 0 mV are VALID here — D− going low is the entry signal.
            let Some(dm) = dm_mv else {
                return State::Entering(handshake, entry_request);
            };
            match handshake.poll(now, dm) {
                Progress::Waiting(handshake) => State::Entering(handshake, entry_request),
                Progress::Ready(session) => {
                    log(b"[qc] entry ready\r\n");
                    dispatch(session, entry_request, now, vbus_mv)
                }
                Progress::Failed(failure) => {
                    let name: &[u8] = match failure {
                        Failure::NotDcp => b"NODCP",
                        Failure::NoEntry => b"NOENTRY",
                    };
                    // r190: t= (elapsed since entering is now - its stamp)
                    // plus the last D− reading — NODCP means D− never
                    // showed the charger's 0.6 V short.
                    let mut d = [0u8; 80];
                    let mut n = 0;
                    for &b in b"[qc] t=" {
                        d[n] = b;
                        n += 1;
                    }
                    n = crate::push_dec(&mut d, n, (now % 100_000) as u32);
                    for &b in b" fail " {
                        d[n] = b;
                        n += 1;
                    }
                    d[n..n + name.len()].copy_from_slice(name);
                    n += name.len();
                    for &b in b" dm=" {
                        d[n] = b;
                        n += 1;
                    }
                    n = crate::push_dec(&mut d, n, u32::from(dm));
                    for &b in b"mV\r\n" {
                        d[n] = b;
                        n += 1;
                    }
                    log(&d[..n]);
                    FAILURE.store(
                        match failure {
                            Failure::NotDcp => 1,
                            Failure::NoEntry => 2,
                        },
                        Ordering::Relaxed,
                    );
                    STATE.store(STATE_FAILED, Ordering::Relaxed);
                    release_lines();
                    State::Off
                }
            }
        }
        State::Active {
            session,
            voltage,
            follow,
            decided,
        } => {
            // A commit retargets — or stops, with the 5 V restore request.
            // The vendored Session moves by value, so re-requests rebuild
            // the Active state through begin().
            if commit {
                if request == Request::Fixed(Voltage::V5) && voltage != Voltage::V5 {
                    let (dp, dm) = session.request(Voltage::V5);
                    let (dp_mv, dm_mv) = drive_request(dp, dm);
                    log_dec(
                        b"[qc] stop: req 5v dp=",
                        &[dp_mv, dm_mv],
                        &[b"mV dm=", b"mV settle\r\n"],
                    );
                    STATE.store(STATE_OFF, Ordering::Relaxed);
                    return State::Stopping(now);
                }
                if request != Request::Fixed(voltage) {
                    return dispatch(session, request, now, vbus_mv);
                }
            }
            // Follow the output until it decides (midpoint crossing or
            // the 610 ms timeout); decided outcomes stay latched.
            let (follow, decided) = if let (Some(f), false) = (follow, decided) {
                match f.poll(now, u16::try_from(vbus_mv).unwrap_or(u16::MAX)) {
                    Outcome::Waiting => (Some(f), false),
                    Outcome::Followed => {
                        log(b"[qc] follow: output followed\r\n");
                        OUTCOME.store(2, Ordering::Relaxed);
                        (None, true)
                    }
                    Outcome::Ignored => {
                        log(b"[qc] follow: output ignored\r\n");
                        OUTCOME.store(3, Ordering::Relaxed);
                        (None, true)
                    }
                }
            } else {
                (follow, decided)
            };
            STATE.store(STATE_ACTIVE, Ordering::Relaxed);
            State::Active {
                session,
                voltage,
                follow,
                decided,
            }
        }
        State::Stepping {
            session,
            mut continuous,
            target,
            last_pulse,
        } => {
            // A commit retargets the stepping, switches the flow, or
            // stops (5 V).
            if commit {
                match request {
                    Request::Continuous(t) if t != target => {
                        log_dec(b"[qc] step retarget ", &[u32::from(t)], &[b"mV\r\n"]);
                        return State::Stepping {
                            session,
                            continuous,
                            target: t,
                            last_pulse,
                        };
                    }
                    Request::Continuous(_) => {}
                    other => {
                        // Leave continuous the crate's way: the 5 V pair
                        // held (a sustained pair is not a pulse) requests
                        // 5 V and ends continuous mode.
                        return exit_continuous(session, other, now);
                    }
                }
            }
            // One pulse per settle window toward the target.
            let mut last_pulse = last_pulse;
            if now.saturating_sub(last_pulse) >= REQUEST_SETTLE_MS {
                if let Some(pulse) = continuous.step_toward(target) {
                    let hold = continuous.hold();
                    let _ = drive_request(pulse.0, pulse.1);
                    block_for(Duration::from_micros(u64::from(
                        qc_protocol::qc3::PULSE_US,
                    )));
                    let _ = drive_request(hold.0, hold.1);
                    SET_MV.store(continuous.set_mv(), Ordering::Relaxed);
                    // No per-pulse log: a ramp fires one pulse per settle
                    // window and would flood the ring — the QC3 screen's
                    // SET row shows the progress live.
                    last_pulse = now;
                }
            }
            STATE.store(STATE_STEPPING, Ordering::Relaxed);
            State::Stepping {
                session,
                continuous,
                target,
                last_pulse,
            }
        }
        State::Exiting { since, session, request } => {
            if now.saturating_sub(since) < REQUEST_SETTLE_MS {
                return State::Exiting { since, session, request };
            }
            dispatch(session, request, now, vbus_mv)
        }
        State::Stopping(since) => {
            if now.saturating_sub(since) < REQUEST_SETTLE_MS {
                return State::Stopping(since);
            }
            release_lines_logged();
            STATE.store(STATE_OFF, Ordering::Relaxed);
            State::Off
        }
    }
}

/// Route a settled session to its request: fixed voltages drive and
/// follow; a continuous target takes the mode (from the current output)
/// and steps toward it.
fn dispatch(session: Session, request: Request, now: u64, vbus_mv: u32) -> State {
    match request {
        Request::Fixed(Voltage::V5) => {
            let (dp, dm) = session.request(Voltage::V5);
            let (dp_mv, dm_mv) = drive_request(dp, dm);
            log_dec(
                b"[qc] 5v dp=",
                &[dp_mv, dm_mv],
                &[b"mV dm=", b"mV release\r\n"],
            );
            release_lines();
            STATE.store(STATE_OFF, Ordering::Relaxed);
            State::Off
        }
        Request::Fixed(voltage) => begin(session, voltage, now, vbus_mv),
        Request::Continuous(target) => {
            // The output is wherever the last request left it; continuous
            // mode starts from there (kws-x1 resets to 5 V first — ours
            // starts from the honest current set point).
            let from = Voltage::V5;
            let (continuous, hold) = Continuous::request(&session, from);
            let (dp_mv, dm_mv) = drive_request(hold.0, hold.1);
            SET_MV.store(continuous.set_mv(), Ordering::Relaxed);
            log_dec(
                b"[qc] continuous from=",
                &[u32::from(from.millivolts()), u32::from(target)],
                &[b"mV target=", b"mV\r\n"],
            );
            let _ = (dp_mv, dm_mv);
            STATE.store(STATE_STEPPING, Ordering::Relaxed);
            State::Stepping {
                session,
                continuous,
                target,
                last_pulse: now,
            }
        }
    }
}

/// Leave continuous mode: the 5 V pair held (a sustained pair outlasts
/// the pulse filter, so the charger reads it as a fixed request and ends
/// continuous mode), then settle behind it.
fn exit_continuous(session: Session, request: Request, now: u64) -> State {
    let (dp, dm) = session.request(Voltage::V5);
    let (dp_mv, dm_mv) = drive_request(dp, dm);
    log_dec(
        b"[qc] exit continuous dp=",
        &[dp_mv, dm_mv],
        &[b"mV dm=", b"mV settle\r\n"],
    );
    STATE.store(STATE_OFF, Ordering::Relaxed);
    match request {
        Request::Fixed(Voltage::V5) => State::Stopping(now),
        other => State::Exiting {
            since: now,
            session,
            request: other,
        },
    }
}

/// Request `voltage` from `session`, drive the returned levels, and arm
/// the output follow from the last known VBUS.
fn begin(session: Session, voltage: Voltage, now: u64, vbus_mv: u32) -> State {
    let (dp, dm) = session.request(voltage);
    let (dp_mv, dm_mv) = drive_request(dp, dm);
    log_dec(
        b"[qc] req ",
        &[
            u32::from(voltage.millivolts() / 1000),
            dp_mv,
            dm_mv,
        ],
        &[b"v dp=", b"mV dm=", b"mV\r\n"],
    );
    let follow = if vbus_mv != 0 {
        OUTCOME.store(1, Ordering::Relaxed);
        Some(Follow::start(
            now,
            u16::try_from(vbus_mv).unwrap_or(u16::MAX),
            voltage.millivolts(),
        ))
    } else {
        // No output measurement to judge from — outcome stays none
        // (kws-x1 carries Option<Outcome> for exactly this case).
        OUTCOME.store(0, Ordering::Relaxed);
        None
    };
    STATE.store(STATE_ACTIVE, Ordering::Relaxed);
    State::Active {
        session,
        voltage,
        follow,
        decided: false,
    }
}

/// Drive both lines to the requested levels; returns the applied mV pair
/// for the log line.
fn drive_request(dp: Level, dm: Level) -> (u32, u32) {
    let dp_mv = dac::drive(Line::Dp, Some(dp));
    let dm_mv = dac::drive(Line::Dm, Some(dm));
    (dp_mv, dm_mv)
}

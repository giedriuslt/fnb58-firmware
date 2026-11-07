//! r167 pdmin: the minimal PD probe (user directive: "Remove all external
//! stuff except usb logging and fusb302, when start on boot and try select
//! 9 volts from charger").
//!
//! What is GONE compared to the r166 main build: LCD, buzzer, buttons/UI,
//! W25Q128 flash, TPA626 meter reads, KXTJ3 accelerometer, PC-link mode,
//! console commands, the vendored ENGINE (Trial/Selection/DPM) — and with
//! it every await point between the caps decode and the Request transmit
//! except the Session's own I2C. What stays: the exact r166 console/fault
//! glue (console.rs, shared verbatim), the proven FUSB302 bring-up
//! (diagnostic + Session::start), and the crate's transmit chain — the
//! same gauntlet/FIFO code the engine drives, driven directly.
//!
//! The probe's loop: receive -> on Source_Capabilities build the 9 V
//! request with the vendored `request_fixed` (the crate's own builder, the
//! exact path the host repro validated) -> transmit at once. Every step
//! and every failure branch logs. A silent charger (no packets at all)
//! is woken with the proven restart_unheard Hard Reset flow (r147) on the
//! NEXT attempt; a charger-only power source is NOT supported by this
//! probe (logs need the PC anyway).
//!
//! Deviation from kws-x1 shape, flagged explicitly: kws-x1's engine never
//! transmits before PdCommand::Confirm from its UI. This probe AUTO-
//! requests 9 V at boot — that is the experiment (can the PHY + transmit
//! chain negotiate at all when nothing else runs?). The engine itself is
//! untouched and remains the product path.

#![no_std]
#![no_main]

#[path = "../board.rs"]
mod board;
#[path = "../console.rs"]
mod console;
#[path = "../pd_phy.rs"]
mod pd_phy;
// BUS (the shared I2C1 cell) lives in pd_task; the probe never spawns it.
#[path = "../pd_task.rs"]
mod pd_task;

// pd_phy/pd_task address these through the crate root.
pub(crate) use console::{dbg_stage, pd_log, push_dec, push_hex8};
use console::{fnb58_usb_configured, fnb58_usb_init};

use core::sync::atomic::Ordering;
use embassy_executor::Spawner;
use embassy_time::Timer;
use fusb302_pd::header;
use fusb302_pd::observation::{Observation, Phase};
use fusb302_pd::startup::Report;
use pd_phy::FnbLimits;
use pd_protocol::pd_spr;
use usbpd::protocol_layer::message::data::request::PowerSource;
use usbpd::protocol_layer::message::data::source_capabilities;
use usbpd_traits::Driver as _;
use usbpd_traits::DriverRxError;

/// Attempt budget: wake/catch pairs converge well inside this.
const MAX_ATTEMPTS: u32 = 8;
/// Retries of a Deferred (SinkTxNG) transmit before the attempt ends.
const DEFER_RETRIES: u32 = 3;

struct MinClock;
impl fusb302_pd::phy::Clock for MinClock {
    fn now_ms(&self) -> u64 {
        embassy_time::Instant::now().as_millis()
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // r180: this image is linked at 0x08010000 under the bootloader
    // (first 64 KiB of the bank). Point VTOR at the vector table before
    // anything can raise an exception — cortex-m-rt does not do it for
    // us (set-vtor feature off); the kws-x1 app performs the same write
    // first thing in main.
    unsafe {
        let scb = &*cortex_m::peripheral::SCB::PTR;
        scb.vtor.write(board::APP_BASE);
        core::arch::asm!("dsb", "isb", options(nostack, preserves_flags));
    }

    // Crystal clock: 192 MHz PLL, USB /4 (same profile as the main build).
    let Ok((clocks, _)) = (unsafe { embassy_at32::clocks::configure_usb(board::HEXT_HZ) }) else {
        loop {
            cortex_m::asm::nop();
        }
    };
    unsafe { embassy_at32::init(clocks) };
    unsafe { cortex_m::interrupt::enable() };

    // I2C1 on PB6/PB7 at 100 kHz Standard — the ONLY bus users here are
    // the FUSB302 (0x22) and, inside Session::begin, the Platform's VBUS
    // read (TPA626 register 0x02, the board's qualified voltage source).
    // No meter configuration, no accelerometer, no flash.
    {
        use embassy_at32::gpio::{AlternatePin, Drive, Port};
        use embassy_at32::i2c::{I2c, Speed};
        let _scl = unsafe { AlternatePin::new_open_drain(Port::B, 6, Drive::Moderate) };
        let _sda = unsafe { AlternatePin::new_open_drain(Port::B, 7, Drive::Moderate) };
        let i2c = unsafe { I2c::new_i2c1(clocks.pclk1_hz, Speed::Standard) };
        let mut guard = pd_task::BUS.lock().await;
        *guard = pd_task::BusCell(i2c.ok());
    }

    // r168: configure the TPA626 (the Platform's qualified VBUS source).
    // Without this its bus-voltage register reads garbage and Session::begin
    // fails every first attempt with InputVoltage (start err=3, the r167
    // log's 'attempt=1 works only from attempt=2' pattern). Same config the
    // main build writes (AVG=64 continuous + the 10 mOhm shunt calibration).
    {
        use embedded_hal_async::i2c::I2c as _;
        let mut guard = pd_task::BUS.lock().await;
        if let Some(bus) = guard.0.as_mut() {
            let cfg = bus
                .write(0x40, &[0x00, 0x48, 0x97])
                .await
                .is_ok(); // AVG=64, continuous, 588 us + 588 us
            let cal = bus.write(0x40, &[0x05, 0x28, 0x00]).await.is_ok(); // 5.12 mShunt cal
            let mut d = [0u8; 40];
            let mut n = 0;
            for &b in b"[pdmin] tpa cfg=" {
                d[n] = b;
                n += 1;
            }
            d[n] = b'0' + cfg as u8;
            n += 1;
            for &b in b" cal=" {
                d[n] = b;
                n += 1;
            }
            d[n] = b'0' + cal as u8;
            n += 1;
            d[n] = b'\r';
            d[n + 1] = b'\n';
            console::boot_log().push(&d[..n + 2]);
        }
    }

    // r169: per-transaction I2C timing — the receive-cadence suspect. The
    // r168 log's 62 ms begin() (13 transactions) and the one-to-two poll
    // cycle caps delivery both fit ~5 ms per transaction; this measures it
    // directly. Healthy at 100 kHz: ~3-5 ms for all ten reads.
    {
        use embedded_hal_async::i2c::I2c as _;
        let mut guard = pd_task::BUS.lock().await;
        if let Some(bus) = guard.0.as_mut() {
            let t = embassy_time::Instant::now();
            let mut ok = 0u32;
            let mut id0 = 0u8;
            for _ in 0..10 {
                let mut id = [0u8; 1];
                if bus.write_read(0x22, &[0x01], &mut id).await.is_ok() {
                    ok += 1;
                    id0 = id[0];
                }
            }
            let ms = embassy_time::Instant::now().as_millis() - t.as_millis();
            let mut d = [0u8; 40];
            let mut n = 0;
            for &b in b"[pdmin] i2c 10rd=" {
                d[n] = b;
                n += 1;
            }
            n = push_dec(&mut d, n, ms as u32);
            for &b in b"ms ok=" {
                d[n] = b;
                n += 1;
            }
            n = push_dec(&mut d, n, ok);
            for &b in b" id=" {
                d[n] = b;
                n += 1;
            }
            n = push_hex8(&mut d, n, u32::from(id0));
            d[n] = b'\r';
            d[n + 1] = b'\n';
            console::boot_log().push(&d[..n + 2]);
        }
    }

    spawner.spawn(probe().expect("one probe task"));

    // Unmask USB interrupt lines and bring the CDC console up.
    use embassy_at32::pac::at32f403a::interrupt;
    unsafe {
        cortex_m::peripheral::NVIC::unmask(interrupt::USBFS_L_CAN1_RX0);
        cortex_m::peripheral::NVIC::unmask(interrupt::USBFS_H_CAN1_TX);
        fnb58_usb_init();
    }
    fn say(msg: &[u8]) {
        console::boot_log().push(msg);
    }
    say(b"[pdmin] FNB58 r180: usb log + fusb302 only\r\n");
    dbg_stage(1);

    let mut sender = console::Sender::new();
    let mut tick: u32 = 0;
    loop {
        Timer::after_millis(10).await;
        // Stall-watch stamps (the SysTick probe needs MAIN_SEQ moving).
        console::MAIN_STAGE.store(2, Ordering::Relaxed);
        console::MAIN_SEQ.fetch_add(1, Ordering::Relaxed);
        // PD task lines (split fragments land whole; see pd_log).
        console::drain_pd_log(&mut sender);
        // r138: no USB wait; the boot ring flushes once the host connects.
        if console::boot_log().flush_step(unsafe { fnb58_usb_configured() } == 1) {
            sender.drain();
        }
        tick = tick.wrapping_add(1);
        if tick % 500 == 0 {
            // 5 s heartbeat: probe stage, queue depth, rejected sends, INT
            // count. No PD phase decoding — the [m] lines carry that.
            let mut d = [0u8; 80];
            let mut n = 0;
            for &b in b"[pdmin] hb st=" {
                d[n] = b;
                n += 1;
            }
            n = push_dec(&mut d, n, u32::from(console::PD_STAGE.load(Ordering::Relaxed)));
            for &b in b" q=" {
                d[n] = b;
                n += 1;
            }
            n = push_dec(&mut d, n, sender.qlen() as u32);
            for &b in b" f=" {
                d[n] = b;
                n += 1;
            }
            n = push_dec(&mut d, n, sender.take_fails());
            for &b in b" i=" {
                d[n] = b;
                n += 1;
            }
            n = push_dec(&mut d, n, console::INT_COUNT.load(Ordering::Relaxed) % 100_000);
            for &b in b" last=" {
                d[n] = b;
                n += 1;
            }
            n = push_dec(&mut d, n, u32::from(console::dbg_last_stage()));
            d[n] = b'\r';
            d[n + 1] = b'\n';
            sender.push(&d[..n + 2]);
        }
    }
}

#[embassy_executor::task]
async fn probe() {
    console::PD_STAGE.store(1, Ordering::Relaxed);
    // Log through the ring; the main task flushes it to USB.
    pd_log(b"[pdmin] probe task up\r\n");
    let clock = MinClock;
    let mut attempt: u32 = 0;
    // Set after a session that received NOTHING: the next attempt is a
    // wake (restart_unheard Hard Reset, r147 flow), not a negotiate.
    let mut wake_due = false;
    let mut contracted_9v = false;

    // r169: NEVER park. The r167/r168 probe stopped after MAX_ATTEMPTS and
    // the user's switch toggles then did nothing ("nothing more is logged
    // even if I disable and enable pd switch") — the task had exited. Each
    // exhausted cycle now rests 3 s and restarts the counter, so a fresh
    // attach (charger re-plug, PD switch toggle) is always caught.
    loop {
        attempt += 1;
        if attempt > MAX_ATTEMPTS {
            pd_log(b"[pdmin] cycle done, restart\r\n");
            attempt = 1;
            console::PD_STAGE.store(6, Ordering::Relaxed);
            Timer::after_millis(3000).await;
        }
        let mut d = [0u8; 32];
        let mut n = 0;
        for &b in b"[pdmin] attempt=" {
            d[n] = b;
            n += 1;
        }
        n = push_dec(&mut d, n, attempt);
        if wake_due {
            for &b in b" wake" {
                d[n] = b;
                n += 1;
            }
        }
        d[n] = b'\r';
        d[n + 1] = b'\n';
        pd_log(&d[..n + 2]);

        // --- CC diagnostic (the proven bring-up: baseline restore,
        // snapshot, one-shot measure + listen) ---
        console::PD_STAGE.store(2, Ordering::Relaxed);
        dbg_stage(11);
        let report: Option<Report> = run_diagnostic().await;
        let Some(report) = report else {
            // run_diagnostic logged the reason; back off and retry.
            Timer::after_millis(1000).await;
            continue;
        };

        // --- the session (Session::start runs the checked startup) ---
        console::PD_STAGE.store(3, Ordering::Relaxed);
        dbg_stage(12);
        let config = pd_phy::Config {
            persistent: false,
            selection: None,
            control: None,
            identity_started: None,
            publish: None,
            trace_steps: true,
            trace_cable: false,
            stop_requested: None,
            svids: None,
            emarker: false,
        };
        let mut delay = embassy_time::Delay;
        let mut observation = Observation::new();
        observation.phase = Phase::Negotiating;
        fusb302_pd::phy::TX_ENTER_MS.store(0, Ordering::Relaxed);
        fusb302_pd::phy::PRE_TX_MS.store(0, Ordering::Relaxed);
        fusb302_pd::phy::LAST_TX_REFUSAL.store(0, Ordering::Relaxed);
        let t0 = embassy_time::Instant::now();
        let t0_ms = t0.as_millis() as u32;
        let mut bus = pd_phy::SharedBus;
        let (mut phy, started) = pd_phy::Session::start(
            &mut bus,
            &mut delay,
            &clock,
            config,
            observation,
            report,
        )
        .await;
        log_start(started.as_ref().err());
        if let Err(error) = started {
            log_end(&phy, error_code(&error), t0_ms).await;
            quiesce().await;
            Timer::after_millis(1000).await;
            continue;
        }

        // --- wake flow: the charger was silent last attempt and this
        // fresh session has heard nothing either -> Hard Reset it awake
        // (r147 restart_unheard: refused once anything was received) ---
        if wake_due && phy.observation.rx_packets == 0 {
            console::PD_STAGE.store(5, Ordering::Relaxed);
            dbg_stage(15);
            pd_log(b"[m] wake: HR now\r\n");
            match phy.restart_unheard().await {
                Ok(()) => pd_log(b"[m] wake HR sent\r\n"),
                Err(error) => {
                    pd_log(b"[m] wake HR NOT sent\r\n");
                    log_end(&phy, error_code(&error), t0_ms).await;
                }
            }
            if phy.hard_reset_sent() {
                // On PC power the transition's vSafe0V wait never passes
                // (HardResetVbus = the documented rerun outcome, r144).
                match phy.transition_to_default().await {
                    Ok(()) => pd_log(b"[m] wake default ok\r\n"),
                    Err(error) => {
                        let code = error_code(&error);
                        let mut d = [0u8; 32];
                        let mut n = 0;
                        for &b in b"[m] wake default err=" {
                            d[n] = b;
                            n += 1;
                        }
                        n = push_dec(&mut d, n, u32::from(code));
                        d[n] = b'\r';
                        d[n + 1] = b'\n';
                        pd_log(&d[..n + 2]);
                    }
                }
                wake_due = false;
                quiesce().await;
                continue;
            }
            wake_due = false;
            quiesce().await;
            Timer::after_millis(1000).await;
            continue;
        }

        // --- negotiate: receive; caps -> 9 V Request at once ---
        console::PD_STAGE.store(4, Ordering::Relaxed);
        dbg_stage(13);
        let outcome = negotiate(&mut phy, t0_ms).await;
        match outcome {
            Negotiate::Contracted => {
                contracted_9v = true;
                pd_log(b"[pdmin] SUCCESS 9v contract\r\n");
                // No quiesce: the contract stays alive (RX on, auto-GoodCRC
                // keeps acknowledging; the probe just stops talking). No
                // dbg_stage here either: 20 is transmit()'s own mark —
                // parking the cookie on it would mislabel the next boot's
                // evidence. The last real mark (22 FIFO write / 13 engine)
                // is the accurate "where we ended".
                break;
            }
            Negotiate::HardReset => {
                // The charger is alive; its fresh announce window follows.
                wake_due = false;
                log_end(&phy, 7, t0_ms).await;
                quiesce().await;
                Timer::after_millis(1600).await; // tSrcRecover + margin
            }
            Negotiate::Silent(code) => {
                // Nothing at all arrived: wake it next attempt. `code` is
                // WHAT killed the wait (Budget=16 = the full ~16 s window;
                // anything else = an early death before any packet — the
                // distinction is evidence, not noise).
                wake_due = true;
                log_end(&phy, code, t0_ms).await;
                quiesce().await;
            }
            Negotiate::Failed(code) => {
                wake_due = false;
                log_end(&phy, code, t0_ms).await;
                quiesce().await;
                Timer::after_millis(500).await;
            }
        }
    }

    if !contracted_9v {
        pd_log(b"[pdmin] GAVE UP: no 9v contract\r\n");
        dbg_stage(14);
    }
    console::PD_STAGE.store(0, Ordering::Relaxed);
}

/// What one negotiate pass concluded. `Silent` carries the code that ended
/// the empty wait (Budget=16 = full window elapsed; else an early death).
enum Negotiate {
    Contracted,
    HardReset,
    Silent(u16),
    Failed(u16),
}

async fn negotiate(phy: &mut pd_phy::Session<'_, pd_phy::SharedBus, embassy_time::Delay, MinClock>, t0_ms: u32) -> Negotiate {
    let mut buf = [0u8; 40];
    let mut tx_id: u8 = 0;
    let mut defers: u32 = 0;
    loop {
        console::PD_STAGE.store(4, Ordering::Relaxed);
        buf.fill(0);
        match phy.receive(&mut buf).await {
            Ok(len) => {
                if len < 2 {
                    pd_log(b"[m] rx short\r\n");
                    continue;
                }
                let hdr_rx = u16::from_le_bytes([buf[0], buf[1]]);
                let noo = ((hdr_rx >> 12) & 7) as usize;
                let ty = (hdr_rx & 0x1f) as u8;
                let rev = ((hdr_rx >> 6) & 3) as u8;
                if noo == 0 {
                    // Control: 3 Accept, 4 Reject, 6 PS_RDY (Table 6.4).
                    let name: &[u8] = match ty {
                        3 => b"[m] ACCEPT\r\n",
                        4 => b"[m] REJECT\r\n",
                        6 => b"[m] PS_RDY\r\n",
                        _ => b"[m] ctrl\r\n",
                    };
                    pd_log(name);
                    if ty == 3 || ty == 6 {
                        log_vbus(ty).await;
                    }
                    if ty == 6 {
                        return Negotiate::Contracted;
                    }
                } else if ty == 1 {
                    // Source_Capabilities: decode, then the 9 V Request at
                    // once — no engine, no await between this and transmit.
                    // r168: the decode ms (relative to session start) —
                    // paired with the end line's e=/p= it decides whether
                    // the 77 ms gap sits BEFORE transmit (delivery) or
                    // inside the chain.
                    let mut d = [0u8; 32];
                    let mut n = 0;
                    for &b in b"[m] caps@" {
                        d[n] = b;
                        n += 1;
                    }
                    n = push_dec(
                        &mut d,
                        n,
                        (embassy_time::Instant::now().as_millis() as u32).wrapping_sub(t0_ms),
                    );
                    d[n] = b'\r';
                    d[n + 1] = b'\n';
                    pd_log(&d[..n + 2]);
                    log_pdos(phy);
                    match build_request(phy, hdr_rx, tx_id) {
                        Ok((req_hdr, rdo_raw, req)) => {
                            let mut d = [0u8; 64];
                            let mut n = 0;
                            for &b in b"[m] req hdr=" {
                                d[n] = b;
                                n += 1;
                            }
                            for shift in [12u16, 8, 4, 0] {
                                n = push_hex8(&mut d, n, u32::from((req_hdr >> shift) & 0xF));
                            }
                            for &b in b" rdo=" {
                                d[n] = b;
                                n += 1;
                            }
                            for shift in [24u32, 16, 8, 0] {
                                n = push_hex8(&mut d, n, (rdo_raw >> shift) & 0xFF);
                            }
                            for &b in b" id=" {
                                d[n] = b;
                                n += 1;
                            }
                            n = push_dec(&mut d, n, u32::from(tx_id & 7));
                            d[n] = b'\r';
                            d[n + 1] = b'\n';
                            pd_log(&d[..n + 2]);
                            match phy.transmit(&req).await {
                                Ok(()) => {
                                    pd_log(b"[m] tx OK\r\n");
                                    tx_id = (tx_id + 1) & 7;
                                    defers = 0;
                                }
                                Err(usbpd_traits::DriverTxError::Deferred) => {
                                    defers += 1;
                                    let mut d = [0u8; 32];
                                    let mut n = 0;
                                    for &b in b"[m] tx deferred " {
                                        d[n] = b;
                                        n += 1;
                                    }
                                    n = push_dec(&mut d, n, defers);
                                    d[n] = b'\r';
                                    d[n + 1] = b'\n';
                                    pd_log(&d[..n + 2]);
                                    if defers >= DEFER_RETRIES {
                                        return Negotiate::Failed(25);
                                    }
                                    Timer::after_millis(25).await;
                                }
                                Err(usbpd_traits::DriverTxError::HardReset) => {
                                    pd_log(b"[m] tx HR\r\n");
                                    return Negotiate::HardReset;
                                }
                                Err(usbpd_traits::DriverTxError::NotAcknowledged) => {
                                    // On the wire, never GoodCRC'd (section
                                    // 7.1.1); the session stays usable.
                                    pd_log(b"[m] tx NOT-ACKED\r\n");
                                    return Negotiate::Failed(21);
                                }
                                Err(usbpd_traits::DriverTxError::Discarded) => {
                                    // The session carries the specific error
                                    // (TxTimeout, a refusal, Bus, ...).
                                    let code = phy
                                        .take_error()
                                        .map_or(17, |e| error_code(&e));
                                    let mut d = [0u8; 32];
                                    let mut n = 0;
                                    for &b in b"[m] tx err=" {
                                        d[n] = b;
                                        n += 1;
                                    }
                                    n = push_dec(&mut d, n, u32::from(code));
                                    d[n] = b'\r';
                                    d[n + 1] = b'\n';
                                    pd_log(&d[..n + 2]);
                                    return Negotiate::Failed(code);
                                }
                            }
                        }
                        Err(()) => {
                            // No 9 V object (or no vSafe5V first): logged,
                            // not silent. The offer lines above show why.
                            pd_log(b"[m] no 9v in offer\r\n");
                        }
                    }
                } else {
                    let mut d = [0u8; 40];
                    let mut n = 0;
                    for &b in b"[m] rx h=" {
                        d[n] = b;
                        n += 1;
                    }
                    for shift in [12u16, 8, 4, 0] {
                        n = push_hex8(&mut d, n, u32::from((hdr_rx >> shift) & 0xF));
                    }
                    for &b in b" rev=" {
                        d[n] = b;
                        n += 1;
                    }
                    n = push_dec(&mut d, n, u32::from(rev));
                    for &b in b" n=" {
                        d[n] = b;
                        n += 1;
                    }
                    n = push_dec(&mut d, n, len as u32);
                    d[n] = b'\r';
                    d[n + 1] = b'\n';
                    pd_log(&d[..n + 2]);
                }
            }
            Err(DriverRxError::HardReset) => {
                pd_log(b"[m] rx HARDRESET\r\n");
                return Negotiate::HardReset;
            }
            Err(DriverRxError::Discarded) => {
                // Budget (a fully silent source), Boundary, RxFull, Frame,
                // Peer... take_error names it; rx_packets==0 marks silence.
                let code = phy.take_error().map_or(0, |e| error_code(&e));
                if phy.observation.rx_packets == 0 {
                    return Negotiate::Silent(code);
                }
                return Negotiate::Failed(code);
            }
        }
    }
}

/// Build the 9 V Request with the vendored crate's own review path:
/// `request_fixed::<FnbLimits>(caps, TARGET_MV)` — the exact builder the
/// engine's DPM uses and the host repro validated, so the gauntlet's
/// `request_is_reviewed` passes it by construction. Returns
/// (header, raw rdo, wire bytes) or Err when 9 V is not offered.
fn build_request(
    phy: &pd_phy::Session<'_, pd_phy::SharedBus, embassy_time::Delay, MinClock>,
    caps_header: u16,
    tx_id: u8,
) -> Result<(u16, u32, [u8; 6]), ()> {
    let Some(caps) = phy.observation.capabilities else {
        return Err(());
    };
    // Rebuild the crate's SourceCapabilities from the raw PDOs.
    let mut pdos = heapless::Vec::new();
    for object in caps.objects.iter().take(usize::from(caps.count)) {
        let _ = pdos.push(source_capabilities::parse_raw_pdo(object.raw));
    }
    let caps = source_capabilities::SourceCapabilities::new_with_pdos(pdos);
    let rdo = match pd_protocol::pd_spr::request_fixed::<FnbLimits>(
        &caps,
        <FnbLimits as pd_spr::Limits>::TARGET_MV,
    ) {
        Ok(PowerSource::FixedVariableSupply(rdo)) => rdo.0,
        _ => return Err(()), // no 9 V object / no vSafe5V first / unsafe
    };
    // Header: one Data object (Request), the offer's revision, SINK/UFP
    // roles clear, our MessageID (fresh session: the counter starts at 0,
    // exactly the pinned protocol layer's policy).
    let hdr: u16 = header::objects(1)
        | (caps_header & header::SPEC_REVISION)
        | header::data::REQUEST
        | header::message_id(u16::from(tx_id & 7));
    let mut req = [0u8; 6];
    req[..2].copy_from_slice(&hdr.to_le_bytes());
    req[2..].copy_from_slice(&rdo.to_le_bytes());
    Ok((hdr, rdo, req))
}

/// The CC diagnostic: restore the power-on baseline, snapshot, one-shot
/// measure + listen (pd_task's exact flow, logged instead of VIEW'd).
async fn run_diagnostic() -> Option<Report> {
    let mut guard = pd_task::BUS.lock().await;
    let bus = guard.0.as_mut()?;
    if pd_task::restore_baseline(bus).await.is_err() {
        pd_log(b"[pdmin] baseline restore FAIL\r\n");
        return None;
    }
    let snapshot = match fusb302_pd::snapshot::snapshot(bus).await {
        Ok(snapshot) => snapshot,
        Err(failure) => {
            let mut d = [0u8; 48];
            let mut n = 0;
            for &b in b"[pdmin] snapshot err reg=" {
                d[n] = b;
                n += 1;
            }
            n = push_dec(&mut d, n, u32::from(failure.register));
            d[n] = b'\r';
            d[n + 1] = b'\n';
            pd_log(&d[..n + 2]);
            return None;
        }
    };
    let mut delay = embassy_time::Delay;
    match fusb302_pd::startup::Diagnostic::once(
        <pd_phy::FnbPlatform as fusb302_pd::phy::Platform>::BASELINE,
        pd_phy::MEASURE,
    )
    .run(bus, &mut delay, snapshot)
    .await
    {
        Ok(report) => {
            const HEX: &[u8; 16] = b"0123456789abcdef";
            let mut d = [0u8; 40];
            let mut n = 0;
            for &b in b"[pdmin] diag cc=" {
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
            d[n] = b'0' + u8::from(report.listen.is_some_and(|l| l.traffic.is_some()));
            n += 1;
            d[n] = b'\r';
            d[n + 1] = b'\n';
            pd_log(&d[..n + 2]);
            Some(report)
        }
        Err(failure) => {
            // Baseline/verify failures report the offending register.
            let code = match &failure.error {
                fusb302_pd::config::Error::Baseline { register, .. }
                | fusb302_pd::config::Error::Verify { register, .. } => {
                    u16::from(*register) | 0x100
                }
                fusb302_pd::config::Error::Bus { register, .. } => u16::from(*register) | 0x200,
            };
            let mut d = [0u8; 40];
            let mut n = 0;
            for &b in b"[pdmin] diag err=" {
                d[n] = b;
                n += 1;
            }
            n = push_dec(&mut d, n, u32::from(code));
            d[n] = b'\r';
            d[n + 1] = b'\n';
            pd_log(&d[..n + 2]);
            None
        }
    }
}

/// start err= line (Session::start's own verdict).
fn log_start(error: Option<&pd_phy::Error<embassy_at32::i2c::Error>>) {
    let code = error.map_or(0, error_code);
    let mut d = [0u8; 32];
    let mut n = 0;
    for &b in b"[pdmin] start err=" {
        d[n] = b;
        n += 1;
    }
    n = push_dec(&mut d, n, u32::from(code));
    d[n] = b'\r';
    d[n + 1] = b'\n';
    pd_log(&d[..n + 2]);
}

/// The offer, parsed: pN=<raw> <mV>mV<mA>mA for every fixed object.
fn log_pdos(phy: &pd_phy::Session<'_, pd_phy::SharedBus, embassy_time::Delay, MinClock>) {
    let Some(caps) = phy.observation.capabilities else {
        pd_log(b"[m] no caps decoded\r\n");
        return;
    };
    let mut d = [0u8; 24];
    let mut n = 0;
    for &b in b"[m] n=" {
        d[n] = b;
        n += 1;
    }
    n = push_dec(&mut d, n, u32::from(caps.count));
    d[n] = b'\r';
    d[n + 1] = b'\n';
    pd_log(&d[..n + 2]);
    for (i, object) in caps.objects.iter().take(6).enumerate() {
        if i >= usize::from(caps.count) {
            break;
        }
        let mut d = [0u8; 64];
        let mut n = 0;
        for &b in b"[m] p" {
            d[n] = b;
            n += 1;
        }
        d[n] = b'1' + i as u8;
        n += 1;
        d[n] = b'=';
        n += 1;
        for shift in [24u32, 16, 8, 0] {
            n = push_hex8(&mut d, n, (object.raw >> shift) & 0xFF);
        }
        if let Some((mv, ma)) = object.fixed {
            for &b in b" " {
                d[n] = b;
                n += 1;
            }
            n = push_dec(&mut d, n, u32::from(mv));
            for &b in b"mV" {
                d[n] = b;
                n += 1;
            }
            n = push_dec(&mut d, n, u32::from(ma));
            for &b in b"mA" {
                d[n] = b;
                n += 1;
            }
        }
        d[n] = b'\r';
        d[n + 1] = b'\n';
        pd_log(&d[..n + 2]);
    }
}

/// VBUS through the Platform's qualified source, once per Accept/PS_RDY.
async fn log_vbus(ty: u8) {
    match <pd_phy::FnbPlatform as fusb302_pd::phy::Platform>::vbus_raw(
        &mut pd_phy::SharedBus,
    )
    .await
    {
        Ok(raw) => {
            let mv = <pd_phy::FnbPlatform as fusb302_pd::phy::Platform>::voltage_mv(raw);
            let mut d = [0u8; 40];
            let mut n = 0;
            for &b in b"[m] vbus=" {
                d[n] = b;
                n += 1;
            }
            n = push_dec(&mut d, n, mv);
            for &b in b"mV t=" {
                d[n] = b;
                n += 1;
            }
            n = push_dec(&mut d, n, u32::from(ty));
            if mv >= 8100 && mv <= 9900 {
                for &b in b" (9v)\r\n" {
                    d[n] = b;
                    n += 1;
                }
            } else {
                for &b in b" (NOT 9v)\r\n" {
                    d[n] = b;
                    n += 1;
                }
            }
            pd_log(&d[..n]);
        }
        Err(error) => {
            let code = error_code(&error);
            let mut d = [0u8; 32];
            let mut n = 0;
            for &b in b"[m] vbus read err=" {
                d[n] = b;
                n += 1;
            }
            n = push_dec(&mut d, n, u32::from(code));
            d[n] = b'\r';
            d[n + 1] = b'\n';
            pd_log(&d[..n + 2]);
        }
    }
}

/// End-of-attempt summary: the transmit evidence (w/g/c/x + timeout
/// registers when present), the receive evidence (n/m) and the wall time.
async fn log_end(
    phy: &pd_phy::Session<'_, pd_phy::SharedBus, embassy_time::Delay, MinClock>,
    code: u16,
    t0_ms: u32,
) {
    let mut d = [0u8; 96];
    let mut n = 0;
    for &b in b"[m] end code=" {
        d[n] = b;
        n += 1;
    }
    n = push_dec(&mut d, n, u32::from(code));
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
        if let Some(timeout) = spr.tx_timeout {
            for &b in b" t=1 s1=" {
                d[n] = b;
                n += 1;
            }
            n = push_hex8(&mut d, n, u32::from(timeout.status1));
            for &b in b" ia=" {
                d[n] = b;
                n += 1;
            }
            n = push_hex8(&mut d, n, u32::from(timeout.flags));
        }
    }
    for &b in b" x=" {
        d[n] = b;
        n += 1;
    }
    d[n] = b'0' + u32::from(fusb302_pd::phy::LAST_TX_REFUSAL.load(Ordering::Relaxed)) as u8;
    n += 1;
    // r168: the transmit-chain timestamps (r161's fields, finally
    // printable). e = transmit() entry, p = the pre-TX poll — both as ms
    // after session start; '-' = the step never ran. With the caps@ line
    // this splits the 77 ms gap: e large = delivery was late (receive
    // side); e small + p large = inside the chain (impossible per the
    // sync-read gauntlet); e,p small + HR = the latch predates the poll.
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
            n = push_dec(&mut d, n, at.wrapping_sub(t0_ms));
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
            n = push_dec(&mut d, n, at.wrapping_sub(t0_ms));
        }
    }
    for &b in b" n=" {
        d[n] = b;
        n += 1;
    }
    n = push_dec(&mut d, n, u32::from(phy.observation.rx_packets));
    for &b in b" m=" {
        d[n] = b;
        n += 1;
    }
    n = push_dec(&mut d, n, u32::from(phy.observation.boot_step_ms));
    for &b in b" d=" {
        d[n] = b;
        n += 1;
    }
    let now = embassy_time::Instant::now().as_millis() as u32;
    n = push_dec(&mut d, n, now.wrapping_sub(t0_ms));
    d[n] = b'\r';
    d[n + 1] = b'\n';
    pd_log(&d[..n + 2]);
    // The TX-chain cookie marks (20 transmit entered / 21 pre-TX poll /
    // 22 FIFO write next) survive in the backup domain even across the
    // reset this probe must never cause; print the last one seen.
    let mut d = [0u8; 32];
    let mut n = 0;
    for &b in b"[m] cookie stage=" {
        d[n] = b;
        n += 1;
    }
    n = push_dec(&mut d, n, u32::from(console::dbg_last_stage()));
    d[n] = b'\r';
    d[n + 1] = b'\n';
    pd_log(&d[..n + 2]);
}

/// Blind power-on baseline restore between attempts (r139 rationale: a
/// dead attempt must not leave the FUSB302 acking caps). Failures log.
async fn quiesce() {
    let mut guard = pd_task::BUS.lock().await;
    if let Some(bus) = guard.0.as_mut() {
        let ok = pd_task::restore_baseline(bus).await.is_ok();
        pd_log(if ok {
            b"[pdmin] baseline restored\r\n"
        } else {
            b"[pdmin] baseline restore FAIL\r\n"
        });
    }
}

/// pd_task's error-code table (same numbers the main build prints).
fn error_code(error: &pd_phy::Error<embassy_at32::i2c::Error>) -> u16 {
    pd_task::error_code(error)
}

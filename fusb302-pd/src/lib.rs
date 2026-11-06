//! FUSB302 transport, checked startup and receive-only adapters around the
//! controller-neutral `pd-protocol` library. Bus/time ownership, physical
//! observations, limits and board policy are supplied by callers; no MCU or
//! Embassy runtime, UI, sensor model or bootloader configuration is assumed.
#![no_std]

pub mod config;
pub mod header;
pub mod observation;
pub mod pd_rx;
pub mod pd_sniff;
pub mod phy;
pub mod register;
pub mod service;
pub mod snapshot;
pub mod sniffer;
pub mod startup;

/// r158 host repro: the r99-r157 wall, offline. A scripted Driver feeds the
/// EXACT capabilities the FNB58 hardware decodes (5V/3A, 9V/2.44A, 12V/1.83A,
/// 15V/1.46A, 20V/1.1A, PPS APDO - PD3 Source, 6 objects) into
/// `Trial::run_selectable` and records every transmit. On hardware the
/// engine never sends the Request (w=0, x=0) and the charger Hard-Resets
/// ~130 ms after the caps were auto-acknowledged; this test answers WHY
/// without hardware: if no Request lands in the TX log, the wall is inside
/// the vendored engine/wrapper and the failing predicate is host-debuggable.
#[cfg(test)]
mod engine_repro {
    use core::future::Future;
    use core::task::{Context, Poll};

    extern crate std;
    use std::{vec, vec::Vec};

    use pd_protocol::pd_spr::{Limits, MaintainedTrace, Selection, Trial};
    use usbpd_traits::{Driver, DriverRxError, DriverTxError};

    // The firmware's FnbLimits, verbatim (firmware/src/pd_phy.rs).
    struct FnbLimits;
    impl Limits for FnbLimits {
        const SINK_IDENTITY: pd_protocol::pd_spr::SinkIdentity =
            pd_protocol::pd_spr::SinkIdentity::UNASSIGNED;
        const SESSION_POLICY: pd_protocol::pd_spr::SessionPolicy =
            pd_protocol::pd_spr::SessionPolicy {
                pps_status_settle_ms: 300,
                query_settle_ms: 500,
                status_min_interval_ms: 200,
                retry_delay_ms: 100,
                pps_expiry_grace_ms: 5_000,
                pps_status_max_deferrals: 3,
                query_max_deferrals: 3,
            };
        const MAX_REQUEST_MV: u32 = 20_000;
        const MIN_PPS_MV: u32 = 3_300;
        const MAX_PPS_MV: u32 = 21_000;
        const TARGET_MV: u32 = 5_000;
        const REQUEST_MA: u32 = 100;
        const MAX_CURRENT_MA: u32 = 12_000;
        const USB_COMMUNICATIONS_CAPABLE: bool = false;
        const NO_USB_SUSPEND: bool = true;
        const HIGHER_CAPABILITY: bool = false;
        const UNCONSTRAINED_POWER: bool = false;
        const SINK_POWER_MODES: u8 = 0b0000_0010;
        const FIXED_TARGETS: &'static [u16] = &[5_000, 9_000, 12_000, 15_000, 20_000];
        const PPS_REFRESH_MS: u64 = 4_000;
    }

    /// Host timer: deadline-based Pending futures. embassy's select polls
    /// the TIMEOUT future first, so an instantly-ready timer would starve
    /// every receive (the caps would never be delivered); wall-clock
    /// deadlines reproduce the hardware race faithfully under the busy
    /// poll loop below.
    struct HostTimer;
    impl HostTimer {
        fn wait(ms: u64) -> impl Future<Output = ()> {
            let deadline = std::time::Instant::now() + std::time::Duration::from_millis(ms);
            core::future::poll_fn(move |_| {
                if std::time::Instant::now() >= deadline {
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            })
        }
    }
    impl pd_protocol::pd_spr::Timer for HostTimer {
        async fn after_millis(ms: u64) {
            Self::wait(ms).await;
        }
        async fn after_timer(_timer: pd_protocol::pd_spr::TimerType, ms: u64) {
            Self::wait(ms).await;
        }
    }

    /// Scripted source: delivers the observed caps frame once, then parks
    /// (a quiet line). Every transmit is recorded.
    struct ScriptedDriver {
        frames: Vec<Vec<u8>>,
        tx_log: Vec<Vec<u8>>,
        hr_log: u32,
    }
    impl ScriptedDriver {
        fn new(caps: &[u8]) -> Self {
            Self {
                frames: vec![caps.to_vec()],
                tx_log: Vec::new(),
                hr_log: 0,
            }
        }
    }
    impl Driver for ScriptedDriver {
        const HAS_AUTO_GOOD_CRC: bool = true;
        const HAS_AUTO_RETRY: bool = true;

        fn wait_for_vbus(&mut self) -> impl Future<Output = ()> {
            async {}
        }
        fn receive(&mut self, buffer: &mut [u8]) -> impl Future<Output = Result<usize, DriverRxError>> {
            async move {
                if self.frames.is_empty() {
                    // A quiet line: no packet until the end of time. The
                    // engine's timer selects fire and drive it forward.
                    core::future::pending::<()>().await;
                }
                let frame = self.frames.remove(0);
                buffer[..frame.len()].copy_from_slice(&frame);
                Ok(frame.len())
            }
        }
        fn transmit(&mut self, data: &[u8]) -> impl Future<Output = Result<(), DriverTxError>> {
            async move {
                self.tx_log.push(data.to_vec());
                Ok(())
            }
        }
        fn transmit_hard_reset(&mut self) -> impl Future<Output = Result<(), DriverTxError>> {
            async move {
                self.hr_log += 1;
                Ok(())
            }
        }
    }

    /// Source_Capabilities, PD 3.0, 6 objects, MessageID 0, Source/DFP:
    /// wire type 1 (usbpd's DataMessageType::SourceCapabilities = 1), then
    /// the six observed PDOs in standard wire encoding.
    fn observed_caps() -> Vec<u8> {
        let header: u16 = 6 << 12 | 0 << 9 | 1 << 8 | 2 << 6 | 0 << 5 | 1;
        // Fixed PDO layout: voltage (50 mV) @ 19:10, current (10 mA) @ 9:0.
        let pdos: [u32; 6] = [
            (100 << 10) | 300,  // 5V / 3.00A fixed
            (180 << 10) | 244,  // 9V / 2.44A fixed
            (240 << 10) | 183,  // 12V / 1.83A fixed
            (300 << 10) | 146,  // 15V / 1.46A fixed
            (400 << 10) | 110,  // 20V / 1.10A fixed
            0xC0DC_3230,        // PPS APDO 5-11V / 2.40A (the observed p6)
        ];
        let mut frame = Vec::new();
        frame.extend_from_slice(&header.to_le_bytes());
        for pdo in pdos {
            frame.extend_from_slice(&pdo.to_le_bytes());
        }
        frame
    }

    fn spin_bounded<F: Future>(future: F) -> F::Output {
        let mut future = core::pin::pin!(future);
        let mut cx = Context::from_waker(core::task::Waker::noop());
        let mut polls: u64 = 0;
        loop {
            polls += 1;
            assert!(polls < 500_000_000, "engine future never settled");
            if let Poll::Ready(output) = future.as_mut().poll(&mut cx) {
                return output;
            }
            // Yield the core so wall-clock deadlines can elapse.
            if polls % 20_000 == 0 {
                std::thread::sleep(core::time::Duration::from_micros(200));
            }
        }
    }

    /// THE repro: caps arrive, then the line goes quiet. The engine MUST
    /// transmit the 5V Request (6 bytes) inside its own state machine.
    #[test]
    fn engine_sends_request_after_observed_caps() {
        let mut driver = ScriptedDriver::new(&observed_caps());
        let selection = Selection::<FnbLimits>::new();
        let trace = core::cell::RefCell::new(MaintainedTrace::default());
        let mut trial = Trial::maintained();
        let outcome = spin_bounded(trial.run_selectable::<_, HostTimer>(
            &mut driver,
            &trace,
            &selection,
        ));
        let requests: Vec<_> = driver
            .tx_log
            .iter()
            .enumerate()
            .map(|(i, tx)| (i, tx.len()))
            .collect();
        let rdo: Option<u32> = driver.tx_log.first().and_then(|tx| {
            if tx.len() == 6 {
                Some(u32::from_le_bytes([tx[2], tx[3], tx[4], tx[5]]))
            } else {
                None
            }
        });
        use pd_protocol::pd_spr::{
            DEBUG_LAST_FAIL, DEBUG_REQ_GATE, DEBUG_REQ_MV, DEBUG_REQ_OK, DEBUG_STAGE, DEBUG_T_BYTES, DEBUG_T_CAPS,
            DEBUG_T_MODE, DEBUG_T_SENT, DEBUG_T_SELECTED,
        };
        use core::sync::atomic::Ordering;
        std::println!(
            "outcome={:?} tx={:?} rdo={:x?} hr={} stage={} t_bytes={} t_mode={} t_sent={} t_sel={} t_caps={} req_mv={} req_ok={} last_fail={} req_gate={}",
            outcome, requests, rdo, driver.hr_log,
            DEBUG_STAGE.load(Ordering::Relaxed),
            DEBUG_T_BYTES.load(Ordering::Relaxed),
            DEBUG_T_MODE.load(Ordering::Relaxed),
            DEBUG_T_SENT.load(Ordering::Relaxed),
            DEBUG_T_SELECTED.load(Ordering::Relaxed),
            DEBUG_T_CAPS.load(Ordering::Relaxed),
            DEBUG_REQ_MV.load(Ordering::Relaxed),
            DEBUG_REQ_OK.load(Ordering::Relaxed),
            DEBUG_LAST_FAIL.load(Ordering::Relaxed),
            DEBUG_REQ_GATE.load(Ordering::Relaxed),
        );
        assert_eq!(
            driver.tx_log.first().map(|tx| tx.len()),
            Some(6),
            "the engine must transmit the 6-byte Request after caps; tx_log={:?} outcome={:?}",
            driver.tx_log.iter().map(|tx| tx.len()).collect::<Vec<_>>(),
            outcome
        );
        // Object position 1, operating = maximum = 300 (3000 mA in 10 mA units).
        assert_eq!(rdo.map(|r| r >> 28), Some(1));
    }
}

//! FNB58 board-side implementation of the [`fusb302_pd`] interfaces: the
//! [`Platform`] policy (power-on baseline, MDAC, VBUS through the TPA626),
//! the sink [`Limits`] policy, and the [`GatedBus`] over the shared I2C1
//! bus. All wire authorization stays in the vendored crate; this file only
//! supplies board facts.
//!
//! The power-on baseline is the FUSB302B's reset configuration (DS Table 16,
//! same silicon and revision as kws-x1's, DEVICE_ID 0x91 verified in r20):
//! our firmware leaves the chip untouched until a scan starts.

use core::sync::atomic::{AtomicBool, Ordering};
use embedded_hal_async::i2c::{ErrorType, I2c, Operation};

use crate::pd_task::BUS;
pub use fusb302_pd::phy::{Clock, Config, Error, GatedBus, Waiter};

/// ID Header VDO (USB PD Table 6.33): Passive Cable (bits 29:27 = 011b),
/// USB Type-C plug (bits 22:21 = 11b), unassigned VID FFFFh (section 6.1.5).
const EMARKER_ID_HEADER: u32 = 0x1860_ffff;
/// Cert Stat VDO (Table 6.38): no XID.
const EMARKER_CERT_STAT: u32 = 0;
/// Product VDO (Table 6.39): PID 0, bcdDevice 0.
const EMARKER_PRODUCT: u32 = 0;
/// Passive Cable VDO (Table 6.41): USB Type-C to Type-C (bits 19:18 = 10b),
/// under 10 ns latency (bits 16:13 = 0001b), 20 V maximum (bits 10:9 = 00b),
/// 5 A (bits 6:5 = 10b), USB 2.0 only (bits 2:0 = 000b); not EPR capable.
const EMARKER_CABLE: u32 = 0x0008_2040;

/// MDAC 0x31 = a 2.100 V CC comparator with MEAS_VBUS clear (kws-x1's value;
/// the sink start does not depend on it, the inspection gates do).
pub const MEASURE: u8 = 0x31;

pub struct FnbPlatform;
static RX_IDLE: AtomicBool = AtomicBool::new(false);
impl fusb302_pd::phy::Platform for FnbPlatform {
    const CABLE_RESPONSE_TIMEOUT_MS: u64 = 51;
    const CABLE_SVID_NO_PROGRESS_TIMEOUT_MS: u64 = 1_000;
    type Limits = FnbLimits;
    /// FUSB302B power-on configuration (DS Table 16 reset values; DEVICE_ID
    /// verified 0x91 on this board in r20).
    const BASELINE: [u8; 11] = [
        0x91, // DEVICE_ID: Version B, default product, revision B (DS Table 17)
        0x03, // SWITCHES0: PDWN1 | PDWN2, Rd on both CCs
        0x20, // SWITCHES1: SPECREV 2.0, transmit drivers off
        0x24, // CONTROL0: INT_MASK, HOST_CUR 80 uA
        0x00, // CONTROL1: SOP'/SOP'' reception off
        0x02, // CONTROL2: MODE DRP, toggle off
        0x06, // CONTROL3: N_RETRIES 3, AUTO_RETRY off
        0x01, // POWER: bandgap and wake only
        0x00, // MASK
        0x00, // MASKA
        0x00, // MASKB
    ];
    const MEASURE: u8 = MEASURE;
    const INSPECTION_CC: u8 = 1;
    // VCONN on CC2 while the measure block watches CC1 (DS Table 18).
    const INSPECTION_SWITCHES0: u8 =
        fusb302_pd::register::switches0::VCONN_CC2 | fusb302_pd::register::switches0::MEAS_CC1;
    const ALLOW_STALE_SINK_TX_NG: bool = true;
    // The tester's plug sits in the Source's receptacle, so it answers SOP'
    // Discover Identity as a 5 A passive cable so the charger offers its 5 A
    // objects (kws-x1's owner-approved convention).
    const EMARKER: Option<[u32; 4]> = Some([
        EMARKER_ID_HEADER,
        EMARKER_CERT_STAT,
        EMARKER_PRODUCT,
        EMARKER_CABLE,
    ]);
    // VBUS through the TPA626 (INA226 protocol) at 0x40, bus voltage
    // register 0x02, 1.25 mV/LSB.
    const VBUS_ADDRESS: u8 = 0x40;
    const VBUS_REGISTER: u8 = 2;
    fn voltage_mv(raw: u16) -> u32 {
        u32::from(raw) * 5 / 4
    }
    fn set_rx_idle(idle: bool) {
        RX_IDLE.store(idle, Ordering::Relaxed);
    }
    async fn vbus_raw<B: I2c>(bus: &mut B) -> Result<u16, Error<B::Error>> {
        let mut raw = [0; 2];
        bus.write_read(Self::VBUS_ADDRESS, &[Self::VBUS_REGISTER], &mut raw)
            .await
            .map_err(|source| Error::Bus {
                address: Self::VBUS_ADDRESS,
                register: Self::VBUS_REGISTER,
                source,
            })?;
        Ok(u16::from_be_bytes(raw))
    }
}

/// The FNB58's sink policy: a meter/tester powered from VBUS only, USB data
/// not wired to the charger, no battery. The engine requests the target
/// object's Maximum Current up to MAX_CURRENT_MA.
pub struct FnbLimits;
impl pd_protocol::pd_spr::Limits for FnbLimits {
    // No VID, no metadata (USB PD section 6.1.5, 7.19.4.1).
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
    // 3.3 V is the lowest SPR PPS voltage the library accepts.
    const MIN_PPS_MV: u32 = 3_300;
    const MAX_PPS_MV: u32 = 21_000;
    const TARGET_MV: u32 = 5_000;
    // Smallest Fixed PDO Maximum Current this sink accepts (100 mA class).
    const REQUEST_MA: u32 = 100;
    // Fixed Requests ask for the object's Maximum Current up to this value.
    const MAX_CURRENT_MA: u32 = 12_000;
    // The charger-facing port has no USB data path to the charger.
    const USB_COMMUNICATIONS_CAPABLE: bool = false;
    const NO_USB_SUSPEND: bool = true;
    // The tester works at vSafe5V; no higher-capability claim yet.
    const HIGHER_CAPABILITY: bool = false;
    const UNCONSTRAINED_POWER: bool = false;
    // Powered from VBUS only: no battery, no own supply (Table 6.61 bit 1).
    const SINK_POWER_MODES: u8 = 0b0000_0010;
    const FIXED_TARGETS: &'static [u16] = &[5_000, 9_000, 12_000, 15_000, 20_000];
    const PPS_REFRESH_MS: u64 = 4_000;
}

pub type Session<'a, B, D, C> = fusb302_pd::phy::Session<'a, B, D, C, FnbPlatform>;

/// The shared I2C1 bus: one Mutex lease per transaction (the meter gets
/// turns between PD transfers), each transfer spun to completion inside
/// its lease.
pub struct SharedBus;

impl ErrorType for SharedBus {
    type Error = embassy_at32::i2c::Error;
}

/// Run `future` to completion here, never returning to the executor. The
/// HAL waits re-read the I2C status on every poll and the time driver
/// advances from the SysTick exception, so only interrupt handlers run
/// until the transfer completes — the SenderResponseTimer survives a
/// 31-byte FIFO read (kws-x1's measured 14 ms executor-turn cost).
pub(crate) fn spin<F: core::future::Future>(future: F) -> F::Output {
    let mut future = core::pin::pin!(future);
    let mut cx = core::task::Context::from_waker(core::task::Waker::noop());
    loop {
        if let core::task::Poll::Ready(output) = future.as_mut().poll(&mut cx) {
            return output;
        }
    }
}

impl I2c for SharedBus {
    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        let mut guard = BUS.lock().await;
        let bus = guard.0.as_mut().expect("I2C1 not initialized");
        spin(bus.transaction(address, operations))
    }
}

/// SinkTxOK gate (USB PD section 7.2): one Mutex lease and no executor turn
/// from the STATUS0 sample to the TXON byte.
impl GatedBus for SharedBus {
    async fn gated_write(
        &mut self,
        address: u8,
        registers: [u8; 2],
        allow: impl FnOnce(u8, u8) -> bool,
        bytes: &[u8],
    ) -> Result<([u8; 2], bool), Self::Error> {
        let mut guard = BUS.lock().await;
        let bus = guard.0.as_mut().expect("I2C1 not initialized");
        spin(async {
            let mut samples = [0; 2];
            for (sample, register) in samples.iter_mut().zip(registers) {
                bus.transaction(
                    address,
                    &mut [
                        Operation::Write(&[register]),
                        Operation::Read(core::slice::from_mut(sample)),
                    ],
                )
                .await?;
            }
            if !allow(samples[0], samples[1]) {
                return Ok((samples, false));
            }
            bus.transaction(address, &mut [Operation::Write(bytes)])
                .await?;
            Ok((samples, true))
        })
    }
}

pub async fn wait_for_rx_idle(delay: &mut impl Waiter) {
    while !RX_IDLE.load(Ordering::Relaxed) {
        delay.wait_event().await;
    }
}

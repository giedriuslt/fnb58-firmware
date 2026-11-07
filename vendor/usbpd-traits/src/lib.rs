//! USB PD library traits.
//!
//! Provides a driver trait that allows to add support for various USB PD PHYs.
#![cfg_attr(not(test), no_std)]
#![warn(missing_docs)]
use core::future::Future;

/// SenderResponseTimer duration in milliseconds (USB PD R3.2 V1.2,
/// section 7.31.2 and Table 7.9: 24-30 ms).
///
/// r174 DEVIATION, flagged: the spec value here is 24-30 ms and a
/// compliant sink Shall Hard Reset when it expires — but the FNB58's
/// test chargers Accept a RE-Request (a contract change from Ready)
/// slower than that: the r173 log shows our Request GoodCRC'd, the
/// charger's Accept on the wire (the user's packet tracer), and our
/// Hard Reset ~30 ms after the GoodCRC (code=44 HardResetSent) — the
/// timer won the race against a slow partner. 100 ms tolerates them
/// while staying a bounded wait; a compliant source answers inside 30 ms
/// and never notices the difference. kws-x1 ships the spec value.
pub const SENDER_RESPONSE_MS: u64 = 100;
/// Selected tVDMSenderResponse in milliseconds (USB PD R3.2 V1.2 Table 7.9:
/// 24–50 ms): the maximum, for partners that answer VDMs from software.
pub const VDM_RESPONSE_MS: u64 = 50;

/// Receive Error.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DriverRxError {
    /// Received message discarded, e.g. due to CRC errors.
    Discarded,

    /// Hard Reset received before or during reception.
    HardReset,
}

/// Transmit Error.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DriverTxError {
    /// Concurrent receive in progress or excessive noise on the line.
    Discarded,

    /// Hard Reset received before or during transmission.
    HardReset,

    /// The first Message of a Sink-initiated AMS was not sent: Rp was not
    /// SinkTxOK, or a SOP Message arrived first and the pending Message was
    /// Discarded (USB PD R3.2 V1.2 sections 7.2, 7.3). Nothing went on the wire.
    Deferred,

    /// The Message went on the wire but no GoodCRC arrived after nRetryCount
    /// retries (section 6.6.9.1). A Protocol Error leading to Soft Reset
    /// (section 7.1.1), unlike `Discarded`, which may be a local fault.
    NotAcknowledged,
}

/// Bounded cable discovery outcome; no automatic retries are requested.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CableStatus {
    /// Identity ACK received.
    Identity,
    /// Cable rejected Discover Identity.
    Nak,
    /// Cable reported busy; the caller does not retry.
    Busy,
    /// Cable did not respond before the deadline.
    Timeout,
    /// Inspection ended early without sending: Rp was not SinkTxOK, or a SOP
    /// Message from the port partner took priority (sections 7.2, 7.3).
    Deferred,
}

/// Result of an exclusive, explicit-contract cable discovery handoff.
/// Normal completion restores the driver's pre-handoff controller configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CableReport {
    /// Outcome of the bounded operation.
    pub status: CableStatus,
    /// Raw identity objects including VDM header, when a response was received.
    pub objects: [u32; 7],
    /// Number of valid objects.
    pub count: u8,
    /// Interpreted cable revision code (1=PD2, 2=PD3), not the partner revision.
    /// Received code 0 is normalized to 1 per PD R3.2 V1.2 Table 6.2.
    pub revision: u8,
    /// Next SOP transmit ID after successful partner exchanges.
    pub next_tx_id: u8,
    /// Last accepted SOP receive ID; cable IDs must never be stored here.
    pub last_rx_id: Option<u8>,
}

/// Driver trait, through which the protocol layer talks to the PHY.
pub trait Driver {
    /// If this is `true`, the protocol layer will not send its own
    /// GoodCRC messages and will instead rely on the hardware.
    const HAS_AUTO_GOOD_CRC: bool = false;

    /// If this is `true`, the hardware automatically retries transmission
    /// when no GoodCRC is received. The protocol layer will skip its own
    /// retry loop and call driver.transmit() directly. On success, it
    /// maintains protocol state (TX message counter) without calling
    /// wait_for_good_crc(), since the hardware already verified GoodCRC.
    const HAS_AUTO_RETRY: bool = false;

    /// Wait for availability of VBus voltage.
    fn wait_for_vbus(&mut self) -> impl Future<Output = ()>;

    /// Receive a packet.
    fn receive(&mut self, buffer: &mut [u8]) -> impl Future<Output = Result<usize, DriverRxError>>;

    /// Transmit a packet.
    fn transmit(&mut self, data: &[u8]) -> impl Future<Output = Result<(), DriverTxError>>;

    /// Milliseconds since the matching GoodCRC of the last successful transmit.
    /// Auto-retry drivers that finish confirmation after receiving GoodCRC must
    /// report that elapsed time so SenderResponseTimer is not restarted on return
    /// (USB PD R3.2 V1.2 section 7.31.2). Other drivers default to zero.
    fn sender_response_elapsed_ms(&self) -> u64 {
        0
    }

    /// Exclusive bounded cable discovery extension. The caller hands over SOP
    /// message IDs only while Ready and owns no concurrent receive future.
    /// Implementations must return updated SOP IDs, keep SOP' IDs separate,
    /// and restore pre-handoff controller configuration before success.
    /// Errors must not resume normal
    /// traffic or trigger an automatic discovery retry. Unsupported by default.
    fn discover_cable(
        &mut self,
        _next_tx_id: u8,
        _last_rx_id: Option<u8>,
    ) -> impl Future<Output = Result<CableReport, DriverRxError>> {
        async { Err(DriverRxError::Discarded) }
    }

    /// Transmit a hard reset signal.
    fn transmit_hard_reset(&mut self) -> impl Future<Output = Result<(), DriverTxError>>;

    /// BIST Carrier Mode (USB PD R3.2 V1.2 section 5.4.1): send the BMC carrier,
    /// stop it within tBISTContMode (30-60 ms, section 7.31.6.2) and reset the PHY
    /// protocol state for PE_SNK_Transition_to_default without Hard Reset. Returns
    /// only after the carrier has stopped. `Discarded` means not supported or not
    /// sent; the default implementation sends nothing.
    fn transmit_bist_carrier(&mut self) -> impl Future<Output = Result<(), DriverTxError>> {
        async { Err(DriverTxError::Discarded) }
    }
}

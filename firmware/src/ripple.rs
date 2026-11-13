//! r199/r200 ripple scope, board layer — the port of kws-x1's dedicated
//! `firmware/src/ripple.rs` acquisition task. Analysis lives in the
//! `ripple-core` workspace crate (kws-x1's firmware-core split), ported
//! 1:1 with two hardware-forced deviations, both documented there:
//! PGA_GAIN 1 (no AT32 PGA) and the AT32's discrete sample-time windows.
//!
//! Hardware (user trace, r199): PA6 / ADC12_IN6 / LQFP48 pin 16 carries
//! a diode to GND and a 100 k pull-up — the node idles around 0.6 V, so
//! a DC reading near ~750 counts (at VDDA 3.3 V) is EXPECTED, not a
//! fault. The ripple analysis is mean-centered, so the bias drops out.
//!
//! r200/r201 DEVIATION FROM kws-x1's ARRANGEMENT (hardware-forced, RM
//! receipts): kws-x1 gives ripple an EXCLUSIVE ADC. On the AT32F403A
//! that is impossible for a dedicated second ADC: RM V2.06 Table 9-5
//! (p.133) lists the DMA request sources as 0x00=No select, 0x01=ADC1,
//! 0x02=RESERVED, 0x03=ADC3 — **ADC2 has no DMA request line** (p.323),
//! and ADC3's external channels are only IN0..IN3 and IN10..IN13
//! (p.318), so it cannot see IN6/PA6 either. Ripple therefore shares
//! ADC1 with the internal-channel reads (temperature/VREFINT, ADC1-only
//! peripherals), excluded by adc.rs's BURST fail-fast flag; the D+/D−
//! taps moved to ADC3 (r201, user-directed — PA2/PA3 are ADC123_IN2/3),
//! which removes every timing-critical reader from ADC1. The r200
//! hardware round also proved the fixed request mapping is what runs
//! (the SRC_SEL registers read back 0 after write), so the burst rides
//! DMA1 CHANNEL 1 — ADC1's fixed source, the vendor example's exact
//! pairing (repeat_conversion_loop_transfer).
//!
//! Acquisition: DMA bursts of 1024 samples at 5 timebases, captures
//! every 100 ms for the UI, nothing between (kws-x1 ripple.rs:32-33).
//! The ADC converts in repeat mode for the length of the burst, then
//! stops (the F1-style DMA channel auto-disables at full transfer).
//! The ratiometric reference is ADC1's own VREFINT read (CH17) before
//! each burst — simpler than kws-x1's cross-ADC borrow, same idea.

use core::sync::atomic::{AtomicBool, AtomicU16, AtomicU8, Ordering};

use embassy_at32::dma::{self, Direction, Request, Unit, Width};
use embassy_at32::gpio::{AnalogPin, Port};
// The cortex_m_rt::interrupt macro below expands references to the
// device's `interrupt` module — bring it into scope (as main.rs does).
use embassy_at32::pac;
use embassy_at32::pac::at32f403a::interrupt;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Timer, with_timeout};

use ripple_core::ripple::{PeakHistory, Reading, SAMPLE_COUNT, SampleTime, Timebase};
use ripple_core::spectrum::Spectrum;

/// The shared ADC's clock — PCLK2 96 MHz / 8 (adc.rs). The ADCDIV in
/// CRM.CFG is common to ADC1/2/3 and programmed once by `adc::init()`.
pub const ADC_CLOCK_HZ: u32 = 12_000_000;

/// Dedicated ripple DMA channel — **channel 1, ADC1's FIXED mapping**
/// (RM V2.06 Table 9-3, p.132: DMA1 channel 1 = ADC1; the vendor's
/// repeat_conversion_loop_transfer example pairs exactly these). The
/// r200 round on channel 3 failed because the flexible request mapping
/// never engaged on this part (its SRC_SEL registers read back 0 after
/// write), leaving channel 3 on its fixed source — USART3_RX, no ADC
/// requests. board.rs's PANEL_DMA channel-1 reservation is dead code
/// (panel.rs is not in the build); ripple takes the channel.
const DMA_CHANNEL: u8 = 1;

/// Ripple pickup: PA6 = ADC12_IN6 (the diode/100k node, see module doc).
const ADC_CHANNEL: u8 = 6;

/// ADC1.ODT sits at +0x4C (PAC RegisterBlock field order: STS, CTRL1,
/// CTRL2, SPT1, SPT2, PCDTO[4], VMB[2], OSQ1..3, PSQ, PDT[4], ODT) —
/// the vendor example DMA-configures exactly `&(ADC->odt)`.
const ODT_OFFSET: usize = 0x4C;

/// A generous capture bound, in milliseconds, not a sample-rate claim
/// (kws-x1 ripple.rs:27-31): the longest burst at 12 MHz is the 500 us
/// timebase's 571 samples x 84 cycles = 4.0 ms — but the CH17 self-test
/// samples VREFINT through SPT1's 239.5-cycle window (1024 x 21 us =
/// 21.5 ms), so the bound covers that too.
const CAPTURE_TIMEOUT: Duration = Duration::from_millis(40);

/// UI capture cadence, in milliseconds; no acquisition between captures
/// (kws-x1 ripple.rs:32-33).
const UPDATE_PERIOD: Duration = Duration::from_millis(100);

/// r199 internal reference mV for `Reading`'s estimates: the FNB58's
/// VREFINT constant, matching adc.rs's `vdda_mv` math (the kws-x1
/// screens pass their line task's 1083 mV; ours is the 1.2 V bandgap).
pub const REFERENCE_MV: u16 = 1200;

/// Ripple captures: acquisition task publishes, display loop consumes
/// (kws-x1 main.rs:348-349). `None` = no capture (init failure, capture
/// failure, or an unreachable timebase).
pub static RIPPLE: Watch<CriticalSectionRawMutex, Option<Reading>, 1> = Watch::new();

/// Whether captures run: true only while the ripple screen is shown and
/// not held (kws-x1's `view == View::Ripple && !ripple_hold` gate).
pub static LIVE: AtomicBool = AtomicBool::new(false);

/// The UI's selected timebase, as an index into `Timebase::ALL`.
pub static TIMEBASE: AtomicU8 = AtomicU8::new(1);

/// The ratiometric reference — ADC1's own VREFINT (CH17) count, read
/// before each burst. 0 = none yet.
pub static REFERENCE: AtomicU16 = AtomicU16::new(0);

/// Consecutive capture failures since the last success (evidence for
/// the console line; no failure branch stays silent).
static FAILS: AtomicU8 = AtomicU8::new(0);

/// Captures that completed without the DMA interrupt waking the wait
/// (salvaged by the flag check; counted so a dead vector is visible).
static UNWAKE: AtomicU8 = AtomicU8::new(0);

/// Consecutive perfectly-flat-zero captures (an unpowered front end is
/// worth one console line, not one per capture).
static FLAT: AtomicU8 = AtomicU8::new(0);

/// r204: console 'rs' sets this; the task then runs ONE capture on CH17
/// (VREFINT, ~1485 counts at VDDA 3.3 V) through the exact burst
/// machinery and logs the outcome. A self-test burst reading ~1485
/// proves ADC1+DMA1+analysis end to end; all-zero there instead
/// indicts the DMA data path itself, not the PA6 front end.
pub static SELFTEST: AtomicBool = AtomicBool::new(false);

/// r205: console 'r2' — the same CH17 burst with 32-BIT peripheral and
/// memory widths, into a u32 buffer. If the 16-bit port read is what
/// returns zeros, the wide read shows the real 0x000005CC-ish words.
pub static SELFTEST2: AtomicBool = AtomicBool::new(false);

/// r205: console 'm' — a MEMORY-TO-MEMORY copy through the same DMA
/// channel (no peripheral, no requests: transfers start on CHEN).
/// Splits the hypothesis space: a working copy proves the engine and
/// the memory-write path, isolating the fault to the ADC peripheral
/// read; a zero/failing copy indicts the channel configuration itself.
pub static SELFTEST_M2M: AtomicBool = AtomicBool::new(false);

/// The M2M probe's pattern and destination (single-task context).
static mut M2M_SRC: [u32; 8] = [
    0x1234_5678, 0xDEAD_BEEF, 0x0BAD_C0DE, 0x1357_9BDF, 0x2468_ACE0, 0x00C0_FFEE, 0x0F0F_0F0F,
    0x7654_3210,
];
static mut M2M_DST: [u32; 8] = [0; 8];

/// One M2M transfer, hand-rolled on the PAC (the HAL models no M2M).
/// Transfers start on CHEN (no requests); 8 words complete instantly.
/// The next `Channel::start` rewrites every field this touches.
fn m2m_run() {
    use embassy_at32::pac::at32f403a::dma1::channel::ctrl::{DTD_A, MWIDTH_A, PWIDTH_A};
    let dma1 = unsafe { pac::at32f403a::DMA1::steal() };
    let ch = dma1.channel(usize::from(DMA_CHANNEL - 1));
    // SAFETY: single task context; buffers are this module's statics.
    let (src, dst) = unsafe {
        (
            core::ptr::addr_of_mut!(M2M_SRC) as *mut u32,
            core::ptr::addr_of_mut!(M2M_DST) as *mut u32,
        )
    };
    ch.ctrl().modify(|_, w| w.chen().disable());
    // SAFETY: whole-value registers; 8 items, plain addresses.
    unsafe {
        ch.dtcnt().write(|w| w.bits(8));
        ch.paddr().write(|w| w.bits(src as u32));
        ch.maddr().write(|w| w.bits(dst as u32));
    }
    ch.ctrl().modify(|_, w| {
        w.m2m().enable();
        // DTD 0 = read from the PADDR side (the pattern) — the probe
        // originally said Memory and wrote zeros INTO its own source,
        // which is what exposed the HAL's inverted direction mapping.
        w.dtd().variant(DTD_A::Peripherals);
        w.pwidth().variant(PWIDTH_A::Bit32);
        w.mwidth().variant(MWIDTH_A::Bit32);
        w.pincm().enable();
        w.mincm().enable();
        w.lm().disable();
        w.fdtien().disable();
        w.dterrien().disable();
        w.chen().enable();
        w
    });
    let mut done = false;
    for _ in 0..2_000_000 {
        if dma1.sts().read().fdtf1().bit() {
            done = true;
            break;
        }
    }
    dma1.clr().write(|w| w.fdtfc1().set_bit());
    let cnt = ch.dtcnt().read().bits();
    ch.ctrl().modify(|_, w| {
        w.m2m().disable();
        w.pincm().disable();
        w.chen().disable();
        w
    });
    // SAFETY: single task context, transfer quiesced.
    let (src_ref, dst_ref) = unsafe { (&*core::ptr::addr_of!(M2M_SRC), &*core::ptr::addr_of!(M2M_DST)) };
    let mut d = [0u8; 64];
    let mut n = 0;
    for &b in b"[ripple] m2m d=" {
        d[n] = b;
        n += 1;
    }
    d[n] = b'0' + u8::from(done);
    d[n + 1] = b' ';
    n += 2;
    for &b in b"cnt=" {
        d[n] = b;
        n += 1;
    }
    n = crate::push_dec(&mut d, n, u32::from(cnt));
    for &b in b" s0=" {
        d[n] = b;
        n += 1;
    }
    n = crate::push_hex16(&mut d, n, src_ref[0] >> 16);
    n = crate::push_hex16(&mut d, n, src_ref[0] & 0xFFFF);
    for &b in b" d0=" {
        d[n] = b;
        n += 1;
    }
    n = crate::push_hex16(&mut d, n, dst_ref[0] >> 16);
    n = crate::push_hex16(&mut d, n, dst_ref[0] & 0xFFFF);
    for &b in b" d7=" {
        d[n] = b;
        n += 1;
    }
    n = crate::push_hex16(&mut d, n, dst_ref[7] >> 16);
    n = crate::push_hex16(&mut d, n, dst_ref[7] & 0xFFFF);
    for &b in b"\r\n" {
        d[n] = b;
        n += 1;
    }
    crate::pd_log(&d[..n]);
}

fn log(line: &[u8]) {
    crate::pd_log(line);
}

/// Bring up DMA1 channel 1 + PA6. ADC1 itself stays as `adc::init()`
/// configured it (base config, calibration, OCDMAEN) — the r200 sharing
/// model. Failure leaves only ripple unavailable (the task then
/// publishes None forever, kws-x1's ripple_adc=None path).
fn init() -> Option<dma::Channel> {
    // SAFETY: exclusive-startup contract — this task alone claims PA6,
    // DMA1 channel 1 and the DMA1_CHANNEL1 vector. Nothing else
    /// configures them (PANEL_DMA's panel never existed in the build).
    unsafe {
        // PA6 analog (the ripple node). The claim is kept alive for the
        // boot by leaking the token, like the other pin claims.
        core::mem::forget(AnalogPin::new(Port::A, 6).ok()?);

        // DMA1: claim the unit, keep channel 1 for this task (dropping
        // the rest merely disables idle channels, as kws-x1's
        // `let [channel, ..] = channels` does). The NVIC vector is
        // unmasked here; the handler below drives the HAL's completion
        // waker.
        let channels = dma::Channel::split(Unit::Dma1).ok()?;
        let channel = channels.into_iter().nth(usize::from(DMA_CHANNEL - 1))?;
        cortex_m::peripheral::NVIC::unmask(pac::at32f403a::interrupt::DMA1_CHANNEL1);
        Some(channel)
    }
}

/// What went wrong in [`capture`] — carried by the fail dump.
enum Fail {
    /// The DMA channel was still marked busy (start refused).
    Busy,
    /// The burst never finished within CAPTURE_TIMEOUT.
    Timeout,
    /// The DMA reported a data error.
    DataError,
}

/// Capture with `sampling`, first switching the sample time if it changed
/// (kws-x1 ripple.rs `capture`). One repeat-mode burst of `adc_channel`
/// into `samples`, bracketed by adc.rs's BURST claim. The sample-time
/// switch targets channel 6's window (the PA6 pickup); the CH17
/// self-test rides SPT1's fixed 239.5-cycle window instead.
async fn capture(
    channel: &mut dma::Channel,
    samples: &mut [u16; SAMPLE_COUNT],
    sample_code: &mut u8,
    adc_channel: u8,
    timebase: Timebase,
    sampling: ripple_core::ripple::Sampling,
) -> Result<Reading, Fail> {
    let adc = unsafe { pac::at32f403a::ADC1::steal() };
    let code = sampling.sample_time.code();
    if *sample_code != code {
        adc.spt2().modify(|_, w| unsafe { w.cspt6().bits(code) });
        *sample_code = code;
    }
    if !crate::adc::burst_begin() {
        return Err(Fail::Busy);
    }
    // From here every exit path must burst_end().
    // The one-shot reads rewrite OSQ3.OSN1 per conversion, so the burst's
    // channel is programmed per capture too (right after the claim).
    // SEQUENCE mode joins the repeat for the burst — the vendor's
    // repeat_conversion_loop_transfer shape (sequence TRUE + repeat
    // TRUE); r200's dump could not prove repeat fires with SQEN=0, and
    // adc.rs's one-shot path wants it clear again afterwards.
    adc.osq3().modify(|_, w| unsafe { w.osn1().bits(adc_channel) });
    adc.ctrl1().modify(|_, w| w.sqen().set_bit());
    adc.sts().write(|w| w.occe().bit(true)); // write-1-to-clear
    // PRIMING CONVERSION (r208, discarded): the pre-burst VREFINT read
    // leaves the S/H cap holding ~1484 counts; this channel's first
    // burst sample must slew to the node's own voltage (PA6 sits ~750
    // counts away), which a 1.5-cycle window cannot do — the first
    // sample(s) then land in between and max-min reads HUNDREDS of
    // counts of phantom peak-to-peak (the r207 '~600 mV on a good
    // supply' report). One thrown-away conversion of the burst's own
    // channel, at its own window, slews the cap first. kws-x1 needs
    // nothing like this — its ripple ADC is exclusive; ours shares
    // ADC1 with the CH16/17 internal reads.
    adc.ctrl2().modify(|_, w| w.ocswtrg().triggered());
    for _ in 0..2_000_000 {
        if adc.sts().read().occe().bit() {
            let _ = adc.odt().read().odt().bits(); // discard
            break;
        }
    }
    adc.sts().write(|w| w.occe().bit(true));
    // Arm the DMA first, then start the burst — the vendor example's
    // order (dma_channel_enable before adc_ordinary_software_trigger).
    let odt = unsafe { (pac::at32f403a::ADC1::ptr() as *mut u8).add(ODT_OFFSET) as *mut u32 };
    let started = channel.start(
        Request::Adc1,
        Direction::PeripheralToMemory,
        odt,
        samples.as_mut_ptr() as *mut u32,
        SAMPLE_COUNT,
        Width::Bit16,
        true,
    );
    match started {
        Ok(()) => {}
        Err(_) => {
            adc.ctrl1().modify(|_, w| w.sqen().clear_bit());
            crate::adc::burst_end();
            return Err(Fail::Busy);
        }
    }
    // One atomic write starts the repeat burst (r203: two separate CTRL2
    // writes left a window where repeat was armed but the trigger had
    // not fired — one r202 capture in ~30 stalled after 2 transfers).
    adc.ctrl2()
        .modify(|_, w| w.rpen().enable().ocswtrg().triggered());
    let done = with_timeout(CAPTURE_TIMEOUT, channel.wait()).await;
    // Stop the repeat burst whether the capture finished or not; with
    // the channel's count exhausted there is nothing left to drain.
    // Clearing OCCE afterwards also drops the tail conversion's pending
    // DMA request, so it cannot kick the next burst's channel early.
    adc.ctrl2().modify(|_, w| w.rpen().disable());
    adc.sts().write(|w| w.occe().bit(true)); // write-1-to-clear
    adc.ctrl1().modify(|_, w| w.sqen().clear_bit());
    // The timeout may have beaten an unwakeful (but finished) transfer:
    // check the hardware full-transfer flag directly before declaring
    // failure. A salvaged capture still reports the unwake count — the
    // interrupt path is broken then, and the log must say so.
    let result = match done {
        Ok(Ok(())) => Ok(ripple_core::ripple::analyze(
            samples,
            timebase,
            sampling.window,
        )),
        Ok(Err(_)) => {
            channel.abort();
            Err(Fail::DataError)
        }
        Err(_) => {
            let finished =
                unsafe { pac::at32f403a::DMA1::steal().sts().read().fdtf1().bit() };
            if finished {
                channel.abort();
                let count = UNWAKE.fetch_add(1, Ordering::Relaxed);
                if count == 0 || count % 20 == 19 {
                    log(b"[ripple] unwake salvage\r\n");
                }
                Ok(ripple_core::ripple::analyze(
                    samples,
                    timebase,
                    sampling.window,
                ))
            } else {
                channel.abort();
                Err(Fail::Timeout)
            }
        }
    };
    crate::adc::burst_end();
    result
}

/// One `[ripple]` evidence line for a failure: what failed plus the
/// state of every link in the chain (ADC config, its conversion flag,
/// the DMA channel, the remaining transfer count, the flex request
/// mapping and the first landed sample). Built as complete ≤64-byte
/// fragments (the r163 line rule; the two halves are pushed in order).
fn dump_fail(fail: Fail, samples: &[u16; SAMPLE_COUNT]) {
    let adc = unsafe { pac::at32f403a::ADC1::steal() };
    let dma1 = unsafe { pac::at32f403a::DMA1::steal() };
    let cause: &[u8] = match fail {
        Fail::Busy => b"busy",
        Fail::Timeout => b"timeout",
        Fail::DataError => b"dterr",
    };
    let mut d = [0u8; 64];
    let mut n = 0;
    for &b in b"[ripple] fail " {
        d[n] = b;
        n += 1;
    }
    for &b in cause {
        d[n] = b;
        n += 1;
    }
    for &b in b" c2=" {
        d[n] = b;
        n += 1;
    }
    // Eight hex digits: CTRL2's trigger-enable bits live at bit 20 and
    // above — the r202 4-digit dump hid them and briefly misread as a
    // missing config (c2=0101 was only the register's low half).
    n = crate::push_hex16(&mut d, n, adc.ctrl2().read().bits() >> 16);
    n = crate::push_hex16(&mut d, n, adc.ctrl2().read().bits() & 0xFFFF);
    for &b in b" sts=" {
        d[n] = b;
        n += 1;
    }
    n = crate::push_hex8(&mut d, n, adc.sts().read().bits());
    for &b in b" ch=" {
        d[n] = b;
        n += 1;
    }
    n = crate::push_hex16(&mut d, n, dma1.channel(0).ctrl().read().bits());
    for &b in b" cnt=" {
        d[n] = b;
        n += 1;
    }
    n = crate::push_dec(&mut d, n, u32::from(dma1.channel(0).dtcnt().read().bits()));
    for &b in b"\r\n" {
        d[n] = b;
        n += 1;
    }
    crate::pd_log(&d[..n]);

    let mut d = [0u8; 64];
    let mut n = 0;
    for &b in b"[ripple] fail sel=" {
        d[n] = b;
        n += 1;
    }
    n = crate::push_hex16(&mut d, n, dma1.dma_src_sel0().read().bits());
    d[n] = b'/';
    n += 1;
    n = crate::push_hex16(&mut d, n, dma1.dma_src_sel1().read().bits());
    for &b in b" s0=" {
        d[n] = b;
        n += 1;
    }
    n = crate::push_dec(&mut d, n, u32::from(samples[0]));
    for &b in b" s1023=" {
        d[n] = b;
        n += 1;
    }
    n = crate::push_dec(&mut d, n, u32::from(samples[SAMPLE_COUNT - 1]));
    for &b in b"\r\n" {
        d[n] = b;
        n += 1;
    }
    crate::pd_log(&d[..n]);
}

/// The channel's live address wiring (r205): PADDR/MADDR as the DMA
/// holds them, plus the intended buffer address — one `[ripple]` line.
fn log_dma_wide(channel: &dma::Channel, buffer: *const u32) {
    let dma1 = unsafe { pac::at32f403a::DMA1::steal() };
    let ch = dma1.channel(usize::from(DMA_CHANNEL - 1));
    let mut d = [0u8; 64];
    let mut n = 0;
    for &b in b"[ripple] dma pa=" {
        d[n] = b;
        n += 1;
    }
    n = crate::push_hex16(&mut d, n, ch.paddr().read().bits() >> 16);
    n = crate::push_hex16(&mut d, n, ch.paddr().read().bits() & 0xFFFF);
    for &b in b" ma=" {
        d[n] = b;
        n += 1;
    }
    n = crate::push_hex16(&mut d, n, ch.maddr().read().bits() >> 16);
    n = crate::push_hex16(&mut d, n, ch.maddr().read().bits() & 0xFFFF);
    for &b in b" sa=" {
        d[n] = b;
        n += 1;
    }
    n = crate::push_hex16(&mut d, n, (buffer as u32) >> 16);
    n = crate::push_hex16(&mut d, n, (buffer as u32) & 0xFFFF);
    for &b in b"\r\n" {
        d[n] = b;
        n += 1;
    }
    crate::pd_log(&d[..n]);
}

/// r205: the 32-bit CH17 burst — same machinery as [`capture`], but
/// Width::Bit32 into a u32 buffer, logging raw words and the count of
/// nonzero ones. Diagnoses whether the 16-bit peripheral read is the
/// zero-source (16-bit reads of ODT return 0, 32-bit return the word).
async fn capture32(
    channel: &mut dma::Channel,
    wide: &mut [u32; SAMPLE_COUNT],
) -> Option<(u32, u32, u32)> {
    let adc = unsafe { pac::at32f403a::ADC1::steal() };
    if !crate::adc::burst_begin() {
        return None;
    }
    adc.osq3().modify(|_, w| unsafe { w.osn1().bits(17) });
    adc.ctrl1().modify(|_, w| w.sqen().set_bit());
    adc.sts().write(|w| w.occe().bit(true));
    let odt = unsafe { (pac::at32f403a::ADC1::ptr() as *mut u8).add(ODT_OFFSET) as *mut u32 };
    let started = channel.start(
        Request::Adc1,
        Direction::PeripheralToMemory,
        odt,
        wide.as_mut_ptr(),
        SAMPLE_COUNT,
        Width::Bit32,
        true,
    );
    if started.is_err() {
        adc.ctrl1().modify(|_, w| w.sqen().clear_bit());
        crate::adc::burst_end();
        return None;
    }
    adc.ctrl2()
        .modify(|_, w| w.rpen().enable().ocswtrg().triggered());
    let done = with_timeout(CAPTURE_TIMEOUT, channel.wait()).await;
    adc.ctrl2().modify(|_, w| w.rpen().disable());
    adc.sts().write(|w| w.occe().bit(true));
    adc.ctrl1().modify(|_, w| w.sqen().clear_bit());
    crate::adc::burst_end();
    match done {
        Ok(Ok(())) => {
            let nonzero = wide.iter().filter(|&&w| w != 0).count() as u32;
            Some((wide[0], wide[1], nonzero))
        }
        _ => {
            channel.abort();
            None
        }
    }
}

#[embassy_executor::task]
pub async fn run() {
    let mut channel = init();
    match channel.is_some() {
        true => log(b"[ripple] adc1+dma1c1 up\r\n"),
        false => log(b"[ripple] init FAIL\r\n"),
    }
    let sender = RIPPLE.sender();
    let mut samples = [0u16; SAMPLE_COUNT];
    // The spectrum's working buffer, kept with the task rather than on
    // the stack (kws-x1 ripple.rs:60-62). r205 adds the 32-bit self-test
    // buffer.
    let mut scratch = [0f32; SAMPLE_COUNT];
    let mut wide = [0u32; SAMPLE_COUNT];
    let mut sample_code = 0xFFu8;
    let mut timebase = Timebase::default();
    let mut history = PeakHistory::default();
    loop {
        // r204: the console 'rs' self-test — one CH17 (VREFINT) burst
        // through the exact capture machinery, logged with its own
        // min/max/mean. Independent of the screen and the front end.
        // r205: followed by the channel's live address wiring, and the
        // 'r2' 32-bit-wide variant separating a broken 16-bit port read
        // from a wrong-address transfer.
        if SELFTEST.swap(false, Ordering::Relaxed) {
            if let (Some(channel), Some(sampling)) =
                (channel.as_mut(), Timebase::Us500.sampling(ADC_CLOCK_HZ))
            {
                let buffer = samples.as_ptr() as *const u32;
                match capture(
                    channel,
                    &mut samples,
                    &mut sample_code,
                    17,
                    Timebase::Us500,
                    sampling,
                )
                .await
                {
                    // CH17 rides the fixed 239.5-cycle window, so the
                    // per-timebase sample-time cache is now stale for
                    // channel 6 — force the next PA6 capture to re-set it.
                    Ok(reading) => {
                        sample_code = 0xFF;
                        log_dec(
                            b"[ripple] self c17 mn= mx= mean=",
                            &[
                                u32::from(reading.min),
                                u32::from(reading.max),
                                u32::from(reading.mean),
                            ],
                        );
                    }
                    Err(_) => {
                        log(b"[ripple] self c17 FAIL\r\n");
                    }
                }
                log_dma_wide(channel, buffer);
                // r207: the same burst on the real pickup — the PA6
                // verdict without needing the screen (the 'a' scan read
                // c6 ~2234 by CPU; this checks it through the DMA).
                if let Some(sampling) = Timebase::Us500.sampling(ADC_CLOCK_HZ) {
                    match capture(
                        channel,
                        &mut samples,
                        &mut sample_code,
                        ADC_CHANNEL,
                        Timebase::Us500,
                        sampling,
                    )
                    .await
                    {
                        Ok(reading) => {
                            sample_code = 0xFF;
                            log_dec(
                                b"[ripple] self c6 mn= mx= mean=",
                                &[
                                    u32::from(reading.min),
                                    u32::from(reading.max),
                                    u32::from(reading.mean),
                                ],
                            );
                        }
                        Err(_) => log(b"[ripple] self c6 FAIL\r\n"),
                    }
                }
            } else {
                log(b"[ripple] self c17 NOCHANNEL\r\n");
            }
        }
        if SELFTEST2.swap(false, Ordering::Relaxed) {
            if let Some(channel) = channel.as_mut() {
                match capture32(channel, &mut wide).await {
                    Some((w0, w1, nonzero)) => {
                        log_dec(
                            b"[ripple] self32 w0= w1= nz=",
                            &[w0, w1, nonzero],
                        );
                    }
                    None => log(b"[ripple] self32 FAIL\r\n"),
                }
                log_dma_wide(channel, wide.as_ptr());
                sample_code = 0xFF;
            } else {
                log(b"[ripple] self32 NOCHANNEL\r\n");
            }
        }
        // r205: the M2M probe — pure engine test, no peripheral.
        if SELFTEST_M2M.swap(false, Ordering::Relaxed) {
            m2m_run();
            sample_code = 0xFF;
        }
        let live = LIVE.load(Ordering::Relaxed);
        let want = Timebase::ALL[usize::from(TIMEBASE.load(Ordering::Relaxed))
            .min(Timebase::ALL.len() - 1)];
        if live {
            // The ratiometric reference: ADC1's own VREFINT read before
            // the burst claims the converter.
            if let Some(count) = crate::adc::read_vref_raw() {
                REFERENCE.store(count, Ordering::Relaxed);
            }
            // Peaks at another sample rate are not comparable.
            if want != timebase {
                timebase = want;
                history.clear();
            }
            let reading = match (channel.as_mut(), timebase.sampling(ADC_CLOCK_HZ)) {
                (Some(channel), Some(sampling)) => {
                    match capture(
                        channel,
                        &mut samples,
                        &mut sample_code,
                        ADC_CHANNEL,
                        timebase,
                        sampling,
                    )
                    .await
                    {
                        Ok(mut reading) => {
                            let fails = FAILS.swap(0, Ordering::Relaxed);
                            if fails > 0 {
                                log_dec(b"[ripple] ok fails=", &[u32::from(fails)]);
                            }
                            // A perfectly flat ZERO capture is its own
                            // evidence (r203): min==0 trips CLIPPED on
                            // screen, and with the DUT side unpowered
                            // (bus at 0 V) that is the EXPECTED reading
                            // of the PA6 node — say so once in a while
                            // instead of looking like a fault.
                            if reading.min == 0 && reading.max == 0 {
                                let flats = FLAT.fetch_add(1, Ordering::Relaxed);
                                if flats == 0 || flats % 20 == 19 {
                                    log(b"[ripple] flat zero node=0V\r\n");
                                }
                            } else {
                                FLAT.store(0, Ordering::Relaxed);
                            }
                            history.add(&mut reading);
                            reading.spectrum = Spectrum::new(&samples, &mut scratch);
                            Some(reading)
                        }
                        Err(fail) => {
                            // First failure and every 20th after (2 s at
                            // the capture cadence): the evidence dump.
                            let count = FAILS.fetch_add(1, Ordering::Relaxed);
                            if count == 0 || count % 20 == 19 {
                                dump_fail(fail, &samples);
                            }
                            None
                        }
                    }
                }
                _ => None,
            };
            sender.send(reading);
        } else {
            // A smoothed readout restarts with the next live capture.
            history.clear();
        }
        Timer::after(UPDATE_PERIOD).await;
    }
}

/// One `[ripple] ` console line with decimal values (the 64-byte rule).
fn log_dec(prefix: &[u8], values: &[u32]) {
    let mut d = [0u8; 64];
    let mut n = 0;
    for &b in prefix {
        d[n] = b;
        n += 1;
    }
    for (i, &v) in values.iter().enumerate() {
        if i > 0 {
            d[n] = b' ';
            n += 1;
        }
        n = crate::push_dec(&mut d, n, v);
    }
    d[n] = b'\r';
    d[n + 1] = b'\n';
    crate::pd_log(&d[..n + 2]);
}

/// DMA1 channel 1 completion: feed the HAL's waker (console.rs owns the
/// EXINT/SysTick vectors; this one belongs to the ripple task).
#[cortex_m_rt::interrupt]
unsafe fn DMA1_CHANNEL1() {
    embassy_at32::dma::on_interrupt(Unit::Dma1, DMA_CHANNEL);
}

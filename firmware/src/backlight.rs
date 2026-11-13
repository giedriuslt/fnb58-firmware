//! r216: LCD backlight PWM on PB3 through TMR2 channel 2 — idle dimming.
//!
//! PB3 drove the backlight as a plain push-pull output (lcd.rs) since
//! r111; board.rs's note ("a PWM timer can replace it once the real pin
//! is known") becomes actionable now that the pin is traced: PB3 is
//! JTDO, freed for GPIO/AF use by the SWJTAG remap (lcd.rs `new()`,
//! IOMUX_REMAP7.SWJTAG_GMUX = 100). The AT32's F1-style IOMUX gives PB3
//! a timer function only under **TMR2 partial remap 01** (RM IOMUX
//! chapter, TMR2_MUX = 01: CH1 = PA15, CH2 = PB3, CH3 = PB10, CH4 =
//! PB11) — PA15 is the LEFT button, a plain GPIO input the routing
//! cannot disturb (remaps reroute peripheral signals, not GPIO modes).
//!
//! The module follows the vendor `tmr_base_init`/`pwm_output_tmr3`
//! pattern on PAC registers (the `dac.rs` precedent — the HAL's `pwm`
//! module is typed on TMR3's register block, which cannot reach PB3;
//! TMR2's CM registers ARE mapped in this PAC, so no raw pointers):
//! DIV = prescaler − 1, PR = period − 1, one overflow software event
//! reloads both immediately, period + compare buffering (PRBEN, C2OBEN)
//! for glitch-free duty changes, PWM mode A (C2OCTRL = 0b110 — output
//! active while the counter runs below the compare value), polarity
//! C2P = 0 (active-high: high = lit).
//!
//! Clock: TMR2 sits on APB1; divided APB buses run timers at twice the
//! bus clock (`Clocks::timer1_hz`) — 192 MHz at the 192 MHz USB
//! profile. PWM runs at 20 kHz (above audible, above visible flicker):
//! period 9600 ticks, 20% = 1920.

use core::sync::atomic::{AtomicU32, Ordering};

use embassy_at32::gpio::{AlternatePin, Drive, Port};
use embassy_at32::pac;

/// PWM frequency: above audible, above visible flicker.
const PWM_HZ: u32 = 20_000;

/// The programmed period in counter ticks (PR + 1); 0 before init.
static PERIOD_TICKS: AtomicU32 = AtomicU32::new(0);

/// Claim PB3, route TMR2 CH2 to it, program the base and start at 100%
/// (the screen stays lit through boot; the idle logic dims later).
/// Returns the TMR2_MUX readback (0b01 = the remap stuck — r217: the
/// r216 build wrote REMAP **before the IOMUX APB2 clock was on** (it is
/// only enabled later, in Lcd::new()), so the write was lost and the
/// backlight sat dead; the readback turns a silent lost-write into boot
/// evidence.
///
/// # Safety
///
/// Exclusive-startup contract: claims PB3 and TMR2 once per boot, and
/// writes IOMUX REMAP.TMR2_MUX (a field no other code touches — lcd.rs
/// uses REMAP7's SWJTAG field). The LEFT button sits on PA15, CH1's
/// remapped pin, as a plain GPIO input: untouched by the timer routing.
/// (Remap 01 also reroutes CH3/CH4 to PA2/PA3 — the D± read taps, which
/// are analog-configured pads, so that routing is inert.)
pub unsafe fn init(timer_hz: u32) -> u8 {
    let crm = pac::at32f403a::CRM::steal();
    let iomux = pac::at32f403a::IOMUX::steal();
    let tmr = pac::at32f403a::TMR2::steal();

    // The IOMUX sits on APB2 and its clock is OFF this early (Lcd::new()
    // enables it later) — register writes to a clock-dead peripheral are
    // lost. Enable it here; the later set_bit is idempotent.
    crm.apb2en().modify(|_, w| w.iomux().bit(true));

    // PB3 alternate-function push-pull (the timer drives the pad; a
    // plain GPIO output mode would keep the pad off the timer).
    core::mem::forget(
        AlternatePin::new_push_pull(Port::B, 3, Drive::Moderate).expect("PB3 claim"),
    );

    // TMR2 partial remap 01 (RM IOMUX chapter p.114): CH1/PA15,
    // CH2/PB3, CH3/PA2, CH4/PA3. Read-modify-write: REMAP also holds
    // TMR1/TMR3/TMR4 mux fields.
    iomux.remap().modify(|_, w| w.tmr2_mux().bits(0b01));
    let mux = iomux.remap().read().tmr2_mux().bits();

    crm.apb1en().modify(|_, w| w.tmr2().bit(true));

    // Base: 20 kHz at `timer_hz` (9600 ticks at 192 MHz), prescaler 1.
    let period = timer_hz / PWM_HZ;
    // SAFETY: whole-value registers; the overflow event reloads DIV/PR
    // immediately (vendor tmr_base_init's pattern).
    unsafe {
        tmr.div().write(|w| w.bits(0));
        tmr.pr().write(|w| w.bits(period - 1));
        tmr.swevt().write(|w| w.ovfswtr().set_bit());
    }
    // Buffer period and compare so duty changes take effect at a period
    // boundary (glitch-free).
    tmr.ctrl1().modify(|_, w| w.prben().set_bit());
    // CH2: PWM mode A (C2OCTRL = 0b110), compare preload on. C2P (the
    // polarity bit in CCTRL) stays 0 = active-high.
    tmr.cm1_output()
        .modify(|_, w| unsafe { w.c2octrl().bits(0b110) }.c2oben().set_bit());

    PERIOD_TICKS.store(period, Ordering::Relaxed);
    set_percent(100);
    tmr.cctrl().modify(|_, w| w.c2en().set_bit());
    tmr.ctrl1().modify(|_, w| w.tmren().set_bit());
    mux
}

/// Duty in percent of the full period (0 = off, 100 = always on). The
/// compare register is 32-bit on TMR2 and takes `period` itself (the
/// counter never reaches it → output continuously active = true 100%).
/// Returns the applied tick count.
pub fn set_percent(percent: u32) -> u32 {
    let period = PERIOD_TICKS.load(Ordering::Relaxed);
    if period == 0 {
        return 0;
    }
    let duty = period * percent.min(100) / 100;
    let tmr = unsafe { pac::at32f403a::TMR2::steal() };
    // SAFETY: whole-value 32-bit compare register, owned by this module.
    unsafe {
        tmr.c2dt().write(|w| w.bits(duty));
    }
    duty
}

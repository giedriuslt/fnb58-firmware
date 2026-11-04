//! AT32F403A ADC1: internal temperature sensor (CH16) with VREFINT
//! (CH17) ratiometric correction, for the PC-link temperature field
//! (r136) and the console heartbeat.
//!
//! Artery's `internal_temperature_sensor` example on PAC registers
//! (the embassy HAL has no ADC yet): software-triggered one-shot
//! ordinary conversions, 239.5-cycle sample time, calibration at init.
//! Formula from the example: T = (1.28 V - V_sense) / (-0.00426) + 25,
//! except V_sense is scaled by the measured VDDA (1.2 V internal
//! reference) instead of the example's fixed 3.3 V.

use embassy_at32::pac;

/// ADC clock: PCLK2 96 MHz / 8 = 12 MHz (the vendor example runs the
/// same ADC at 30 MHz, so this is comfortably in range). CRM_ADC_DIV_8
/// = 0b011 split across cfg.adcdiv1_0 (bits 14:15) and cfg.adcdiv2
/// (bit 28).
const ADCDIV_LOW: u8 = 0b11;
const ADCDIV_HIGH: bool = false;

/// Wait bound for one 12-bit conversion at 12 MHz (~21 us) and for the
/// calibration steps: generous, they take far less.
const WAIT_BOUND: u32 = 2_000_000;

/// Enable and calibrate ADC1 with CH16/CH17 ready. Call once at boot,
/// before either USB mode's loop runs.
pub fn init() {
    let crm = unsafe { pac::at32f403a::CRM::steal() };
    let adc = unsafe { pac::at32f403a::ADC1::steal() };

    // Peripheral clock + divider, then the vendor adc_reset pulse.
    crm.apb2en().modify(|_, w| w.adc1().set_bit());
    crm.cfg().modify(|_, w| unsafe {
        w.adcdiv1_0().bits(ADCDIV_LOW);
        w.adcdiv2().bit(ADCDIV_HIGH);
        w
    });
    crm.apb2rst().modify(|_, w| w.adc1().set_bit());
    crm.apb2rst().modify(|_, w| w.adc1().clear_bit());

    // Base config: no sequence mode, no repeat, right-aligned data,
    // internal temperature sensor + VINTRV enabled, software trigger
    // for the ordinary group.
    adc.ctrl1().modify(|_, w| w.sqen().clear_bit());
    adc.ctrl2().modify(|_, w| {
        w.rpen().disable();
        w.dtalign().right();
        w.itsrven().set_bit();
        unsafe { w.octesel_l().bits(0b111) }; // ADC12_ORDINARY_TRIG_SOFTWARE
        w.octesel_h().clear_bit();
        w.octen().set_bit();
        w.adcen().enable()
    });

    // CH16/CH17 sample time 239.5 cycles (sensor needs the long window).
    adc.spt1().modify(|_, w| unsafe {
        w.cspt16().bits(0b111);
        w.cspt17().bits(0b111)
    });
    // Ordinary sequence length 1; the channel is written per read.
    adc.osq1().modify(|_, w| unsafe { w.oclen().bits(0) });

    // Calibration (vendor example: init -> wait, start -> wait).
    adc.ctrl2().modify(|_, w| w.adcalinit().init());
    for _ in 0..WAIT_BOUND {
        if adc.ctrl2().read().adcalinit().is_completed() {
            break;
        }
    }
    adc.ctrl2().modify(|_, w| w.adcal().calibrate());
    for _ in 0..WAIT_BOUND {
        if adc.ctrl2().read().adcal().is_completed() {
            break;
        }
    }
}

/// One software-triggered conversion of `ch` (0..=17). Returns None on
/// the (never-seen) EOC timeout.
fn read_raw(ch: u8) -> Option<u16> {
    let adc = unsafe { pac::at32f403a::ADC1::steal() };
    adc.osq3().modify(|_, w| unsafe { w.osn1().bits(ch) });
    adc.sts().write(|w| w.occe().bit(true)); // write-1-to-clear
    adc.ctrl2().modify(|_, w| w.ocswtrg().triggered());
    for _ in 0..WAIT_BOUND {
        if adc.sts().read().occe().bit() {
            return Some(adc.odt().read().odt().bits() as u16);
        }
    }
    None
}

/// Internal die temperature in tenths of a degree C, measured against
/// the 1.2 V internal reference so USB-supply drift does not skew it.
/// None = conversion timed out (caller keeps the previous value).
pub fn read_temp_deci() -> Option<i32> {
    let vref_raw = u32::from(read_raw(17)?);
    let vref_raw = u32::from(read_raw(17)?);
    let temp_raw = u32::from(read_raw(16)?);
    if vref_raw == 0 {
        return None;
    }
    // VDDA from the 1.2 V reference, then the sensor voltage in uV.
    // u64 for the product: raw*vdda*1000 reaches ~5.2e9 and WRAPS u32
    // (release builds wrap silently) - the r136 "0.0 C" was exactly
    // that overflow computing -221 C, clamped back to 0.
    let vdda_mv = 1200 * 4095 / vref_raw;
    let vsense_uv = u64::from(temp_raw) * u64::from(vdda_mv) * 1000 / 4095;
    // T = (Vs - 1.28 V) / 4.26 mV/K + 25 C, in millidegrees first
    // (the raw span would brush i32::MAX).
    let t_milli: i64 = (vsense_uv as i64 - 1_280_000) * 1000 / 4260 + 25_000;
    Some((t_milli / 100) as i32)
}

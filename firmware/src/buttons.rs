//! Button discovery monitor: every probe-safe GPIO becomes an input with
//! the internal pull-up enabled; transitions (a button shorting the pin
//! to ground) are debounced and reported over the CDC.
//!
//! Excluded pins: PA11/PA12 (USB data), PA13/PA14 (SWD), PB6/PB7 (the
//! live I2C bus), PC14/PC15 (crystal territory). PD0/PD1 (crystal) are
//! not in the enumerated set at all.
//!
//! Buttons are assumed active-low (pull-up + short to ground) — the same
//! scheme the KWS-X1 firmware used. If nothing changes in this mode, a
//! pull-down phase (press-to-VCC buttons) is the fallback.


const GPIOA_BASE: usize = 0x4001_0800;
const GPIOB_BASE: usize = 0x4001_0C00;
const GPIOC_BASE: usize = 0x4001_1000;

/// (port letter, base address, pin count) for the monitored ports.
const PORTS: [(u8, usize, u8); 3] = [(b'A', GPIOA_BASE, 16), (b'B', GPIOB_BASE, 16), (b'C', GPIOC_BASE, 13)];

/// Pins excluded from monitoring (function pins: USB, SWD, I2C1).
const SKIP: [(u8, u8); 6] = [(b'A', 11), (b'A', 12), (b'A', 13), (b'A', 14), (b'B', 6), (b'B', 7)];

const DEBOUNCE_TICKS: u8 = 10; // ~20 ms at the 2 ms loop cadence

static mut LAST: [u16; 3] = [0xFFFF; 3];
static mut DEB_DOWN: [u8; 47] = [0; 47];
static mut DEB_UP: [u8; 47] = [0; 47];
static mut INITED: bool = false;

fn is_skipped(port: u8, num: u8) -> bool {
    SKIP.iter().any(|(p, n)| *p == port && *n == num)
}

fn pin_index(port: u8, num: u8) -> usize {
    // Flat index into the debounce arrays.
    (port as usize - b'A' as usize) * 16 + num as usize
}

/// Configure every monitored pin as input with the pull-up latched, and
/// capture the baseline levels. Enables the GPIOA/B/C clocks.
pub fn init() {
    // SAFETY: bring-up scanner; exclusive CRM/GPIO ownership.
    let crm = unsafe { embassy_at32::pac::at32f403a::CRM::steal() };
    crm.apb2en().modify(|_, w| {
        w.gpioa().bit(true);
        w.gpiob().bit(true);
        w.gpioc().bit(true);
        w
    });
    for (port, base, count) in PORTS {
        let basereg = base as *mut u32;
        let scr = (base + 0x10) as *mut u32;
        for num in 0..count {
            if is_skipped(port, num) {
                continue;
            }
            let (reg, shift) = if num < 8 {
                (basereg, num * 4)
            } else {
                ((base + 4) as *mut u32, (num - 8) * 4)
            };
            // SAFETY: valid GPIO register block under init's ownership.
            unsafe {
                let value = (reg.read_volatile() & !(0xF << shift)) | (0b0010 << shift);
                reg.write_volatile(value);
                // Latch the pull direction high (pull-up) via SCR set bits.
                scr.write_volatile(1 << num);
            };
        }
        // Baseline.
        // SAFETY: valid GPIO register block.
        let levels = unsafe { ((base + 8) as *const u32).read_volatile() } as u16;
        // SAFETY: single-threaded bring-up.
        unsafe {
            LAST[port as usize - b'A' as usize] = levels;
        }
    }
    // SAFETY: single-threaded bring-up.
    unsafe {
        INITED = true;
    }
}

/// Poll every monitored pin; report debounced transitions as
/// `btn P<x> down` / `btn P<x> up` lines through the CDC.
pub fn poll() {
    // SAFETY: single-threaded bring-up.
    // SAFETY: single-threaded.
    if unsafe { !INITED } {
        return;
    }
    for (port, base, count) in PORTS {
        // SAFETY: the GPIO clocks are enabled by init().
        let levels = unsafe { ((base + 8) as *const u32).read_volatile() } as u16;
        let last_idx = port as usize - b'A' as usize;
        // SAFETY: single-threaded.
        let changed = unsafe { LAST[last_idx] } ^ levels;
        if changed == 0 {
            continue;
        }
        for num in 0..count {
            let bit = 1 << num;
            if changed & bit == 0 || is_skipped(port, num) {
                continue;
            }
            let idx = pin_index(port, num);
            let now_down = levels & bit == 0;
            // SAFETY: single-threaded.
            let (deb, other) = unsafe {
                if now_down {
                    (&mut DEB_DOWN[idx], &mut DEB_UP[idx])
                } else {
                    (&mut DEB_UP[idx], &mut DEB_DOWN[idx])
                }
            };
            *deb = deb.saturating_add(1);
            *other = 0;
            if *deb >= DEBOUNCE_TICKS {
                let mut line = [0u8; 24];
                let prefix = b"btn P";
                line[..prefix.len()].copy_from_slice(prefix);
                line[prefix.len()] = port;
                let mut n = prefix.len() + 1;
                // Decimal pin number.
                if num >= 10 {
                    line[n] = b'0' + num / 10;
                    n += 1;
                }
                line[n] = b'0' + num % 10;
                n += 1;
                let suffix: &[u8] = if now_down { b" down" } else { b" up" };
                line[n..n + suffix.len()].copy_from_slice(suffix);
                n += suffix.len();
                crate::send_line(&line[..n]);
                // SAFETY: single-threaded.
                unsafe {
                    let l = &mut LAST[last_idx];
                    *l ^= bit;
                }
                *deb = 0;
            }
        }
    }
}

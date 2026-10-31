//! FUSB302 INT_N pin discovery (r72).
//!
//! The INT line is open-drain: without knowing the pin we cannot pull it
//! up, and a floating pin reads low — indistinguishable from dead pins
//! in an idle snapshot. Two facts make the search deterministic:
//!
//! 1. The FUSB302 latches INT_N low until its interrupt registers are
//!    read. This build SKIPS the per-tick interrupt read, so the first
//!    attach event (VBUS OK or the first PD packet) holds INT_N low
//!    indefinitely — no timing race.
//! 2. kws-x1 uses an INTERNAL pull-up on its INT input (PB12), so the
//!    FNB58's INT pin most likely has no external pull either; an
//!    internal pull-up per candidate is the only way to see it.
//!
//! Sweep: one candidate per ~100 ms window, internal pull-up latched,
//! 2 ms-dense sampling, back to floating after. Any pin reading LOW is
//! reported once per state change. Button pins (PB4/8/9, permanent
//! pull-ups from Buttons) keep their pulls — a LOW there without a
//! press is also a discovery.

use crate::buttons;
use crate::Sender;
use embassy_time::Timer;

const GPIOA_BASE: usize = 0x4001_0800;
const GPIOB_BASE: usize = 0x4001_0C00;
const GPIOC_BASE: usize = 0x4001_1000;

/// Candidates: every probe-safe GPIO, including the button pins.
/// Excluded: PA11/12 (USB), PA13/14 (SWD), PB3 (backlight output),
/// PB6/7 (I2C1), PB10-15 (LCD), PC14/15 (crystal).
const CANDS: [(u8, usize, u8); 20] = [
    (b'A', GPIOA_BASE, 0),
    (b'A', GPIOA_BASE, 1),
    (b'A', GPIOA_BASE, 2),
    (b'A', GPIOA_BASE, 3),
    (b'A', GPIOA_BASE, 4),
    (b'A', GPIOA_BASE, 5),
    (b'A', GPIOA_BASE, 6),
    (b'A', GPIOA_BASE, 7),
    (b'A', GPIOA_BASE, 8),
    (b'A', GPIOA_BASE, 9),
    (b'A', GPIOA_BASE, 10),
    (b'A', GPIOA_BASE, 15),
    (b'B', GPIOB_BASE, 0),
    (b'B', GPIOB_BASE, 1),
    (b'B', GPIOB_BASE, 2),
    (b'B', GPIOB_BASE, 4),
    (b'B', GPIOB_BASE, 5),
    (b'B', GPIOB_BASE, 8),
    (b'B', GPIOB_BASE, 9),
    (b'C', GPIOC_BASE, 13),
];

/// Button pins keep their permanent pull-ups from Buttons::new — do not
/// float them after their window.
const BUTTON_PINS: [(u8, u8); 3] = [(b'B', 4), (b'B', 8), (b'B', 9)];

pub struct Finder {
    idx: usize,
    low: [bool; CANDS.len()],
}

impl Finder {
    pub fn new() -> Self {
        // SAFETY: diagnostic build; idempotent clock enables.
        let crm = unsafe { embassy_at32::pac::at32f403a::CRM::steal() };
        crm.apb2en().modify(|_, w| {
            w.gpioa().bit(true);
            w.gpiob().bit(true);
            w.gpioc().bit(true);
            w
        });
        Self {
            idx: 0,
            low: [false; CANDS.len()],
        }
    }

    /// One ~100 ms pull-up window on the next candidate, densely sampled.
    /// Reports LOW pins once per state change.
    pub async fn poll(&mut self, sender: &mut Sender) {
        let i = self.idx;
        let (port, base, num) = CANDS[i];
        self.idx = (self.idx + 1) % CANDS.len();

        buttons::pull_up(base, num);
        let mut saw_low = false;
        for _ in 0..50 {
            if buttons::read_levels(base) & (1u32 << num) == 0 {
                saw_low = true;
            }
            Timer::after_millis(2).await;
        }
        if !BUTTON_PINS.iter().any(|&(p, n)| p == port && n == num) {
            buttons::floating(base, num);
        }

        if saw_low && !self.low[i] {
            self.low[i] = true;
            let mut line = [0u8; 40];
            let mut w = 0;
            for &b in b"[int] P" {
                line[w] = b;
                w += 1;
            }
            line[w] = port;
            w += 1;
            if num >= 10 {
                line[w] = b'0' + num / 10;
                w += 1;
            }
            line[w] = b'0' + num % 10;
            w += 1;
            for &b in b" LOW\r\n" {
                line[w] = b;
                w += 1;
            }
            sender.push(&line[..w]);
        }
    }
}

//! Button tracer (r59): two-phase discovery of the four buttons
//! (back / left / ok / right) on untraced GPIOs.
//!
//! The r58 passive pass found NO external pull-ups on any candidate pin
//! (everything floats low; only PA15 reads H via its default JTAG internal
//! pull). Buttons wired pin-to-GND are invisible without a pull, so:
//!
//! Phase A (first 15 s): all candidates FLOATING, debounced level changes
//! reported — catches buttons with external pulls or pull-down+VCC wiring.
//!
//! Phase B (forever): each candidate in turn gets an INTERNAL PULL-UP for
//! a 50 ms window (10 ms settle, sample, 40 ms hold, back to floating).
//! One pin at a time, 50 ms pulses — never board-wide (rounds 24/25
//! showed indefinite board-wide pulls heat something and drag VBUS). A
//! floating pin reads H under the pull; a held button shorts it to GND
//! and reads L: reported once per press as `[btn] HIT PxN`. HOLD each
//! button ~3 s so its pin's window comes around (a full cycle takes
//! ~0.8 s).
//!
//! Excluded: PA11/12 (USB), PA13/14 (SWD), PB3-PB9 (BLK/I2C/flash + user
//! exclusion), PB10-15 (LCD SPI2), PC14/15 (crystal), PC0-12 (do not
//! bond out on the LQFP48).

use embassy_time::Timer;

const GPIOA_BASE: usize = 0x4001_0800;
const GPIOB_BASE: usize = 0x4001_0C00;
const GPIOC_BASE: usize = 0x4001_1000;

/// (port letter, base) pairs for the per-port input registers.
const PORTS: [(u8, usize); 3] = [(b'A', GPIOA_BASE), (b'B', GPIOB_BASE), (b'C', GPIOC_BASE)];

/// Candidate pins for the four buttons.
const CANDIDATES: [(u8, usize, u8); 16] = [
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
    (b'C', GPIOC_BASE, 13),
];

/// Phase A length in 20 ms poll ticks (6 s minimum; presses stretch it —
/// each report line blocks the poll loop for 500 ms).
const PASSIVE_TICKS: u32 = 300;

/// Phase B (original candidates) length in 50 ms sweep steps before
/// phase C (the user-excluded pins) starts: 600 steps = 30 s.
const PHASE2_STEPS: u32 = 600;

/// Phase C extra candidates: PB4 (the other freed-JTAG pin, prime
/// suspect after LEFT landed on PA15), PB5, PB8, PB9. The user excluded
/// PB3-PB9 in r58; these get the same harmless 50 ms pull-up pulses and
/// the sweep is announced on the console.
const EXTRA: [(u8, usize, u8); 4] = [
    (b'B', GPIOB_BASE, 4),
    (b'B', GPIOB_BASE, 5),
    (b'B', GPIOB_BASE, 8),
    (b'B', GPIOB_BASE, 9),
];

/// Heartbeat interval: phase A 150 ticks (~3 s), phase B/C 60 steps (~3 s).
/// Distinguishes a hang (COM alive, heartbeats stop) from a fault (COM
/// drops) from a reboot (new banner).

/// Debounce length in phase-A poll ticks (~60 ms).
const DEBOUNCE_TICKS: u8 = 3;

/// Queue slots; the vendor CDC send rejects a second call while one is in
/// flight, so each line drains with a 500 ms gap.
const QUEUE_SLOTS: usize = 8;

unsafe extern "C" {
    fn fnb58_usb_send(data: *const u8, len: u16) -> i32;
}

// --- Production button consumer (r62): the four verified buttons ---

/// (name, GPIO base, pin number).
const BTN_PINS: [(&[u8], usize, u8); 4] = [
    (b"LEFT", GPIOA_BASE, 15),
    (b"OK", GPIOB_BASE, 4),
    (b"BACK", GPIOB_BASE, 9),
    (b"RIGHT", GPIOB_BASE, 8),
];

/// Debounce length in poll calls (the meter loop polls at 250 ms).
const BTN_DEB: u8 = 2;

/// Button indices (BTN_PINS order).
pub const LEFT: usize = 0;
pub const OK: usize = 1;
pub const BACK: usize = 2;
pub const RIGHT: usize = 3;

/// The four verified FNB58 buttons (see README pin map): to-GND switches,
/// no external pulls — internal pull-ups latched here once at init.
pub struct Buttons {
    stable: [bool; 4],
    deb: [u8; 4],
}

impl Buttons {
    /// Enable GPIO clocks and latch internal pull-ups on all four pins.
    pub fn new() -> Self {
        // SAFETY: production consumer; idempotent clock enables alongside
        // the LCD/I2C C glue.
        let crm = unsafe { embassy_at32::pac::at32f403a::CRM::steal() };
        crm.apb2en().modify(|_, w| {
            w.gpioa().bit(true);
            w.gpiob().bit(true);
            w
        });
        for &(_, base, num) in &BTN_PINS {
            pull_up(base, num);
        }
        Self {
            stable: [false; 4],
            deb: [0; 4],
        }
    }

    /// Poll once; returns (button index, down) on a debounced change.
    pub fn poll(&mut self) -> Option<(usize, bool)> {
        for i in 0..BTN_PINS.len() {
            let (_, base, num) = BTN_PINS[i];
            let down = read_levels(base) & (1u32 << num) == 0;
            if down != self.stable[i] {
                self.deb[i] = self.deb[i].saturating_add(1);
                if self.deb[i] >= BTN_DEB {
                    self.deb[i] = 0;
                    self.stable[i] = down;
                    return Some((i, down));
                }
            } else {
                self.deb[i] = 0;
            }
        }
        None
    }

    /// Button name bytes ("LEFT", "OK", ...).
    pub fn name(i: usize) -> &'static [u8] {
        BTN_PINS[i].0
    }
}

fn flat(port: u8, num: u8) -> usize {
    (port - b'A') as usize * 16 + num as usize
}

/// Write one pin's CFGLR/CFGHR nibble (mode 00; cfg: 01 floating,
/// 10 pull up/down with the SCR-latched direction).
fn set_cfg(base: usize, num: u8, nibble: u32) {
    let (reg, shift) = if num < 8 {
        (base as *mut u32, num * 4)
    } else {
        ((base + 4) as *mut u32, (num - 8) * 4)
    };
    // SAFETY: valid GPIO register block, clocks enabled by new().
    unsafe {
        let v = reg.read_volatile();
        reg.write_volatile((v & !(0xFu32 << shift)) | (nibble << shift));
    }
}

/// Latch the ODT pull direction HIGH, then switch the pin to pull mode.
pub(crate) fn pull_up(base: usize, num: u8) {
    let scr = (base + 0x10) as *mut u32;
    // SAFETY: SCR set-bits are write-1-to-set per pin; other bits unaffected.
    unsafe { scr.write_volatile(1u32 << num) };
    set_cfg(base, num, 0b0010);
}

/// Back to floating input (the latched ODT value is ignored in this mode).
pub(crate) fn floating(base: usize, num: u8) {
    set_cfg(base, num, 0b0001);
}

pub(crate) fn read_levels(base: usize) -> u32 {
    // SAFETY: valid GPIO register block, clocks enabled by new().
    unsafe { ((base + 8) as *const u32).read_volatile() }
}

pub struct Tracer {
    last: [u16; 3],
    /// Two debounce counters per flat pin index: [down, up] (96 slots).
    deb: [u8; 96],
    passive_ticks: u32,
    phase2_steps: u32,
    phase3: bool,
    sweep_idx: usize,
    extra_idx: usize,
    /// Last sweep-phase sample per candidate (init true = idle pull-up H).
    window_l: [bool; 16],
    extra_window_l: [bool; 4],
    since_beat: u32,
    beats: u32,
    queue: [[u8; 64]; QUEUE_SLOTS],
    qhead: usize,
    qlen: usize,
    lost: bool,
}

impl Tracer {
    /// Configure every candidate pin as a floating input and enable the
    /// GPIOA/B/C clocks. Levels are captured by [`Self::snapshot`].
    pub fn new() -> Self {
        // SAFETY: bring-up tracer; exclusive CRM/GPIO ownership.
        let crm = unsafe { embassy_at32::pac::at32f403a::CRM::steal() };
        crm.apb2en().modify(|_, w| {
            w.gpioa().bit(true);
            w.gpiob().bit(true);
            w.gpioc().bit(true);
            w
        });
        for &(_, base, num) in &CANDIDATES {
            floating(base, num);
        }
        Self {
            last: [0; 3],
            deb: [0; 96],
            passive_ticks: 0,
            phase2_steps: 0,
            phase3: false,
            sweep_idx: 0,
            extra_idx: 0,
            window_l: [true; 16],
            extra_window_l: [true; 4],
            since_beat: 0,
            beats: 0,
            queue: [[0; 64]; QUEUE_SLOTS],
            qhead: 0,
            qlen: 0,
            lost: false,
        }
    }

    /// Print the idle level of every candidate pin (4 per line) and arm
    /// the phase-A baselines from the settled levels.
    pub async fn snapshot(&mut self) {
        for (i, &(port, base)) in PORTS.iter().enumerate() {
            let levels = read_levels(base);
            self.last[i] = levels as u16;
            let mut cands = [0u8; 16];
            let mut n = 0;
            for &(_, cbase, cnum) in &CANDIDATES {
                if cbase == base {
                    cands[n] = cnum;
                    n += 1;
                }
            }
            let mut line = [0u8; 40];
            for chunk in cands[..n].chunks(4) {
                let mut w = 0;
                for &b in b"[btn] idle P" {
                    line[w] = b;
                    w += 1;
                }
                line[w] = port;
                line[w + 1] = b' ';
                w += 2;
                for &num in chunk {
                    line[w] = b'0' + num / 10;
                    line[w + 1] = b'0' + num % 10;
                    line[w + 2] = b'=';
                    line[w + 3] = if levels & (1 << num) != 0 { b'H' } else { b'L' };
                    line[w + 4] = b' ';
                    w += 5;
                }
                line[w] = b'\r';
                line[w + 1] = b'\n';
                // SAFETY: single-threaded; CDC send with 500 ms gap.
                unsafe { fnb58_usb_send(line.as_ptr(), (w + 2) as u16) };
                Timer::after_millis(500).await;
            }
        }
    }

    /// One step. Phase A: debounced floating-mode change detection.
    /// Phase B/C: arm one candidate with an internal pull-up per call.
    /// Heartbeats every ~3 s throughout.
    pub async fn poll(&mut self) {
        if self.passive_ticks < PASSIVE_TICKS {
            self.passive_scan();
            self.passive_ticks += 1;
            self.since_beat += 1;
            if self.passive_ticks == PASSIVE_TICKS {
                self.push(b"[btn] passive phase over; pull-up sweep starts\r\n");
                self.push(b"[btn] HOLD each button ~4 s, one at a time\r\n");
            }
            if self.since_beat >= 150 {
                self.heartbeat();
            }
        } else if !self.phase3 {
            self.sweep_step(false).await;
            self.phase2_steps += 1;
            self.since_beat += 1;
            if self.phase2_steps == PHASE2_STEPS {
                self.phase3 = true;
                self.push(b"[btn] phase 3: also sweeping PB4/5/8/9 (excluded\r\n");
                self.push(b"[btn] before - 50 ms pull-up pulses, harmless)\r\n");
            }
            if self.since_beat >= 60 {
                self.heartbeat();
            }
        } else {
            self.sweep_step(true).await;
            self.since_beat += 1;
            if self.since_beat >= 60 {
                self.heartbeat();
            }
        }
        self.drain().await;
    }

    fn heartbeat(&mut self) {
        self.since_beat = 0;
        self.beats = self.beats.wrapping_add(1);
        let mut line = [0u8; 40];
        let mut w = 0;
        for &b in b"[btn] alive " {
            line[w] = b;
            w += 1;
        }
        // Decimal beat counter.
        let mut digits = [0u8; 10];
        let mut n = 0;
        let mut v = self.beats;
        if v == 0 {
            digits[0] = b'0';
            n = 1;
        }
        while v > 0 {
            digits[n] = b'0' + (v % 10) as u8;
            v /= 10;
            n += 1;
        }
        while n > 0 {
            n -= 1;
            line[w] = digits[n];
            w += 1;
        }
        line[w] = b'\r';
        line[w + 1] = b'\n';
        self.push(&line[..w + 2]);
    }

    /// Phase A: floating inputs, debounced prev->new change reports.
    fn passive_scan(&mut self) {
        for (i, &(port, base)) in PORTS.iter().enumerate() {
            let levels = read_levels(base) as u16;
            let changed = self.last[i] ^ levels;
            if changed == 0 {
                continue;
            }
            for &(_, cbase, cnum) in &CANDIDATES {
                if cbase != base {
                    continue;
                }
                let bit = 1u16 << cnum;
                if changed & bit == 0 {
                    continue;
                }
                let f = flat(port, cnum);
                let now_down = levels & bit == 0;
                let (deb_idx, other_idx) = if now_down {
                    (f * 2, f * 2 + 1)
                } else {
                    (f * 2 + 1, f * 2)
                };
                self.deb[deb_idx] = self.deb[deb_idx].saturating_add(1);
                self.deb[other_idx] = 0;
                if self.deb[deb_idx] >= DEBOUNCE_TICKS {
                    self.deb[deb_idx] = 0;
                    self.last[i] ^= bit;
                    let prev = if now_down { b'H' } else { b'L' };
                    let now = if now_down { b'L' } else { b'H' };
                    let mut line = [0u8; 40];
                    let mut w = 0;
                    for &b in b"[btn] P" {
                        line[w] = b;
                        w += 1;
                    }
                    line[w] = port;
                    w += 1;
                    if cnum >= 10 {
                        line[w] = b'0' + cnum / 10;
                        w += 1;
                    }
                    line[w] = b'0' + cnum % 10;
                    w += 1;
                    line[w] = b' ';
                    line[w + 1] = prev;
                    line[w + 2] = b'-';
                    line[w + 3] = b'>';
                    line[w + 4] = now;
                    w += 5;
                    line[w] = b'\r';
                    line[w + 1] = b'\n';
                    self.push(&line[..w + 2]);
                }
            }
        }
    }

    /// Phase B/C: give the next candidate a 50 ms internal pull-up window
    /// (10 ms settle, sample, 40 ms hold, back to floating). `extra`
    /// selects the phase-C list (PB4/5/8/9).
    async fn sweep_step(&mut self, extra: bool) {
        let (i, port, base, num, prev) = if extra {
            let j = self.extra_idx;
            let (p, b, n) = EXTRA[j];
            self.extra_idx = (self.extra_idx + 1) % EXTRA.len();
            (j, p, b, n, self.extra_window_l[j])
        } else {
            let j = self.sweep_idx;
            let (p, b, n) = CANDIDATES[j];
            self.sweep_idx = (self.sweep_idx + 1) % CANDIDATES.len();
            (j, p, b, n, self.window_l[j])
        };
        pull_up(base, num);
        Timer::after_millis(10).await;
        let l = read_levels(base) & (1u32 << num) == 0;
        Timer::after_millis(40).await;
        floating(base, num);
        if l != prev {
            if l {
                let mut line = [0u8; 40];
                let mut w = 0;
                for &b in b"[btn] HIT P" {
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
                for &b in b" (L w/pull)\r\n" {
                    line[w] = b;
                    w += 1;
                }
                self.push(&line[..w]);
            }
            if extra {
                self.extra_window_l[i] = l;
            } else {
                self.window_l[i] = l;
            }
        }
    }

    fn push(&mut self, line: &[u8]) {
        if self.qlen == QUEUE_SLOTS {
            self.lost = true;
            return;
        }
        let slot = (self.qhead + self.qlen) % QUEUE_SLOTS;
        self.queue[slot] = [0; 64];
        self.queue[slot][..line.len()].copy_from_slice(line);
        self.qlen += 1;
    }

    async fn drain(&mut self) {
        while self.qlen > 0 {
            let line = self.queue[self.qhead];
            let len = line.iter().position(|&b| b == 0).unwrap_or(line.len());
            // SAFETY: single-threaded; one send per 500 ms gap.
            unsafe { fnb58_usb_send(line.as_ptr(), len as u16) };
            Timer::after_millis(500).await;
            self.qhead = (self.qhead + 1) % QUEUE_SLOTS;
            self.qlen -= 1;
        }
        if self.lost {
            self.lost = false;
            // SAFETY: single-threaded; CDC send with 500 ms gap.
            unsafe { fnb58_usb_send(b"[btn] events lost\r\n".as_ptr(), 19) };
            Timer::after_millis(500).await;
        }
    }
}

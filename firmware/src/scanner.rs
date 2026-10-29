//! I2C discovery scanner over the vendor C I2C driver (I2C1 on PB6/PB7 —
//! SCL pin 42, user-traced). Probes addresses, identifies known devices
//! by ID registers, and queues report lines for the CDC.

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_at32::pac::at32f403a::interrupt;

unsafe extern "C" {
    fn fnb58_i2c_init();
    fn fnb58_i2c_probe(address: u16) -> u32;
    fn fnb58_i2c_read(address: u16, reg: u8, out: *mut u8, len: u16) -> u32;
}

/// Ring of queued report lines (the scanner queues up to ~5 lines per
/// cycle while the main loop can't drain; a single slot lost them).
const RING_SLOTS: usize = 8;
static mut RING_STORE: [[u8; 64]; RING_SLOTS] = [[0; 64]; RING_SLOTS];
static RING_LEN: core::sync::atomic::AtomicUsize = core::sync::atomic::AtomicUsize::new(0);
static RING_HEAD: core::sync::atomic::AtomicUsize = core::sync::atomic::AtomicUsize::new(0);

fn queue_line(text: &[u8]) {
    // SAFETY: single-threaded scanner and drain (main task only).
    let ring = &raw mut RING_STORE;
    let len = RING_LEN.load(Ordering::Acquire);
    if len >= RING_SLOTS {
        return; // full: drop (bring-up semantics)
    }
    let head = RING_HEAD.load(Ordering::Acquire);
    let slot = unsafe { &mut (*ring)[head] };
    let n = text.len().min(63);
    *slot = [0; 64];
    slot[..n].copy_from_slice(&text[..n]);
    RING_HEAD.store((head + 1) % RING_SLOTS, Ordering::Release);
    RING_LEN.store(len + 1, Ordering::Release);
}

/// Take the queued report line, if any.
pub fn take_line() -> Option<&'static [u8]> {
    let len = RING_LEN.load(Ordering::Acquire);
    if len == 0 {
        return None;
    }
    // SAFETY: single-threaded scanner and drain (main task only).
    let ring = &raw mut RING_STORE;
    let tail = (RING_HEAD.load(Ordering::Acquire) + RING_SLOTS - len) % RING_SLOTS;
    let line = unsafe { &(*ring)[tail] };
    let end = line.iter().position(|&b| b == 0).unwrap_or(64);
    let out: &'static [u8] = unsafe { core::mem::transmute(&line[..end]) };
    RING_LEN.store(len - 1, Ordering::Release);
    Some(out)
}

/// Fixed-capacity scratch line buffer.
mod heapless_line {
    pub struct Line {
        buf: [u8; 64],
        len: usize,
    }

    impl Line {
        pub const fn new() -> Self {
            Self {
                buf: [0; 64],
                len: 0,
            }
        }

        pub fn push(&mut self, bytes: &[u8]) {
            for &b in bytes {
                if self.len < self.buf.len() - 1 {
                    self.buf[self.len] = b;
                    self.len += 1;
                }
            }
        }

        pub fn push_hex(&mut self, value: u8) {
            const HEX: &[u8; 16] = b"0123456789ABCDEF";
            self.push(&[HEX[(value >> 4) as usize], HEX[(value & 0xF) as usize]]);
        }

        /// The text as a NUL-padded fixed buffer for the queue.
        pub fn bytes(&self) -> &[u8; 64] {
            &self.buf
        }

        /// The last pushed text.
        pub fn peak(&self) -> &[u8] {
            &self.buf[..self.len]
        }
    }
}

/// Run one full scan cycle on the traced bus (I2C1, PB6/PB7).
///
/// Address convention (decoded from the round-18 scan): the vendor
/// i2c_application library takes the 8-bit form — the byte on the wire is
/// `X | rw` — so a device at 7-bit address `A` ACKs probe `X = A<<1` and
/// `X+1`. The scan reports real 7-bit addresses (X/2).
pub async fn run() {
    // One-time vendor I2C init (I2C1, PB6/PB7, 100 kHz).
    unsafe {
        fnb58_i2c_init();
    }

    let mut line = heapless_line::Line::new();
    line.push(b"I2C1 PB6/PB7:");
    let mut found = 0usize;
    let mut devices: [u8; 12] = [0; 12];
    let mut previous_acked = false;
    for x in 0x08u8..=0xF0 {
        let acked = unsafe { fnb58_i2c_probe(x as u16) } == 0;
        if acked {
            if !previous_acked {
                // Rising edge of a pair: the device's 7-bit address is x/2.
                if found < devices.len() {
                    devices[found] = x >> 1;
                }
                found += 1;
                line.push(b" ");
                line.push_hex(x >> 1);
            }
        }
        previous_acked = acked;
    }
    if found == 0 {
        line.push(b" none");
    }
    queue_line(line.bytes());

    // Identify + measure every discovered device.
    for i in 0..found.min(devices.len()) {
        let a = devices[i];
        let x = u16::from(a) << 1; // vendor 8-bit form
        let mut line2 = heapless_line::Line::new();
        line2.push(b"dev ");
        line2.push_hex(a);
        line2.push(b":");
        let mut buf = [0u8; 2];

        // Raw register dump for every discovered device - the identification
        // gate is gone: print what's actually there.
        if (0x40..=0x4F).contains(&a) {
            // TPA626/INA226-class: mfr 0xFE, die 0xFF, config 0x00, vbus 0x02.
            for (reg, name) in [(0xFEu8, b" mfr="), (0xFFu8, b" die="), (0x00u8, b" cfg=")] {
                let mut b2 = [0u8; 2];
                let st = unsafe { fnb58_i2c_read(u16::from(x), reg, b2.as_mut_ptr(), 2) };
                if st == 0 {
                    line2.push(name);
                    line2.push_hex(b2[0]);
                    line2.push_hex(b2[1]);
                }
            }
            let mut vb = [0u8; 2];
            let st = unsafe { fnb58_i2c_read(u16::from(x), 0x02, vb.as_mut_ptr(), 2) };
            if st == 0 {
                let raw = (u16::from(vb[0]) << 8) | u16::from(vb[1]);
                let mv = (raw as u32 * 125 / 100) as u16; // 1.25 mV/LSB
                line2.push(b" vbus=");
                if mv >= 10000 {
                    line2.push(&[b'0' + (mv / 10000) as u8]);
                }
                line2.push(&[b'0' + (mv / 1000 % 10) as u8, b'.']);
                line2.push(&[b'0' + (mv / 100 % 10) as u8, b'0' + (mv / 10 % 10) as u8]);
                line2.push(b"V");
            }
            queue_line(line2.bytes());
        }

        if matches!(a, 0x22 | 0x23) {
            // FUSB302: DeviceID 0x01.
            let mut b1 = [0u8; 1];
            let st = unsafe { fnb58_i2c_read(u16::from(x), 0x01, b1.as_mut_ptr(), 1) };
            if st == 0 {
                line2.push(b" FUSB302 id=");
                line2.push_hex(b1[0]);
            } else {
                line2.push(b" FUSB302 rd-err");
            }
            queue_line(line2.bytes());
        }

        if matches!(a, 0x0E | 0x0F) {
            // KXTJ3: WHO_AM_I 0x0F.
            let mut b1 = [0u8; 1];
            let st = unsafe { fnb58_i2c_read(u16::from(x), 0x0F, b1.as_mut_ptr(), 1) };
            if st == 0 {
                line2.push(b" KXTJ3 whoami=");
                line2.push_hex(b1[0]);
            } else {
                line2.push(b" KXTJ3 rd-err");
            }
            queue_line(line2.bytes());
        }

        if a == 0x0C {
            // Mystery device: dump candidate ID registers raw.
            for reg in [0x00u8, 0x07u8, 0x0Du8, 0x0Fu8, 0x32u8] {
                let mut b1 = [0u8; 1];
                let st = unsafe { fnb58_i2c_read(u16::from(x), reg, b1.as_mut_ptr(), 1) };
                if st == 0 && b1[0] != 0 {
                    line2.push(b" r");
                    line2.push_hex(reg);
                    line2.push(b"=");
                    line2.push_hex(b1[0]);
                }
            }
            queue_line(line2.bytes());
        }
    }
}

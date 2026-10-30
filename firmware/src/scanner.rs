//! I2C discovery scanner over the vendor C I2C driver (I2C1 on PB6/PB7 —
//! SCL pin 42, user-traced). Probes addresses, identifies known devices
//! by ID registers, and queues report lines for the CDC.

use core::sync::atomic::Ordering;


unsafe extern "C" {
    fn fnb58_i2c_init();
    fn fnb58_i2c_probe(address: u16) -> u32;
    fn fnb58_i2c_read(address: u16, reg: u8, out: *mut u8, len: u16) -> u32;
    fn fnb58_display_probe_bus(bus_idx: i32, results: *mut u8);
    fn fnb58_display_bus_name(bus_idx: i32) -> *const u8;
    fn fnb58_display_num_buses() -> i32;
}

fn queue_line(text: &[u8]) {
    // Direct send: the CDC path is proven; no queue in between.
    crate::send_line(text);
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

/// The last measurement's length (set by `measure`).
static MEASURE_LEN: core::sync::atomic::AtomicUsize =
    core::sync::atomic::AtomicUsize::new(0);
static mut MEASURE_STORE: [u8; 64] = [0; 64];

/// Read the TPA626 (7-bit 0x40) and format one measurement line:
/// `V=5.04V I=+123uA(raw)` — current is shunt-voltage raw until the board
/// shunt resistance is confirmed for the CAL register.
pub fn measure(out: &mut [u8], tick: u32) {
    // SAFETY: main-task only.
    let store = unsafe { &mut *core::ptr::addr_of_mut!(MEASURE_STORE) };
    let mut line = heapless_line::Line::new();
    const TPA_X: u16 = 0x80; // 7-bit 0x40 in the vendor 8-bit form

    let mut vb = [0u8; 2];
    if unsafe { fnb58_i2c_read(TPA_X, 0x02, vb.as_mut_ptr(), 2) } == 0 {
        let raw = (u16::from(vb[0]) << 8) | u16::from(vb[1]);
        let mv = (raw as u32 * 125 / 100) as u16;
        line.push(b"V=");
        if mv >= 10000 {
            line.push(&[b'0' + (mv / 10000) as u8]);
        }
        line.push(&[b'0' + (mv / 1000 % 10) as u8, b'.']);
        line.push(&[b'0' + (mv / 100 % 10) as u8, b'0' + (mv / 10 % 10) as u8]);
        line.push(b"V");
    } else {
        line.push(b"V=--");
    }

    // Shunt voltage, 2.5 uV/LSB, signed 16-bit. Calibration pending:
    // the board's shunt resistance is unconfirmed (~20 mOhm class).
    let mut sh = [0u8; 2];
    if unsafe { fnb58_i2c_read(TPA_X, 0x04, sh.as_mut_ptr(), 2) } == 0 {
        let raw = (u16::from(sh[0]) << 8) | u16::from(sh[1]);
        let signed = raw as i16;
        line.push(b" I=");
        if signed < 0 {
            line.push(b"-");
        }
        let uv = signed.unsigned_abs() as u32;
        // 6 fixed digits of microvolts.
        for digit in [
            (uv / 100_000 % 10) as u8,
            (uv / 10_000 % 10) as u8,
            (uv / 1_000 % 10) as u8,
            (uv / 100 % 10) as u8,
            (uv / 10 % 10) as u8,
            (uv % 10) as u8,
        ] {
            line.push(&[b'0' + digit]);
        }
        line.push(b"uV");
    } else {
        line.push(b" I=--");
    }
    line.push(b" #");
    if tick == 0 {
        line.push(b"0");
    } else {
        let mut digits = [0u8; 10];
        let mut n = 0;
        let mut v = tick;
        while v > 0 {
            digits[n] = b'0' + (v % 10) as u8;
            v /= 10;
            n += 1;
        }
        while n > 0 {
            n -= 1;
            line.push(&[digits[n]]);
        }
    }
    let n = line.peak().len();
    out[..n].copy_from_slice(&line.peak()[..n]);
    MEASURE_LEN.store(n, Ordering::Release);
}

/// Length of the last formatted measurement.
pub fn last_measure_len() -> usize {
    MEASURE_LEN.load(Ordering::Acquire)
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
            // SMBus Alert Response Address: the TPA626's ALERT is asserted
            // (conversion-ready in continuous mode) and the chip answers
            // ARA reads. Not a separate device.
            let mut line3 = heapless_line::Line::new();
            line3.push(b"0C=ARA (TPA626 alert)");
            queue_line(line3.bytes());
        }
    }
}

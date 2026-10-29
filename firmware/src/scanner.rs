//! I2C discovery scanner: probes every plausible I2C instance/pin pairing
//! on the FNB58's LQFP48 (the wiring is not yet traced), identifies known
//! devices by their ID registers, and queues report lines for the CDC.
//!
//! Candidates (AT32F403A LQFP48 — I2C3 needs PC9, absent on 48 pins):
//! - I2C1 on PB6/PB7 (default mux)
//! - I2C1 on PB8/PB9 (IOMUX I2C1_MUX = 1)
//! - I2C2 on PB10/PB11
//!
//! Known devices and their fingerprints:
//! - TPA626 power monitor (INA226 register map): address 0x40..=0x4F,
//!   reg 0xFE = 0x5449 ("TI"), reg 0xFF = 0x2260.
//! - FUSB302B PD controller: address 0x22/0x23, reg 0x01 in 0x80..=0x9F.
//! - KXTJ3-1057 accelerometer: address 0x0E/0x0F, reg 0x0F = 0x35.

use embassy_at32::gpio::{Drive, Port};
use embassy_at32::i2c::{I2c, Speed};
use core::sync::atomic::{AtomicBool, Ordering};

use embedded_hal_async::i2c::{I2c as _, Operation};

/// One queued report line at a time (the scanner runs between main-loop
/// iterations, so a single slot never overflows).
static LINE: core::sync::atomic::AtomicPtr<[u8; 64]> =
    core::sync::atomic::AtomicPtr::new(core::ptr::null_mut());
static LINE_READY: AtomicBool = AtomicBool::new(false);
static mut LINE_STORE: [u8; 64] = [0; 64];

fn queue_line(text: &[u8]) {
    // SAFETY: single-threaded scanner and drain (main task only).
    let store = &raw mut LINE_STORE;
    let line = unsafe { &mut *store };
    let n = text.len().min(63);
    *line = [0; 64];
    line[..n].copy_from_slice(&text[..n]);
    LINE.store(line as *mut [u8; 64], Ordering::Release);
    LINE_READY.store(true, Ordering::Release);
}

/// Take the queued report line, if any.
pub fn take_line() -> Option<&'static [u8]> {
    if LINE_READY.swap(false, Ordering::AcqRel) {
        // SAFETY: the line was stored by queue_line in this task's context.
        let ptr = LINE.load(Ordering::Acquire);
        Some(unsafe { &*ptr })
    } else {
        None
    }
}

/// One probe bus candidate.
struct Candidate {
    name: &'static str,
    instance: Instance,
    scl: (Port, u8),
    sda: (Port, u8),
    remap_i2c1: bool,
}

#[derive(Clone, Copy, PartialEq)]
enum Instance {
    I2c1,
    I2c2,
}

const CANDIDATES: [Candidate; 3] = [
    Candidate {
        name: "I2C1 PB6/PB7",
        instance: Instance::I2c1,
        scl: (Port::B, 6),
        sda: (Port::B, 7),
        remap_i2c1: false,
    },
    Candidate {
        name: "I2C1 PB8/PB9",
        instance: Instance::I2c1,
        scl: (Port::B, 8),
        sda: (Port::B, 9),
        remap_i2c1: true,
    },
    Candidate {
        name: "I2C2 PB10/PB11",
        instance: Instance::I2c2,
        scl: (Port::B, 10),
        sda: (Port::B, 11),
        remap_i2c1: false,
    },
];

/// Run one full scan cycle: every candidate bus, every address 0x08..=0x78,
/// then identify the ACKed devices. One report line per candidate.
pub async fn run() {
    scan_pullups();
    let pclk1 = 96_000_000; // APB1 under the 192 MHz crystal clock profile.
    for candidate in CANDIDATES {
        let mut report = heapless_line::Line::new();
        report.push(candidate.name.as_bytes());
        report.push(b": ");

        // Pin mux: open-drain alternate function, then the bus.
        // SAFETY: bring-up scanner, exclusive ownership of these pins.
        unsafe {
            let _ = embassy_at32::gpio::AlternatePin::new_open_drain(
                candidate.scl.0,
                candidate.scl.1,
                Drive::Moderate,
            );
            let _ = embassy_at32::gpio::AlternatePin::new_open_drain(
                candidate.sda.0,
                candidate.sda.1,
                Drive::Moderate,
            );
        }
        set_i2c1_mux(candidate.remap_i2c1);

        let bus = match candidate.instance {
            Instance::I2c1 => unsafe { I2c::new_i2c1(pclk1, Speed::Standard) },
            Instance::I2c2 => unsafe { I2c::new_i2c2(pclk1, Speed::Standard) },
        };
        let Ok(mut bus) = bus else {
            report.push(b"init failed");
            queue_line(report.bytes());
            continue;
        };

        let mut acked: [u8; 16] = [0; 16];
        let mut ack_count = 0usize;
        for address in 0x08u8..=0x78 {
            let result = bus
                .transaction(address, &mut [Operation::Write(&[])])
                .await;
            match result {
                Ok(()) => {
                    if ack_count < acked.len() {
                        acked[ack_count] = address;
                    }
                    ack_count += 1;
                }
                Err(embassy_at32::i2c::Error::NoAcknowledge(_)) => {}
                Err(_) => {
                    // Timeout or bus error: note once, keep scanning.
                    if report.peak() == b": " {
                        report.push(b"(bus errors)");
                    }
                }
            }
        }

        if ack_count == 0 {
            report.push(b"no devices");
        } else {
            report.push(b"ACK");
            for i in 0..ack_count.min(acked.len()) {
                report.push_hex(acked[i]);
                report.push(b",");
            }
            // Identify known devices among the ACKed addresses.
            for i in 0..ack_count.min(acked.len()) {
                let address = acked[i];
                if (0x40..=0x4F).contains(&address) {
                    if let Some(id) = read_tpa626(&mut bus, address).await {
                        report.push(b" TPA626@");
                        report.push_hex(address);
                        report.push(b"=");
                        report.push_hex((id >> 8) as u8);
                        report.push_hex(id as u8);
                    }
                }
                if matches!(address, 0x22 | 0x23) {
                    if let Some(id) = read_fusb302(&mut bus, address).await {
                        report.push(b" FUSB302@");
                        report.push_hex(address);
                        report.push(b"=");
                        report.push_hex(id);
                    }
                }
                if matches!(address, 0x0E | 0x0F) {
                    if let Some(id) = read_kxtj3(&mut bus, address).await {
                        report.push(b" KXTJ3@");
                        report.push_hex(address);
                        report.push(b"=");
                        report.push_hex(id);
                    }
                }
            }
        }
        queue_line(report.bytes());
    }
}

async fn read_tpa626(bus: &mut I2c, address: u8) -> Option<u16> {
    let mut buf = [0u8; 2];
    bus.transaction(
        address,
        &mut [Operation::Write(&[0xFE]), Operation::Read(&mut buf)],
    )
    .await
    .ok()?;
    if buf != [0x54, 0x49] {
        return None; // manufacturer ID "TI"
    }
    let mut die = [0u8; 2];
    bus.transaction(
        address,
        &mut [Operation::Write(&[0xFF]), Operation::Read(&mut die)],
    )
    .await
    .ok()?;
    Some((u16::from(die[0]) << 8) | u16::from(die[1]))
}

async fn read_fusb302(bus: &mut I2c, address: u8) -> Option<u8> {
    let mut buf = [0u8; 1];
    bus.transaction(
        address,
        &mut [Operation::Write(&[0x01]), Operation::Read(&mut buf)],
    )
    .await
    .ok()?;
    let id = buf[0];
    (0x80..=0x9F).contains(&id).then_some(id)
}

async fn read_kxtj3(bus: &mut I2c, address: u8) -> Option<u8> {
    let mut buf = [0u8; 1];
    bus.transaction(
        address,
        &mut [Operation::Write(&[0x0F]), Operation::Read(&mut buf)],
    )
    .await
    .ok()?;
    (buf[0] == 0x35).then_some(buf[0])
}

// --- Raw GPIO access (register-level, no HAL pin-claim contract: the
// probe re-runs every cycle on the same pins). ---

const GPIOA_BASE: usize = 0x4001_0800;
const GPIOB_BASE: usize = 0x4001_0C00;
const GPIOC_BASE: usize = 0x4001_1000;

fn gpio_base(port: Port) -> usize {
    match port {
        Port::A => GPIOA_BASE,
        Port::B => GPIOB_BASE,
        Port::C => GPIOC_BASE,
        _ => GPIOC_BASE,
    }
}

/// Configure one pin as input with the pull latch at `low` (pulldown),
/// read it, and restore its previous configuration. Returns the level
/// read while pulled down: 1 means an external pull-up is present.
unsafe fn probe_pullup(port: Port, num: u8) -> Option<bool> {
    if num > 15 {
        return None;
    }
    let base = gpio_base(port);
    let cfglr = (base) as *mut u32;
    let cfghr = (base + 4) as *mut u32;
    let idt = (base + 8) as *mut u32;
    let clr = (base + 0x14) as *mut u32;
    let scr = (base + 0x10) as *mut u32;

    let (reg, shift) = if num < 8 {
        (cfglr, num * 4)
    } else {
        (cfghr, (num - 8) * 4)
    };
    // Save the previous nibble.
    let saved = (reg.read_volatile() >> shift) & 0xF;
    // Input + pull (mode 00, cfg 10).
    let value = (reg.read_volatile() & !(0xF << shift)) | (0b10 << shift);
    reg.write_volatile(value);
    // Latch the pull direction low (pulldown) via CLR (write-1-to-clear
    // the ODT latch) and give the line a moment to settle.
    clr.write_volatile(1 << num);
    let _ = scr.read_volatile();
    for _ in 0..200 {
        core::hint::spin_loop();
    }
    let level = (idt.read_volatile() >> num) & 1 == 1;
    // Restore the previous configuration.
    let value = (reg.read_volatile() & !(0xF << shift)) | (saved << shift);
    reg.write_volatile(value);
    Some(level)
}

/// Pins skipped in the pull-up probe: USB data (PA11/PA12), SWD
/// (PA13/PA14), crystal pins (PD0/PD1, PC14/PC15 - PC14/15 could be
/// OSC32 or GPIO; leaving them out is the safe choice).
const PROBE_SKIP: [(&Port, u8); 5] = [
    (&Port::A, 11),
    (&Port::A, 12),
    (&Port::A, 13),
    (&Port::A, 14),
    (&Port::C, 14),
];

/// Scan every probe-safe pin: report those reading HIGH with the
/// internal pull-down enabled (= external pull-up present). Lines are
/// emitted per port.
fn scan_pullups() {
    let ports = [(Port::A, "PA", 16u8), (Port::B, "PB", 16), (Port::C, "PC", 13)];
    for (port, name, count) in ports {
        let mut line = heapless_line::Line::new();
        line.push(b"[pu] ");
        line.push(name.as_bytes());
        line.push(b":");
        let mut found = 0;
        for num in 0..count {
            if PROBE_SKIP.iter().any(|(p, n)| **p == port && *n == num) {
                continue;
            }
            // SAFETY: forwarded from the scanner's bring-up contract.
            if let Some(true) = (unsafe { probe_pullup(port, num) }) {
                line.push(b" ");
                line.push_num(num);
                found += 1;
            }
        }
        if found == 0 {
            line.push(b" none");
        }
        queue_line(line.bytes());
    }
}

/// Read one pin's current input level as input-floating.
unsafe fn read_pin(port: Port, num: u8) -> Option<bool> {
    let base = gpio_base(port);
    let idt = (base + 8) as *mut u32;
    Some(((idt.read_volatile() >> num) & 1) == 1)
}

fn set_i2c1_mux(remapped: bool) {
    // IOMUX needs the APB2 clock (enabled here unconditionally; cheap).
    // SAFETY: bring-up scanner, exclusive CRM/IOMUX ownership.
    let crm = unsafe { embassy_at32::pac::at32f403a::CRM::steal() };
    let iomux = unsafe { embassy_at32::pac::at32f403a::IOMUX::steal() };
    crm.apb2en().modify(|_, w| w.iomux().bit(true));
    iomux.remap().modify(|_, w| w.i2c1_mux().bit(remapped));
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

        pub fn push_num(&mut self, value: u8) {
            // Decimal.
            if value == 0 {
                self.push(b"0");
                return;
            }
            let mut digits = [0u8; 3];
            let mut n = 0;
            let mut v = value;
            while v > 0 {
                digits[n] = b'0' + v % 10;
                v /= 10;
                n += 1;
            }
            while n > 0 {
                n -= 1;
                self.push(&[digits[n]]);
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

        /// The last pushed text (used to detect "already noted" state).
        pub fn peak(&self) -> &[u8] {
            &self.buf[..self.len]
        }
    }
}

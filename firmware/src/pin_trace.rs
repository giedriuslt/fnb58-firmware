//! Pin tracer: walks through the GPIO pins on physical LQFP48 pins 1-24
//! of the AT32F403ACGT7, sets each HIGH for 10 seconds, and prints the
//! physical pin number + GPIO name to serial. The user probes display
//! connector wires with a multimeter.
//!
//! AT32F403ACGT7 LQFP48 physical pins 1-24 (STM32F103C8T6-compatible):
//!   1  VDD          (power - skip)
//!   2  PC13         (GPIO - trace)
//!   3  PC14         (OSC32 - skip)
//!   4  PC15         (OSC32 - skip)
//!   5  PD0          (OSC - skip)
//!   6  PD1          (OSC - skip)
//!   7  NRST         (reset - skip)
//!   8  VSS          (power - skip)
//!   9  VDD          (power - skip)
//!  10  VDDA         (power - skip)
//!  11  VSSA         (power - skip)
//!  12  PA0          (GPIO - trace)
//!  13  PA1          (GPIO - trace)
//!  14  PA2          (GPIO - trace)
//!  15  PA3          (GPIO - trace)
//!  16  PA4          (GPIO - trace)
//!  17  PA5          (GPIO/SPI1_SCK - trace)
//!  18  PA6          (GPIO/SPI1_MISO - trace)
//!  19  PA7          (GPIO/SPI1_MOSI - trace)
//!  20  PB0          (GPIO - trace)
//!  21  PB1          (GPIO - trace)
//!  22  PB2/BOOT1    (GPIO - trace)
//!  23  PB10         (GPIO - trace)
//!  24  PB11         (GPIO - trace)

const GPIOA_BASE: usize = 0x4001_0800;
const GPIOB_BASE: usize = 0x4001_0C00;
const GPIOC_BASE: usize = 0x4001_1000;

/// A traceable GPIO pin: (physical pin, name, base, num, note).
struct TracePin {
    phys: u8,
    name: &'static str,
    base: usize,
    num: u8,
    note: &'static str,
}

const TRACE_PINS: [TracePin; 14] = [
    TracePin { phys: 2, name: "PC13", base: GPIOC_BASE, num: 13, note: "" },
    TracePin { phys: 12, name: "PA0", base: GPIOA_BASE, num: 0, note: "" },
    TracePin { phys: 13, name: "PA1", base: GPIOA_BASE, num: 1, note: "" },
    TracePin { phys: 14, name: "PA2", base: GPIOA_BASE, num: 2, note: "" },
    TracePin { phys: 15, name: "PA3", base: GPIOA_BASE, num: 3, note: "" },
    TracePin { phys: 16, name: "PA4", base: GPIOA_BASE, num: 4, note: "" },
    TracePin { phys: 17, name: "PA5", base: GPIOA_BASE, num: 5, note: "SPI1_SCK" },
    TracePin { phys: 18, name: "PA6", base: GPIOA_BASE, num: 6, note: "SPI1_MISO" },
    TracePin { phys: 19, name: "PA7", base: GPIOA_BASE, num: 7, note: "SPI1_MOSI" },
    TracePin { phys: 20, name: "PB0", base: GPIOB_BASE, num: 0, note: "" },
    TracePin { phys: 21, name: "PB1", base: GPIOB_BASE, num: 1, note: "" },
    TracePin { phys: 22, name: "PB2", base: GPIOB_BASE, num: 2, note: "BOOT1" },
    TracePin { phys: 23, name: "PB10", base: GPIOB_BASE, num: 10, note: "" },
    TracePin { phys: 24, name: "PB11", base: GPIOB_BASE, num: 11, note: "" },
];

/// Configure one pin as output push-pull (2 MHz).
unsafe fn pin_output(base: usize, num: u8) {
    let reg = if num < 8 { base } else { base + 4 };
    let shift = (num % 8) * 4;
    // CNF=00 (push-pull), MODE=10 (2 MHz) -> 0b0010
    let val = ((reg as *mut u32).read_volatile() & !(0xF << shift)) | (0b0010 << shift);
    (reg as *mut u32).write_volatile(val);
}

/// Set one pin output HIGH via BSRR.
unsafe fn pin_high(base: usize, num: u8) {
    ((base + 0x10) as *mut u32).write_volatile(1 << num);
}

/// Set one pin output LOW via BSRR.
unsafe fn pin_low(base: usize, num: u8) {
    ((base + 0x10) as *mut u32).write_volatile(1 << (num + 16));
}

/// Restore one pin to floating input (reset state).
unsafe fn pin_input(base: usize, num: u8) {
    let reg = if num < 8 { base } else { base + 4 };
    let shift = (num % 8) * 4;
    // CNF=01 (floating), MODE=00 (input) -> 0b0100
    let val = ((reg as *mut u32).read_volatile() & !(0xF << shift)) | (0b0100 << shift);
    (reg as *mut u32).write_volatile(val);
}

static mut DELAY_DUMMY: u32 = 0;

/// Busy-wait ~10 seconds. Uses a volatile write that the compiler cannot
/// optimize away (opt-level=z + LTO eliminates bare nop loops entirely).
/// At ~192 MHz HCLK: 384 outer x 1M inner = 384M iterations,
/// ~5 cycles each (volatile store + loop overhead) = 1.92G cycles = 10s.
fn delay_10s() {
    for _ in 0..384 {
        for _ in 0..1_000_000 {
            // SAFETY: static mut, single-threaded bring-up.
            unsafe {
                core::ptr::write_volatile(core::ptr::addr_of_mut!(DELAY_DUMMY), 0);
            }
        }
    }
}

/// Send a line via CDC.
fn send(text: &[u8]) {
    crate::send_line(text);
}

/// Run the full pin trace sweep for physical pins 1-24.
pub fn sweep() {
    // Enable GPIOA/B/C clocks.
    // SAFETY: exclusive CRM ownership.
    let crm = unsafe { embassy_at32::pac::at32f403a::CRM::steal() };
    crm.apb2en().modify(|_, w| {
        w.gpioa().bit(true);
        w.gpiob().bit(true);
        w.gpioc().bit(true);
        w
    });

    send(b"[trace] === LQFP48 pins 1-24 sweep, 10s each ===");
    send(b"[trace] probe display wires with multimeter");

    for pin in &TRACE_PINS {
        // Format the announcement directly into a byte array.
        let mut buf = [0u8; 80];
        let mut n = 0;
        // "[trace] pin "
        for &b in b"[trace] pin " {
            buf[n] = b;
            n += 1;
        }
        // Physical pin number (decimal).
        let phys = pin.phys;
        if phys >= 10 {
            buf[n] = b'0' + (phys / 10);
            n += 1;
        }
        buf[n] = b'0' + (phys % 10);
        n += 1;
        // " = "
        buf[n] = b' ';
        buf[n + 1] = b'=';
        buf[n + 2] = b' ';
        n += 3;
        // GPIO name.
        for &b in pin.name.as_bytes() {
            buf[n] = b;
            n += 1;
        }
        // Note if any.
        if !pin.note.is_empty() {
            buf[n] = b' ';
            buf[n + 1] = b'(';
            n += 2;
            for &b in pin.note.as_bytes() {
                buf[n] = b;
                n += 1;
            }
            buf[n] = b')';
            n += 1;
        }
        // " HIGH"
        for &b in b" HIGH" {
            buf[n] = b;
            n += 1;
        }
        send(&buf[..n]);

        // Set HIGH.
        // SAFETY: exclusive GPIO ownership during the trace.
        unsafe {
            pin_output(pin.base, pin.num);
            pin_high(pin.base, pin.num);
        }

        // Wait 10 seconds.
        delay_10s();

        // Set LOW and restore to floating input.
        // SAFETY: exclusive GPIO ownership during the trace.
        unsafe {
            pin_low(pin.base, pin.num);
            pin_input(pin.base, pin.num);
        }

        send(b"[trace] done");
    }

    send(b"[trace] === sweep complete ===");
}

/// Fixed-capacity scratch line buffer.
mod heapless_line {
    pub struct Line {
        buf: [u8; 96],
        len: usize,
    }

    impl Line {
        pub const fn new() -> Self {
            Self { buf: [0; 96], len: 0 }
        }

        pub fn push(&mut self, bytes: &[u8]) {
            for &b in bytes {
                if self.len < self.buf.len() - 1 {
                    self.buf[self.len] = b;
                    self.len += 1;
                }
            }
        }

        pub fn push_num(&mut self, value: u32) {
            if value == 0 {
                self.push(b"0");
                return;
            }
            let mut digits = [0u8; 10];
            let mut n = 0;
            let mut v = value;
            while v > 0 {
                digits[n] = b'0' + (v % 10) as u8;
                v /= 10;
                n += 1;
            }
            while n > 0 {
                n -= 1;
                self.push(&[digits[n]]);
            }
        }

        pub fn bytes(&self) -> &[u8; 96] {
            &self.buf
        }
    }
}

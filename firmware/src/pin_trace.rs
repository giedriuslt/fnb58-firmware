//! Pin tracer: walks through the GPIO pins on physical LQFP48 pins 1-24
//! of the AT32F403ACGT7, sets each HIGH for 60 seconds, and prints the
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

static mut DELAY_DUMMY: u32 = 0;

/// Busy-wait ~1ms per unit (volatile write, optimization-proof at 192 MHz).
fn delay_ms(ms: u32) {
    for _ in 0..ms {
        for _ in 0..38_000 {
            // SAFETY: static mut, single-threaded.
            unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(DELAY_DUMMY), 0); }
        }
    }
}

/// Send via vendor CDC + 500ms gap for the host to collect the packet.
/// Without this gap, the next send is REJECTED (endpoint buffer busy).
fn send_trace(msg: &[u8]) {
    unsafe { fnb58_usb_send(msg.as_ptr(), msg.len() as u16); }
    delay_ms(500);
}

unsafe extern "C" {
    fn fnb58_usb_send(data: *const u8, len: u16) -> i32;
    fn fnb58_usb_configured() -> u32;
}

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


/// Busy-wait ~60 seconds. Uses a volatile write that the compiler cannot
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


/// Run the full pin trace sweep.
/// Every message is a DIRECT fnb58_usb_send call with a hardcoded byte
/// literal - no wrapper functions, no formatting, no indirection.
pub fn sweep() {
    // Wait for USB CDC to be configured (max 15 seconds).
    for _ in 0..1500 {
        unsafe {
            if fnb58_usb_configured() == 1 {
                break;
            }
        }
        // 10ms delay
        for _ in 0..100_000 {
            core::hint::black_box(());
        }
    }

    // Signal that we're alive.
    unsafe {
        send_trace(b"[trace] === pins 1-24 sweep, 60s each ===\r\n");
        send_trace(b"[trace] probe display wires w/ multimeter\r\n");
    }

    // --- Pin 2: PC13 ---
    unsafe {
        pin_output(GPIOC_BASE, 13);
        pin_high(GPIOC_BASE, 13);
    }
    send_trace(b"[trace] pin2 PC13 HIGH\r\n");
    delay_10s(); delay_10s(); delay_10s(); delay_10s(); delay_10s(); delay_10s();
    unsafe { pin_low(GPIOC_BASE, 13); pin_input(GPIOC_BASE, 13); }
    send_trace(b"[trace] done\r\n");

    // --- Pin 12: PA0 ---
    unsafe {
        pin_output(GPIOA_BASE, 0);
        pin_high(GPIOA_BASE, 0);
    }
    send_trace(b"[trace] pin12 PA0 HIGH\r\n");
    delay_10s(); delay_10s(); delay_10s(); delay_10s(); delay_10s(); delay_10s();
    unsafe { pin_low(GPIOA_BASE, 0); pin_input(GPIOA_BASE, 0); }
    send_trace(b"[trace] done\r\n");

    // --- Pin 13: PA1 ---
    unsafe {
        pin_output(GPIOA_BASE, 1);
        pin_high(GPIOA_BASE, 1);
    }
    send_trace(b"[trace] pin13 PA1 HIGH\r\n");
    delay_10s(); delay_10s(); delay_10s(); delay_10s(); delay_10s(); delay_10s();
    unsafe { pin_low(GPIOA_BASE, 1); pin_input(GPIOA_BASE, 1); }
    send_trace(b"[trace] done\r\n");

    // --- Pin 14: PA2 ---
    unsafe {
        pin_output(GPIOA_BASE, 2);
        pin_high(GPIOA_BASE, 2);
    }
    send_trace(b"[trace] pin14 PA2 HIGH\r\n");
    delay_10s(); delay_10s(); delay_10s(); delay_10s(); delay_10s(); delay_10s();
    unsafe { pin_low(GPIOA_BASE, 2); pin_input(GPIOA_BASE, 2); }
    send_trace(b"[trace] done\r\n");

    // --- Pin 15: PA3 ---
    unsafe {
        pin_output(GPIOA_BASE, 3);
        pin_high(GPIOA_BASE, 3);
    }
    send_trace(b"[trace] pin15 PA3 HIGH\r\n");
    delay_10s(); delay_10s(); delay_10s(); delay_10s(); delay_10s(); delay_10s();
    unsafe { pin_low(GPIOA_BASE, 3); pin_input(GPIOA_BASE, 3); }
    send_trace(b"[trace] done\r\n");

    // --- Pin 16: PA4 ---
    unsafe {
        pin_output(GPIOA_BASE, 4);
        pin_high(GPIOA_BASE, 4);
    }
    send_trace(b"[trace] pin16 PA4 HIGH\r\n");
    delay_10s(); delay_10s(); delay_10s(); delay_10s(); delay_10s(); delay_10s();
    unsafe { pin_low(GPIOA_BASE, 4); pin_input(GPIOA_BASE, 4); }
    send_trace(b"[trace] done\r\n");

    // --- Pin 17: PA5 (SPI1_SCK) ---
    unsafe {
        pin_output(GPIOA_BASE, 5);
        pin_high(GPIOA_BASE, 5);
    }
    send_trace(b"[trace] pin17 PA5 SPI1_SCK HIGH\r\n");
    delay_10s(); delay_10s(); delay_10s(); delay_10s(); delay_10s(); delay_10s();
    unsafe { pin_low(GPIOA_BASE, 5); pin_input(GPIOA_BASE, 5); }
    send_trace(b"[trace] done\r\n");

    // --- Pin 18: PA6 (SPI1_MISO) ---
    unsafe {
        pin_output(GPIOA_BASE, 6);
        pin_high(GPIOA_BASE, 6);
    }
    send_trace(b"[trace] pin18 PA6 SPI1_MISO HIGH\r\n");
    delay_10s(); delay_10s(); delay_10s(); delay_10s(); delay_10s(); delay_10s();
    unsafe { pin_low(GPIOA_BASE, 6); pin_input(GPIOA_BASE, 6); }
    send_trace(b"[trace] done\r\n");

    // --- Pin 19: PA7 (SPI1_MOSI) ---
    unsafe {
        pin_output(GPIOA_BASE, 7);
        pin_high(GPIOA_BASE, 7);
    }
    send_trace(b"[trace] pin19 PA7 SPI1_MOSI HIGH\r\n");
    delay_10s(); delay_10s(); delay_10s(); delay_10s(); delay_10s(); delay_10s();
    unsafe { pin_low(GPIOA_BASE, 7); pin_input(GPIOA_BASE, 7); }
    send_trace(b"[trace] done\r\n");

    // --- Pin 20: PB0 ---
    unsafe {
        pin_output(GPIOB_BASE, 0);
        pin_high(GPIOB_BASE, 0);
    }
    send_trace(b"[trace] pin20 PB0 HIGH\r\n");
    delay_10s(); delay_10s(); delay_10s(); delay_10s(); delay_10s(); delay_10s();
    unsafe { pin_low(GPIOB_BASE, 0); pin_input(GPIOB_BASE, 0); }
    send_trace(b"[trace] done\r\n");

    // --- Pin 21: PB1 ---
    unsafe {
        pin_output(GPIOB_BASE, 1);
        pin_high(GPIOB_BASE, 1);
    }
    send_trace(b"[trace] pin21 PB1 HIGH\r\n");
    delay_10s(); delay_10s(); delay_10s(); delay_10s(); delay_10s(); delay_10s();
    unsafe { pin_low(GPIOB_BASE, 1); pin_input(GPIOB_BASE, 1); }
    send_trace(b"[trace] done\r\n");

    // --- Pin 22: PB2 ---
    unsafe {
        pin_output(GPIOB_BASE, 2);
        pin_high(GPIOB_BASE, 2);
    }
    send_trace(b"[trace] pin22 PB2 HIGH\r\n");
    delay_10s(); delay_10s(); delay_10s(); delay_10s(); delay_10s(); delay_10s();
    unsafe { pin_low(GPIOB_BASE, 2); pin_input(GPIOB_BASE, 2); }
    send_trace(b"[trace] done\r\n");

    // --- Pin 23: PB10 ---
    unsafe {
        pin_output(GPIOB_BASE, 10);
        pin_high(GPIOB_BASE, 10);
    }
    send_trace(b"[trace] pin23 PB10 HIGH\r\n");
    delay_10s(); delay_10s(); delay_10s(); delay_10s(); delay_10s(); delay_10s();
    unsafe { pin_low(GPIOB_BASE, 10); pin_input(GPIOB_BASE, 10); }
    send_trace(b"[trace] done\r\n");

    // --- Pin 24: PB11 ---
    unsafe {
        pin_output(GPIOB_BASE, 11);
        pin_high(GPIOB_BASE, 11);
    }
    send_trace(b"[trace] pin24 PB11 HIGH\r\n");
    delay_10s(); delay_10s(); delay_10s(); delay_10s(); delay_10s(); delay_10s();
    unsafe { pin_low(GPIOB_BASE, 11); pin_input(GPIOB_BASE, 11); }
    send_trace(b"[trace] done\r\n");

    unsafe {
        send_trace(b"[trace] === sweep complete ===\r\n");
    }
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

//! Pin tracer: walks through GPIO pins one at a time, sets each HIGH for
//! 30 seconds, prints the pin name to serial. The user probes display
//! connector wires with a multimeter to identify which wire belongs to
//! which AT32 pin.
//!
//! Covers the first 24 probe-safe GPIOs (Port A then Port B), skipping:
//! - PA11/PA12 (USB data)
//! - PA13/PA14 (SWD)
//! - PB6/PB7 (I2C1 bus)
//!
//! Each pin is driven HIGH (output push-pull) for 30 seconds, then
//! returned to floating input (reset state).

/// GPIO port base addresses.
const GPIOA_BASE: usize = 0x4001_0800;
const GPIOB_BASE: usize = 0x4001_0C00;

/// One traceable pin: (name, base address, pin number).
const PINS: [(&str, usize, u8); 24] = [
    ("PA0", GPIOA_BASE, 0),
    ("PA1", GPIOA_BASE, 1),
    ("PA2", GPIOA_BASE, 2),
    ("PA3", GPIOA_BASE, 3),
    ("PA4", GPIOA_BASE, 4),
    ("PA5", GPIOA_BASE, 5),
    ("PA6", GPIOA_BASE, 6),
    ("PA7", GPIOA_BASE, 7),
    ("PA8", GPIOA_BASE, 8),
    ("PA9", GPIOA_BASE, 9),
    ("PA10", GPIOA_BASE, 10),
    ("PA15", GPIOA_BASE, 15),
    ("PB0", GPIOB_BASE, 0),
    ("PB1", GPIOB_BASE, 1),
    ("PB2", GPIOB_BASE, 2),
    ("PB3", GPIOB_BASE, 3),
    ("PB4", GPIOB_BASE, 4),
    ("PB5", GPIOB_BASE, 5),
    ("PB8", GPIOB_BASE, 8),
    ("PB9", GPIOB_BASE, 9),
    ("PB10", GPIOB_BASE, 10),
    ("PB11", GPIOB_BASE, 11),
    ("PB12", GPIOB_BASE, 12),
    ("PB13", GPIOB_BASE, 13),
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

/// Busy-wait approximately 1 second (calibrated for 48-192 MHz, generous).
fn delay_1s() {
    for _ in 0..30 {
        for _ in 0..1_000_000 {
            cortex_m::asm::nop();
        }
    }
}

/// Run the full pin trace sweep: 24 pins, 30 seconds each = ~12 minutes.
/// Prints the pin name before each HIGH period.
pub fn sweep(send: fn(&[u8])) {
    // Enable GPIOA and GPIOB clocks.
    // SAFETY: exclusive CRM ownership.
    let crm = unsafe { embassy_at32::pac::at32f403a::CRM::steal() };
    crm.apb2en().modify(|_, w| {
        w.gpioa().bit(true);
        w.gpiob().bit(true);
        w
    });

    send(b"[trace] === pin trace sweep ===");
    send(b"[probe] each display wire with multimeter (DC volts)");
    send(b"[watch] for the wire going HIGH on each pin");

    for (name, base, num) in PINS {
        // Print the pin name.
        let mut line = [0u8; 32];
        let prefix = b"[trace] ";
        line[..prefix.len()].copy_from_slice(prefix);
        let name_bytes = name.as_bytes();
        line[prefix.len()..prefix.len() + name_bytes.len()]
            .copy_from_slice(name_bytes);
        let n = prefix.len() + name_bytes.len();
        let suffix = b" HIGH now";
        line[n..n + suffix.len()].copy_from_slice(suffix);
        let total = n + suffix.len();
        send(&line[..total]);

        // SAFETY: exclusive GPIO ownership during the trace.
        unsafe {
            pin_output(base, num);
            pin_high(base, num);
        }

        // Wait 30 seconds.
        for _ in 0..30 {
            delay_1s();
        }

        // SAFETY: exclusive GPIO ownership during the trace.
        unsafe {
            pin_low(base, num);
            pin_input(base, num);
        }

        let mut line = [0u8; 32];
        let prefix = b"[trace] ";
        line[..prefix.len()].copy_from_slice(prefix);
        let name_bytes = name.as_bytes();
        line[prefix.len()..prefix.len() + name_bytes.len()]
            .copy_from_slice(name_bytes);
        let n = prefix.len() + name_bytes.len();
        let suffix = b" done";
        line[n..n + suffix.len()].copy_from_slice(suffix);
        let total = n + suffix.len();
        send(&line[..total]);
    }

    send(b"[trace] === sweep complete ===");
}

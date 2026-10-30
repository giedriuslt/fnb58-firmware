//! GPIO pin tracer - LQFP48 physical pins 1-24, traceable GPIOs only.
//!
//! Uses the VENDOR GPIO driver glue (fnb58_pin.c), verified working by the
//! r33 build on pins 27/28 (PB13/PB14, flash SCK/MISO). Each pin is driven
//! HIGH for 15s, then LOW for 15s (30s total), with every edge announced
//! over CDC. The user probes display-connector wires with a multimeter.
//!
//! NOTE: SPI/I2C/USB peripherals are NOT initialized here - pins are used
//! as plain GPIO. No internal pulls on any pin (PE load MOSFET gates are
//! on untraced pins - never enable pulls board-wide).

unsafe extern "C" {
    fn fnb58_gpio_clocks();
    fn fnb58_pin_output(gpio_base: u32, pin_mask: u16);
    fn fnb58_pin_high(gpio_base: u32, pin_mask: u16);
    fn fnb58_pin_low(gpio_base: u32, pin_mask: u16);
    fn fnb58_pin_input(gpio_base: u32, pin_mask: u16);
    fn fnb58_usb_send(data: *const u8, len: u16) -> i32;
    fn fnb58_usb_configured() -> u32;
}

const GPIOA_BASE: u32 = 0x4001_0800;
const GPIOB_BASE: u32 = 0x4001_0C00;
const GPIOC_BASE: u32 = 0x4001_1000;

/// Traceable pins: (GPIO base, pin mask, name literal with LQFP48 number).
/// Full user-provided pinout. PD0(5)/PD1(6) deliberately EXCLUDED: they
/// are OSC_IN/OSC_OUT carrying the 8MHz crystal = live system clock.
const TRACE_PINS: [(u32, u16, &[u8]); 28] = [
    (GPIOC_BASE, 1 << 13, b"PC13 (2)"),
    (GPIOC_BASE, 1 << 14, b"PC14 (3)"),
    (GPIOC_BASE, 1 << 15, b"PC15 (4)"),
    (GPIOA_BASE, 1 << 0, b"PA0 (10)"),
    (GPIOA_BASE, 1 << 1, b"PA1 (11)"),
    (GPIOA_BASE, 1 << 2, b"PA2 (12)"),
    (GPIOA_BASE, 1 << 3, b"PA3 (13)"),
    (GPIOA_BASE, 1 << 4, b"PA4 (14)"),
    (GPIOA_BASE, 1 << 5, b"PA5 (15)"),
    (GPIOA_BASE, 1 << 6, b"PA6 (16)"),
    (GPIOA_BASE, 1 << 7, b"PA7 (17)"),
    (GPIOB_BASE, 1 << 0, b"PB0 (18)"),
    (GPIOB_BASE, 1 << 1, b"PB1 (19)"),
    (GPIOB_BASE, 1 << 2, b"PB2 (20)"),
    (GPIOB_BASE, 1 << 10, b"PB10 (21)"),
    (GPIOB_BASE, 1 << 11, b"PB11 (22)"),
    (GPIOB_BASE, 1 << 12, b"PB12 (25)"),
    (GPIOB_BASE, 1 << 13, b"PB13 (26) flash_SCK"),
    (GPIOB_BASE, 1 << 14, b"PB14 (27) flash_MISO"),
    (GPIOB_BASE, 1 << 15, b"PB15 (28)"),
    (GPIOA_BASE, 1 << 8, b"PA8 (29)"),
    (GPIOA_BASE, 1 << 9, b"PA9 (30)"),
    (GPIOA_BASE, 1 << 10, b"PA10 (31)"),
    (GPIOA_BASE, 1 << 11, b"PA11 (32)"),
    (GPIOA_BASE, 1 << 12, b"PA12 (33)"),
    (GPIOA_BASE, 1 << 13, b"PA13 (34) SWDIO"),
    (GPIOA_BASE, 1 << 14, b"PA14 (37) SWCLK"),
    (GPIOA_BASE, 1 << 15, b"PA15 (38)"),
];

/// Volatile dummy that keeps busy-wait loops alive (opt-level=z + LTO
/// would otherwise eliminate bare nop loops).
static mut DELAY_DUMMY: u32 = 0;

/// Busy-wait roughly 1 second at 192 MHz HCLK (192M cycles / ~5 cycles
/// per volatile store loop iteration).
fn delay_1s() {
    for _ in 0..192 {
        for _ in 0..200_000 {
            // SAFETY: static mut, single-threaded bring-up.
            unsafe {
                core::ptr::write_volatile(core::ptr::addr_of_mut!(DELAY_DUMMY), 0);
            }
        }
    }
}

/// Busy-wait N seconds.
fn delay_s(n: u32) {
    for _ in 0..n {
        delay_1s();
    }
}

/// Busy-wait ~1ms per unit (39 x 1000 volatile iterations, calibrated from
/// the r32 delay loop).
fn delay_ms(ms: u32) {
    for _ in 0..ms {
        for _ in 0..39 {
            for _ in 0..1000 {
                // SAFETY: static mut, single-threaded bring-up.
                unsafe {
                    core::ptr::write_volatile(core::ptr::addr_of_mut!(DELAY_DUMMY), 0);
                }
            }
        }
    }
}

/// Send via vendor CDC + 500ms gap for the host to collect the packet.
/// Without this gap, the next send is REJECTED (endpoint buffer busy).
fn send_trace(msg: &[u8]) {
    unsafe { fnb58_usb_send(msg.as_ptr(), msg.len() as u16) };
    delay_ms(500);
}

/// Drive one pin HIGH 5s / LOW 5s, announcing each edge, then IMMEDIATELY
/// swap it back to floating input.
fn pulse(base: u32, pin_mask: u16, name: &[u8]) {
    unsafe { fnb58_pin_output(base, pin_mask) };
    unsafe { fnb58_pin_high(base, pin_mask) };
    send_trace(b"[trace] ");
    send_trace(name);
    send_trace(b" HIGH\r\n");
    delay_s(5);

    unsafe { fnb58_pin_low(base, pin_mask) };
    send_trace(b"[trace] ");
    send_trace(name);
    send_trace(b" low\r\n");
    delay_s(5);

    unsafe { fnb58_pin_input(base, pin_mask) };
}

/// Sweep: all 28 traceable GPIOs, 5s on / 5s off each (~5 min).
pub fn sweep() {
    // Wait for USB CDC to be configured (max 15 seconds).
    for _ in 0..1500 {
        unsafe {
            if fnb58_usb_configured() == 1 {
                break;
            }
        }
        delay_ms(10);
    }

    unsafe { fnb58_gpio_clocks() };

    // Force every traced pin to floating input before anything is driven.
    let all_a: u16 = TRACE_PINS
        .iter()
        .filter(|&&(b, _, _)| b == GPIOA_BASE)
        .map(|&(_, m, _)| m)
        .fold(0, |a, b| a | b);
    let all_b: u16 = TRACE_PINS
        .iter()
        .filter(|&&(b, _, _)| b == GPIOB_BASE)
        .map(|&(_, m, _)| m)
        .fold(0, |a, b| a | b);
    let all_c: u16 = TRACE_PINS
        .iter()
        .filter(|&&(b, _, _)| b == GPIOC_BASE)
        .map(|&(_, m, _)| m)
        .fold(0, |a, b| a | b);
    unsafe { fnb58_pin_input(GPIOA_BASE, all_a) };
    unsafe { fnb58_pin_input(GPIOB_BASE, all_b) };
    unsafe { fnb58_pin_input(GPIOC_BASE, all_c) };

    send_trace(b"[trace] r39 sweep: all GPIOs, 5s on / 5s off\r\n");
    send_trace(b"[trace] PD0/PD1 skipped - 8MHz crystal pins\r\n");
    send_trace(b"[trace] all pins floating before sweep\r\n");

    for &(base, mask, name) in TRACE_PINS.iter() {
        pulse(base, mask, name);
        send_trace(b"[trace] done\r\n");
    }

    // Restore every traced pin to floating input after the sweep.
    unsafe { fnb58_pin_input(GPIOA_BASE, all_a) };
    unsafe { fnb58_pin_input(GPIOB_BASE, all_b) };
    unsafe { fnb58_pin_input(GPIOC_BASE, all_c) };
    send_trace(b"[trace] all pins floating after sweep\r\n");
    send_trace(b"[trace] === sweep complete ===\r\n");
}

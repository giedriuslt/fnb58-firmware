//! Buzzer driver: PA8 (LQFP48 pin 29, user-identified 2026-11-04).
//!
//! Drives the pin with a square wave (`hz`, for `ms` milliseconds) by
//! busy-toggling — the firmware is single-task enough that a 200 ms block
//! is harmless (interrupts keep running), and a fixed cycle-count delay is
//! jitter-free at 192 MHz. `buzz(2_000, 200)` is the classic piezo click.

use embassy_at32::gpio::{Drive, OutputPin, Port};

pub struct Buzzer {
    pin: OutputPin,
}

impl Buzzer {
    /// Claim PA8 as a push-pull output, idle low.
    ///
    /// # Safety
    ///
    /// Exclusive-startup contract: claims PA8 once, before anything else
    /// touches it.
    pub unsafe fn new() -> Self {
        let pin = unsafe { OutputPin::new_push_pull(Port::A, 8, false, Drive::Moderate) }
            .expect("PA8 claim");
        Self { pin }
    }

    /// Sound the buzzer: a `hz` square wave for `ms` milliseconds (busy
    /// toggle; ~192 cycles per microsecond at the 192 MHz clock).
    pub fn buzz(&mut self, hz: u32, ms: u32) {
        // Half-period in microseconds, then in ~cycles for asm::delay.
        let half_us = 1_000_000 / hz / 2;
        let half_cycles = half_us * 192;
        let toggles = hz * ms / 500; // 2 toggles per period
        for _ in 0..toggles {
            self.pin.set_high();
            cortex_m::asm::delay(half_cycles);
            self.pin.set_low();
            cortex_m::asm::delay(half_cycles);
        }
    }
}

//! Buzzer driver for the bootloader: PA8 (LQFP48 pin 29), a trimmed
//! copy of the app's `firmware/src/buzzer.rs` — same busy-toggle square
//! wave, same cycle math at 192 MHz. One distinct beep marks update mode.

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

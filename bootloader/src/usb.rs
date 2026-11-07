//! Rust glue over the bootloader's C CDC stack (`c/bl_usb.c`): init,
//! configured check, one-packet receive poll, and a small TX ring the
//! main loop drains (the app's `console.rs` `BootLog`/`send_direct`
//! pattern: single-task access, never blocking on a busy endpoint).

// SAFETY contracts on each item: the C functions are the vendor CDC
// stack behind `bl_usb.c`; `init` once at startup before unmasking the
// USB interrupt lines, send/recv/configured from the single main task.
unsafe extern "C" {
    fn bl_usb_init();
    fn bl_usb_recv(buffer: *mut u8) -> u16;
    fn bl_usb_send(data: *const u8, len: u16) -> i32;
    fn bl_usb_configured() -> u32;
}

/// Vendor CDC init + attach. The USB clock (192 MHz PLL / 4) is already
/// programmed by the Rust clock setup, exactly like the app.
///
/// # Safety
///
/// Exclusive-startup contract: call once, then unmask the USB NVIC lines.
pub unsafe fn init() {
    // SAFETY: forwarded from this function's exclusive-startup contract.
    unsafe { bl_usb_init() }
}

/// True once the host has configured the CDC interface.
pub fn configured() -> bool {
    unsafe { bl_usb_configured() != 0 }
}

/// One receive poll: copies at most one 64-byte CDC packet into `buf`
/// (caller supplies ≥64 bytes). Returns the byte count, 0 when nothing
/// is pending.
pub fn recv(buf: &mut [u8]) -> usize {
    debug_assert!(buf.len() >= 64, "CDC packet size");
    let n = unsafe { bl_usb_recv(buf.as_mut_ptr()) } as usize;
    n.min(64)
}

/// TX ring: responses queue here and `flush_step` pushes up to four
/// 64-byte packets per call, stopping early on a busy endpoint (the next
/// loop tick continues). Overflow drops the OLDEST bytes — recent
/// evidence wins, same rule as the app's boot-log ring.
pub struct TxRing {
    buf: [u8; 1024],
    head: usize,
    len: usize,
    dropped: u32,
}

impl TxRing {
    pub const fn new() -> Self {
        Self {
            buf: [0; 1024],
            head: 0,
            len: 0,
            dropped: 0,
        }
    }

    /// Queue bytes for the host.
    pub fn push(&mut self, data: &[u8]) {
        let mut data = data;
        if data.len() > self.buf.len() {
            self.dropped += (data.len() - self.buf.len()) as u32;
            data = &data[data.len() - self.buf.len()..];
        }
        if self.len + data.len() > self.buf.len() {
            let drop_n = self.len + data.len() - self.buf.len();
            self.head = (self.head + drop_n) % self.buf.len();
            self.len -= drop_n;
            self.dropped += drop_n as u32;
        }
        for &b in data {
            self.buf[(self.head + self.len) % self.buf.len()] = b;
            self.len += 1;
        }
    }

    /// Bytes still waiting in the ring.
    pub fn pending(&self) -> usize {
        self.len
    }

    /// Drain step: up to four packets, stopping on a busy endpoint.
    /// Returns bytes actually handed to the USB stack.
    pub fn flush_step(&mut self) -> usize {
        if !configured() {
            return 0;
        }
        let mut sent = 0;
        for _ in 0..4 {
            if self.len == 0 {
                break;
            }
            let n = self.len.min(64);
            let mut chunk = [0u8; 64];
            for (i, item) in chunk.iter_mut().enumerate().take(n) {
                *item = self.buf[(self.head + i) % self.buf.len()];
            }
            // 0 = accepted, nonzero = endpoint busy (retry next tick).
            if unsafe { bl_usb_send(chunk.as_ptr(), n as u16) } != 0 {
                break;
            }
            self.head = (self.head + n) % self.buf.len();
            self.len -= n;
            sent += n;
        }
        sent
    }
}

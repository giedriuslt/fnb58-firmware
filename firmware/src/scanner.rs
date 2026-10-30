//! Display probe over the vendor C SPI driver.
//!
//! Tries SPI1, SPI2, SPI3 to find an ST7789 by sending Software Reset +
//! Sleep Out + Read Display ID and reading MISO. Every step is logged
//! over CDC so a hang or crash is immediately visible.
//!
//! I2C/TPA626 diagnostics removed for now — this module is display-only.

unsafe extern "C" {
    fn fnb58_display_probe_bus(bus_idx: i32, results: *mut u8);
    fn fnb58_display_bus_name(bus_idx: i32) -> *const u8;
    fn fnb58_display_num_buses() -> i32;
}

fn send(text: &[u8]) {
    crate::send_line(text);
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

        pub fn push_hex(&mut self, value: u8) {
            const HEX: &[u8; 16] = b"0123456789ABCDEF";
            self.push(&[HEX[(value >> 4) as usize], HEX[(value & 0xF) as usize]]);
        }

        pub fn bytes(&self) -> &[u8; 96] {
            &self.buf
        }
    }
}

/// C-string to Rust byte slice (up to NUL or 32 bytes).
unsafe fn cstr(ptr: *const u8) -> &'static [u8] {
    let mut len = 0;
    while len < 32 && *ptr.add(len) != 0 {
        len += 1;
    }
    // SAFETY: static C string from the bus descriptor table.
    unsafe { core::slice::from_raw_parts(ptr, len) }
}

/// Run one display probe cycle. Logs every step.
pub async fn run() {
    send(b"[disp] === display probe start ===");

    let num = unsafe { fnb58_display_num_buses() };
    let mut line = heapless_line::Line::new();
    line.push(b"[disp] buses: ");
    line.push_num(num as u32);
    send(line.bytes());

    for bus_idx in 0..num {
        // Bus name.
        let name_ptr = unsafe { fnb58_display_bus_name(bus_idx) };
        let name = unsafe { cstr(name_ptr) };

        let mut line = heapless_line::Line::new();
        line.push(b"[disp] probing bus ");
        line.push_num(bus_idx as u32);
        line.push(b" (");
        line.push(name);
        line.push(b")...");
        send(line.bytes());

        // Call the C probe (includes delays: ~350ms per bus).
        let mut id = [0u8; 4];
        unsafe {
            fnb58_display_probe_bus(bus_idx, id.as_mut_ptr());
        }

        // Report raw ID bytes.
        let mut line = heapless_line::Line::new();
        line.push(b"[disp] bus ");
        line.push_num(bus_idx as u32);
        line.push(b" result:");
        for i in 0..4 {
            line.push(b" ");
            line.push_hex(id[i]);
        }
        // Check for ST7789 signature (85 21 52).
        if id[0] == 0x00 && id[1] == 0x85 && id[2] == 0x21 {
            line.push(b" <-- ST7789 FOUND!");
        } else if id.iter().all(|&b| b == 0x00) {
            line.push(b" (all zeros)");
        } else if id.iter().all(|&b| b == 0xFF) {
            line.push(b" (all FF)");
        }
        send(line.bytes());
    }

    send(b"[disp] === probe complete ===");
}

//! Console and fault glue shared by every bin in this package (r167):
//! the vendor USB CDC externs, the boot-log ring, the line queue, the
//! PD_LOG ring, number formatting, the stall watch, the battery-domain
//! stage cookie and the panic/HardFault evidence handlers.
//!
//! r167: extracted verbatim from main.rs so the pdmin probe bin shares
//! the exact r166 console behavior instead of a transcription of it —
//! the log path itself must not be a variable in the experiment.

use core::sync::atomic::AtomicBool;
use core::sync::atomic::AtomicU8;
use core::sync::atomic::AtomicU32;
use core::sync::atomic::Ordering;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
// The cortex_m_rt::interrupt attribute macro expands references to the
// device's `interrupt` module — bring it into scope for the handlers.
use embassy_at32::pac::at32f403a::interrupt;

// --- Vendor C stack glue (compiled by build.rs into libvendorusb.a) ---

unsafe extern "C" {
    pub(crate) fn fnb58_usb_init();
    pub(crate) fn fnb58_usb_recv(buffer: *mut u8) -> u16;
    pub(crate) fn fnb58_usb_send(data: *const u8, len: u16) -> i32;
    pub(crate) fn fnb58_usb_configured() -> u32;
    // r134: PC-link mode (fnb58_hid.c) — the stock FNB58 USB device.
    pub(crate) fn fnb58_usb_init_hid();
    pub(crate) fn fnb58_hid_tx(report: *const u8) -> i32;
    pub(crate) fn fnb58_hid_rx(buffer: *mut u8) -> u16;
}

/// Called from the vendor IRQ handlers.
#[unsafe(no_mangle)]
extern "C" fn fnb58_usbd_event_hook() {}

/// EXINT line bits latched by the ISRs (bit n = line n saw an edge).
pub(crate) static EXTI_LATCH: AtomicU32 = AtomicU32::new(0);
/// FUSB302 INT_N (PC13, line 13) edge count.
pub(crate) static INT_COUNT: AtomicU32 = AtomicU32::new(0);
/// ISR -> main-loop doorbell (coalescing-safe: the latch carries the bits).
pub(crate) static DOORBELL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// r167: the PD task's stage breadcrumb (was pd_task::STAGE; both bins'
/// PD tasks stamp it and the SysTick watch prints it).
pub(crate) static PD_STAGE: AtomicU8 = AtomicU8::new(0);

/// r138: 4096-byte boot-log ring. USB CDC connects seconds after boot (or
/// never, on charger-only power); every boot line is kept here and sent
/// once the host connects. Overflow drops the OLDEST bytes: recent
/// evidence wins over boot-start evidence.
pub(crate) struct BootLog {
    buf: [u8; 4096],
    /// Index of the oldest buffered byte.
    head: usize,
    /// Bytes currently buffered.
    len: usize,
    /// The ring was fully sent after connection; later lines go direct.
    flushed: bool,
    /// Oldest bytes dropped to fit newer lines.
    dropped: u32,
}

static mut BOOT_LOG: BootLog = BootLog::new();

/// The boot log. SAFETY: single-task use only — `say` during bring-up and
/// the flush step in the main loop, both on the main task.
pub(crate) fn boot_log() -> &'static mut BootLog {
    unsafe { &mut *core::ptr::addr_of_mut!(BOOT_LOG) }
}

impl BootLog {
    const fn new() -> Self {
        Self {
            buf: [0; 4096],
            head: 0,
            len: 0,
            flushed: false,
            dropped: 0,
        }
    }

    /// Buffer `data` (or send it directly once the ring was flushed).
    pub(crate) fn push(&mut self, data: &[u8]) {
        if self.flushed {
            send_direct(data);
            return;
        }
        let mut data = data;
        if data.len() > self.buf.len() {
            // Oversize line even with an empty ring: keep its tail.
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

    /// One non-blocking drain step once the host may be connected: send up
    /// to four 64-byte chunks, stopping to retry on a busy endpoint (the
    /// next loop tick continues). Returns true once the ring is empty and
    /// the completion note was sent; live Sender lines may then follow.
    pub(crate) fn flush_step(&mut self, configured: bool) -> bool {
        if self.flushed {
            return true;
        }
        if !configured {
            return false;
        }
        let mut chunk = [0u8; 64];
        for _ in 0..4 {
            if self.len == 0 {
                let mut note = [0u8; 48];
                let mut n = 0;
                for &b in b"[log] boot ring sent drop=" {
                    note[n] = b;
                    n += 1;
                }
                n = push_dec(&mut note, n, self.dropped);
                note[n] = b'\r';
                note[n + 1] = b'\n';
                n += 2;
                send_direct(&note[..n]);
                self.flushed = true;
                return true;
            }
            let take = self.len.min(64);
            let first = take.min(self.buf.len() - self.head);
            chunk[..first].copy_from_slice(&self.buf[self.head..self.head + first]);
            chunk[first..take].copy_from_slice(&self.buf[..take - first]);
            // SAFETY: single-threaded; the send is an endpoint buffer copy.
            if unsafe { fnb58_usb_send(chunk.as_ptr(), take as u16) } != 0 {
                return false; // endpoint busy: retry on the next tick
            }
            self.head = (self.head + take) % self.buf.len();
            self.len -= take;
        }
        false
    }
}

/// CDC line queue. The vendor send rejects a second call while one is in
/// flight, so every console line the main loop produces is queued here and
/// drained one line per tick — one drain point guarantees the sends never
/// collide. r140: slots are 128 bytes; usbd_ept_send splits >64-byte lines
/// into maxpacket chunks itself (usbd_int.c continues on IN-complete).
pub(crate) struct Sender {
    queue: [[u8; 128]; 20],
    qhead: usize,
    qlen: usize,
    /// Send attempts rejected by the busy endpoint since last take_fails().
    fails: u32,
}

impl Sender {
    pub(crate) fn new() -> Self {
        Self {
            queue: [[0; 128]; 20],
            qhead: 0,
            qlen: 0,
            fails: 0,
        }
    }

    pub(crate) fn push(&mut self, line: &[u8]) {
        if self.qlen == self.queue.len() || line.len() > 128 {
            return; // backlog or oversize: drop, the next report suffices
        }
        let slot = (self.qhead + self.qlen) % self.queue.len();
        self.queue[slot] = [0; 128];
        self.queue[slot][..line.len()].copy_from_slice(line);
        self.qlen += 1;
    }

    /// Try to send one queued line. Non-blocking: the vendor send rejects
    /// (nonzero) while the previous packet is still uncollected by the
    /// host, and the head line then retries on the next tick — a slow host
    /// delays lines instead of losing them.
    pub(crate) fn drain(&mut self) {
        if self.qlen == 0 {
            return;
        }
        let line = self.queue[self.qhead];
        let len = line.iter().position(|&b| b == 0).unwrap_or(line.len());
        // SAFETY: single-threaded; the send is an endpoint buffer copy.
        let ok = unsafe { fnb58_usb_send(line.as_ptr(), len as u16) } == 0;
        if ok {
            self.qhead = (self.qhead + 1) % self.queue.len();
            self.qlen -= 1;
        } else {
            self.fails += 1;
        }
    }

    /// Rejected-send count since the last call.
    pub(crate) fn take_fails(&mut self) -> u32 {
        let f = self.fails;
        self.fails = 0;
        f
    }

    /// Queued (not yet sent) line count, for heartbeats.
    pub(crate) fn qlen(&self) -> usize {
        self.qlen
    }
}

/// Append decimal representation of `v` to `buf` at `pos`, returning the
/// new position (uncapped by design; callers size buffers generously).
pub(crate) fn push_dec(buf: &mut [u8], pos: usize, v: u32) -> usize {
    let mut tmp = [0u8; 10];
    let mut n = 0;
    let mut x = v;
    if x == 0 {
        tmp[0] = b'0';
        n = 1;
    } else {
        while x > 0 && n < 10 {
            tmp[n] = b'0' + (x % 10) as u8;
            x /= 10;
            n += 1;
        }
    }
    let mut p = pos;
    for k in (0..n).rev() {
        buf[p] = tmp[k];
        p += 1;
    }
    p
}

/// Append `v` as two lowercase hex digits.
pub(crate) fn push_hex8(buf: &mut [u8], pos: usize, v: u32) -> usize {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    buf[pos] = HEX[((v >> 4) & 0xF) as usize];
    buf[pos + 1] = HEX[(v & 0xF) as usize];
    pos + 2
}

/// r200: four hex digits — a full 16-bit register (push_hex8 prints
/// only the low byte; register dumps need the whole value).
pub(crate) fn push_hex16(buf: &mut [u8], pos: usize, v: u32) -> usize {
    let pos = push_hex8(buf, pos, v >> 8);
    push_hex8(buf, pos, v)
}

/// Send one line directly, retrying while the previous USB packet is
/// still in flight (bounded).
pub(crate) fn send_direct(line: &[u8]) {
    for _ in 0..50_000 {
        if unsafe { fnb58_usb_send(line.as_ptr(), line.len() as u16) } == 0 {
            return;
        }
        cortex_m::asm::delay(2000);
    }
}

/// FNB58 command checksum: CRC-8, poly 0x39, init 0x42, no reflection,
/// no xorout — over bytes 1..62 of a 64-byte HID report, compared to
/// byte 63 (the OpenFNB58 tools build their commands the same way).
pub(crate) fn crc8_fnb58(data: &[u8]) -> u8 {
    let mut crc: u8 = 0x42;
    for &b in data {
        crc ^= b;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 {
                (crc << 1) ^ 0x39
            } else {
                crc << 1
            };
        }
    }
    crc
}

/// r139: pd_task -> console lines. A tiny CS-mutex ring buffer written by
/// the PD task (`pd_log`) and drained by the main loop into the Sender
/// queue (behind the boot-ring flush, like everything else); the oldest
/// line is dropped when full. r156: 8 -> 24 slots — a session's burst
/// overflowed 8 and silently dropped the engine-end line, the one that
/// carries the diagnosis.
struct PdLog {
    lines: [[u8; 64]; 24],
    lens: [u8; 24],
    head: usize,
    count: usize,
}
static PD_LOG: embassy_sync::blocking_mutex::Mutex<
    CriticalSectionRawMutex,
    core::cell::RefCell<PdLog>,
> = embassy_sync::blocking_mutex::Mutex::new(core::cell::RefCell::new(PdLog {
    lines: [[0; 64]; 24],
    lens: [0; 24],
    head: 0,
    count: 0,
}));

/// Log one PD lifecycle line (called from the PD task). r163 (user
/// directive): entries are 64 bytes again and LONGER input is split
/// into multiple complete lines instead of being silently dropped —
/// the 64- and 96-byte limits each hid the engine-end line once as
/// its evidence fields grew. Every fragment keeps the CRLF so it
/// prints as its own console line.
pub(crate) fn pd_log(line: &[u8]) {
    if line.is_empty() {
        return;
    }
    // Trim the caller's trailing CRLF; fragments get their own.
    let mut rest = if line.ends_with(b"\r\n") {
        &line[..line.len() - 2]
    } else {
        line
    };
    while !rest.is_empty() {
        let take = rest.len().min(61);
        let (chunk, tail) = rest.split_at(take);
        let mut entry = [0u8; 64];
        entry[..chunk.len()].copy_from_slice(chunk);
        entry[chunk.len()] = b'\r';
        entry[chunk.len() + 1] = b'\n';
        let n = chunk.len() + 2;
        rest = tail;
        PD_LOG.lock(|cell| {
            let mut log = cell.borrow_mut();
            if log.count == log.lines.len() {
                log.head = (log.head + 1) % log.lines.len();
                log.count -= 1;
            }
            let slot = (log.head + log.count) % log.lines.len();
            log.lines[slot] = entry;
            log.lens[slot] = n as u8;
            log.count += 1;
        });
    }
}

/// Drain the PD_LOG ring into `sender` (one full pass; called from the
/// main loop's tick).
pub(crate) fn drain_pd_log(sender: &mut Sender) {
    let mut line = [0u8; 64];
    loop {
        let taken = PD_LOG.lock(|cell| {
            let mut log = cell.borrow_mut();
            if log.count == 0 {
                return None;
            }
            let slot = log.head;
            let len = log.lens[slot] as usize;
            line.copy_from_slice(&log.lines[slot]);
            log.head = (log.head + 1) % log.lines.len();
            log.count -= 1;
            Some(len)
        });
        match taken {
            Some(len) => sender.push(&line[..len]),
            None => break,
        }
    }
}

/// r139 stall watch: the main loop stamps MAIN_STAGE/MAIN_SEQ every pass;
/// SysTick probes periodically and, if the stamp did not move across two
/// probes, dumps both tasks' stages once over USB (best-effort raw send —
/// the loop that normally owns the console is the thing that is stuck).
/// MAIN_STAGE: 1 = doorbell wait, 2 = loop body, 3 = accel BUS lock,
/// 4 = meter BUS lock. PD_STAGE: the PD task's own breadcrumb.
pub(crate) static MAIN_STAGE: AtomicU8 = AtomicU8::new(0);
pub(crate) static MAIN_SEQ: AtomicU32 = AtomicU32::new(0);
static WATCH_FIRED: AtomicBool = AtomicBool::new(false);
static WATCH_LAST: AtomicU32 = AtomicU32::new(u32::MAX);
static SYSTICKS: AtomicU32 = AtomicU32::new(0);

/// EXINT ISR body: clear the hardware pending flags, latch the line
/// bits, ring the main-loop doorbell.
fn exti_isr(lo: u8, hi: u8) {
    embassy_at32::exti::on_interrupts_in(lo..=hi);
    let mut bits = 0u32;
    for line in lo..=hi {
        if embassy_at32::exti::take_edge(line) {
            bits |= 1u32 << line;
        }
    }
    if bits != 0 {
        EXTI_LATCH.fetch_or(bits, Ordering::AcqRel);
        DOORBELL.signal(());
    }
}

#[cortex_m_rt::interrupt]
unsafe fn EXINT4() {
    exti_isr(4, 4);
}

#[cortex_m_rt::interrupt]
unsafe fn EXINT9_5() {
    exti_isr(5, 9);
}

#[cortex_m_rt::interrupt]
unsafe fn EXINT15_10() {
    exti_isr(10, 15);
}

#[cortex_m_rt::exception]
fn SysTick() {
    embassy_at32::time_driver::systick_tick();
    // r139 stall watch: probe MAIN_SEQ periodically; if it did not move
    // across two probes (and the loop had started), name the blocking
    // stage of BOTH tasks once, straight from the exception.
    let n = SYSTICKS.fetch_add(1, Ordering::Relaxed);
    if n % 2048 == 0 {
        let seq = MAIN_SEQ.load(Ordering::Relaxed);
        let last = WATCH_LAST.swap(seq, Ordering::Relaxed);
        if seq != 0 && seq == last && !WATCH_FIRED.swap(true, Ordering::Relaxed) {
            let mut d = [0u8; 40];
            let mut w = 0;
            for &b in b"[watch] main=" {
                d[w] = b;
                w += 1;
            }
            w = push_dec(&mut d, w, u32::from(MAIN_STAGE.load(Ordering::Relaxed)));
            for &b in b" pd=" {
                d[w] = b;
                w += 1;
            }
            w = push_dec(&mut d, w, u32::from(PD_STAGE.load(Ordering::Relaxed)));
            d[w] = b'\r';
            d[w + 1] = b'\n';
            w += 2;
            unsafe {
                fnb58_usb_send(d.as_ptr(), w as u16);
            }
        }
    }
}

/// Drop the D+ pull-up on fault. The device VANISHES from the host.
fn fault_halt() -> ! {
    // r166: do NOT drop the D+ pull-up. The old drop made the port
    // vanish (forcing a cookie-wiping re-plug) and hid every fault.
    // Halted but enumerated: the console stays available.
    loop {
        cortex_m::asm::nop();
    }
}

/// r168: the BPR sits in the battery domain — CRM's APB1 PWC clock must be
/// ON and PWC.CTRL.BPWEN set before the DT registers respond at all. The
/// main build only got this from the LEXT probe block (r126); pdmin never
/// ran it, so every r167 cookie write/read silently returned 0 ('[m] cookie
/// stage=0' in every log line). The accessors are now self-sufficient.
fn bpr_access() {
    unsafe {
        use embassy_at32::pac::at32f403a::{CRM, PWC};
        let crm = CRM::steal();
        // r188 (user's external review): BOTH APB1 clocks are required —
        // PWC (bit 28) for BPWEN to matter AND BPR (bit 27) for the DT
        // registers to respond at all. Only PWC was ever enabled, so every
        // DT read returned 0 and every write was dropped ('boots=1' on
        // every boot, 'last stage=0', the run-bootloader cookie lost).
        // BPDC is a different story: it lives in the CRM block and needs
        // only BPWEN — which is why the LEXT probe always worked while
        // the DT cookies never did.
        crm.apb1en().modify(|_, w| w.pwc().bit(true));
        crm.apb1en().modify(|_, w| w.bpr().bit(true));
        // APB writes are POSTED — read the enable register back so both
        // enable writes land before the BPR access; drain BPWEN likewise.
        let _ = crm.apb1en().read();
        let pwc = PWC::steal();
        pwc.ctrl().modify(|_, w| w.bpwen().set_bit());
        let _ = pwc.ctrl().read();
    }
}

/// r164: battery-domain stage cookie. These two registers survive every
/// reset (except full power removal), so each risky step stamps its ID
/// and the next boot prints what the previous boot was doing when it
/// vanished. Stage IDs: 1 boot, 10 pd gate, 11 diagnostic, 12 session
/// start, 13 engine running, 14 finish/restore, 15 restart HR now,
/// 16 vbus rerun, 20 transmit chain entered, 21 pre-TX poll, 22 FIFO write.
pub(crate) fn dbg_stage(stage: u16) {
    bpr_access();
    unsafe {
        use embassy_at32::pac::at32f403a::BPR;
        BPR::steal()
            .dt(0)
            .write(|w| unsafe { w.dt().bits(u16::from(stage)) });
    }
}

/// Boot counter in dt1: +1 every boot (wraps at 65535). r187: returns
/// (written, read-back) — the read-back is the write path's own witness;
/// if the backup domain drops the write, the same breath reports it.
pub(crate) fn dbg_boot_count() -> (u16, u16) {
    bpr_access();
    unsafe {
        use embassy_at32::pac::at32f403a::BPR;
        let bpr = BPR::steal();
        let n = bpr.dt(1).read().dt().bits().wrapping_add(1);
        bpr.dt(1).write(|w| unsafe { w.dt().bits(n) });
        (n, bpr.dt(1).read().dt().bits())
    }
}

/// Read dt0 without modifying it (boot log).
pub(crate) fn dbg_last_stage() -> u16 {
    bpr_access();
    unsafe {
        use embassy_at32::pac::at32f403a::BPR;
        BPR::steal().dt(0).read().dt().bits()
    }
}

/// r187: raw read of DTn — the boot line prints DT0/DT1/DT2 so a broken
/// cookie handshake names its failing link in one boot.
pub(crate) fn dbg_dt(n: usize) -> u16 {
    bpr_access();
    unsafe {
        use embassy_at32::pac::at32f403a::BPR;
        BPR::steal().dt(n).read().dt().bits()
    }
}

/// r184: battery-domain bootloader-request cookie (BPR DT2). The app
/// writes 0xB007 and soft-resets; the bootloader
/// (bootloader/src/main.rs `consume_bootloader_request`) reads it, clears
/// it, and enters update mode — no OK-held + reset dance needed. The
/// cookie survives soft resets only: power removal loses the request,
/// which is the safe failure mode (the device just boots normally).
/// r187: returns the read-back of DT2 right after the write — the
/// write path's own witness.
pub(crate) fn request_bootloader() -> u16 {
    bpr_access();
    unsafe {
        use embassy_at32::pac::at32f403a::BPR;
        let bpr = BPR::steal();
        bpr.dt(2).write(|w| unsafe { w.dt().bits(0xB007) });
        bpr.dt(2).read().dt().bits()
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    // r139: best-effort panic evidence before the halt — the location as
    // one raw console line (the boot ring dies with the board, USB may
    // still be up; a failed send costs nothing, we halt anyway).
    if let Some(location) = info.location() {
        let mut d = [0u8; 96];
        let mut w = 0;
        for &b in b"[panic] " {
            d[w] = b;
            w += 1;
        }
        // Keep the tail of long workspace paths.
        let file = location.file().as_bytes();
        let file = &file[file.len().saturating_sub(70)..];
        for &b in file {
            d[w] = b;
            w += 1;
        }
        d[w] = b':';
        w += 1;
        let mut digits = [0u8; 6];
        let mut t = 0;
        let mut line_no = location.line();
        while line_no > 0 && t < digits.len() {
            digits[t] = b'0' + (line_no % 10) as u8;
            line_no /= 10;
            t += 1;
        }
        while t > 0 {
            t -= 1;
            d[w] = digits[t];
            w += 1;
        }
        d[w] = b'\r';
        d[w + 1] = b'\n';
        w += 2;
        // r163: retry like send_direct — a boot-time panic fires before
        // the host opens the port, and the single send was silently
        // lost, leaving every halt unexplained.
        for _ in 0..50_000 {
            if unsafe { fnb58_usb_send(d.as_ptr(), w as u16) } == 0 {
                break;
            }
            cortex_m::asm::delay(2000);
        }
    }
    fault_halt()
}

#[cortex_m_rt::exception]
unsafe fn HardFault(frame: &cortex_m_rt::ExceptionFrame) -> ! {
    // r166: the faulting PC/LR + CFSR, RETRIED, and USB left alive. The
    // old silent fault_halt dropped D+ (the port vanishes, the user must
    // re-plug, the cookie wipes) - every engine-stage crash looked like a
    // power cycle and the faulting code stayed anonymous.
    let pc = frame.pc();
    let lr = frame.lr();
    let cfsr = core::ptr::read_volatile(0xE000ED28u32 as *const u32);
    let mut d = [0u8; 64];
    let mut w = 0;
    for &b in b"[fault] pc=" {
        d[w] = b;
        w += 1;
    }
    for shift in [24u32, 16, 8, 0] {
        w = push_hex8(&mut d, w, (pc >> shift) & 0xFF);
    }
    for &b in b" lr=" {
        d[w] = b;
        w += 1;
    }
    for shift in [24u32, 16, 8, 0] {
        w = push_hex8(&mut d, w, (lr >> shift) & 0xFF);
    }
    for &b in b" cfsr=" {
        d[w] = b;
        w += 1;
    }
    for shift in [24u32, 16, 8, 0] {
        w = push_hex8(&mut d, w, (cfsr >> shift) & 0xFF);
    }
    for &b in b" stage=" {
        d[w] = b;
        w += 1;
    }
    w = push_dec(&mut d, w, u32::from(dbg_last_stage()));
    d[w] = b'\r';
    d[w + 1] = b'\n';
    w += 2;
    for _ in 0..50_000 {
        if unsafe { fnb58_usb_send(d.as_ptr(), w as u16) } == 0 {
            break;
        }
        cortex_m::asm::delay(2000);
    }
    // Halted but ENUMERATED: the console stays, no re-plug needed.
    loop {
        cortex_m::asm::nop();
    }
}

//! fnb58-bootloader — occupies the first 64 KiB of the flash bank.
//!
//! Boot flow:
//! 1. `clocks::configure_usb(8 MHz)` — the round-15 crystal recipe (192 MHz
//!    PLL, USB /4; falls back to HICK48 without a crystal). A hard clock
//!    failure leaves no USB/LCD/timers: the only useful act is a direct
//!    trailer check + jump, else stall (`clockless_path`).
//! 2. OK (PB4, active-low, internal pull-up) sampled exactly like the app
//!    samples its boot-mode button — 8 rounds with pull-up settle.
//! 3. OK not held + valid image (trailer magic/size/CRC all agree) → jump:
//!    IRQs off, SysTick off, NVIC fully cleared, MSP/PC loaded from
//!    0x08010000. The app sets its own VTOR (kws-x1 pattern).
//! 4. Otherwise (OK held, or the app is empty/invalid): update mode — LCD
//!    banner + one beep, the vendor C CDC stack (the only USB stack ever
//!    proven on this board), and the upload protocol.
//!
//! Update protocol over CDC (2E3C:5740, the same VID/PID as the app
//! console — the host flasher MUST confirm a `[bl]` reply to INFO before
//! writing anything):
//! - `INFO` → `[bl] INFO tag=bl2 state=<empty|valid|invalid> size=<n> crc=<hex8>`
//! - `BEGIN <size>` → `[bl] OK begin size=<n> blocks=<n>`
//!   then the raw block stream: 2056 bytes per block,
//!   `[u32 seq LE][u32 crc32 LE][2048 bytes]`, one per 2 KiB flash
//!   sector, streamed back-to-back after a single BEGIN. Each block is
//!   CRC-checked, the sector erased, 512 words programmed with read-back
//!   verify, and answered `[bl] ACK <seq>` or `[bl] NAK <seq> <reason>`
//!   (the host retries a NAK'd block in place; the countdown advances
//!   only on ACK). After the last announced block — or 3 s of silence —
//!   the device returns to Text.
//! - `COMMIT <size> <crc32hex8>` → device CRCs the written region, and
//!   only on an exact match writes the trailer sector
//!   (`[bl] OK committed crc=<hex8>`, else `[bl] ERR ...` and no trailer —
//!   the app stays invalid, recovery intact).
//! - `RESET` → `[bl] OK reset`, then a system reset into the app.
//! - Anything else → `[bl] ERR ...`. No silent failure branches.

#![no_std]
#![no_main]

mod blcd;
mod buzzer;
mod fmc;
mod usb;

use embassy_at32::pac::at32f403a::interrupt;
use embassy_executor::Spawner;
use embassy_time::Timer;
use fnb58_bootloader::crc32;
use fnb58_bootloader::meta::{self, State, APP_BASE, APP_CAPACITY};
use embassy_at32::gpio::{InputPin, Port};
use embassy_at32::spi::{Divider, Mode};

/// The board crystal (same value as the app's board.rs; the bootloader
/// must not depend on the app crate).
const HEXT_HZ: u32 = 8_000_000;

/// Build tag, carried in the LCD banner, the ready line and every INFO —
/// a paste can never be mistaken for another build. bl4: claim the PB3
/// backlight gate high in the LCD constructor (bl3 never claimed it, so
/// the update-mode screen was dark). bl3: the trailer sector guard
/// covers the whole 2 KiB (bl2 refused word programs at META_BASE+4/8/12,
/// leaving COMMIT's trailer half-written). bl2: blocks stay in the Block
/// phase (countdown on ACK) + 3 s idle timeout.
const TAG: &[u8] = b"bl4";

/// Block payload size = one internal-flash sector.
const BLOCK_DATA: usize = 2048;
/// Wire format of one block: `[u32 seq][u32 crc32][2048 bytes]`.
const BLOCK_WIRE: usize = 8 + BLOCK_DATA;
/// A Block session abandoned mid-stream (host crash, desync) returns to
/// Text after this much silence — otherwise it would swallow the next
/// session's commands as block data.
const BLOCK_IDLE_MS: u64 = 3000;

#[cortex_m_rt::exception]
fn SysTick() {
    embassy_at32::time_driver::systick_tick();
}

/// No console worth trusting during a bootloader panic; stall. Recovery:
/// reset with OK held → update mode.
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::nop();
    }
}

// --- console (single-task access, same contract as the app's BootLog) ---

static mut TX_RING: usb::TxRing = usb::TxRing::new();

fn tx_ring() -> &'static mut usb::TxRing {
    unsafe { &mut *core::ptr::addr_of_mut!(TX_RING) }
}

fn say(msg: &[u8]) {
    tx_ring().push(msg);
}

// --- number formatting (console.rs push_dec/push_hex8 equivalents) ---

fn put(buf: &mut [u8], data: &[u8]) -> usize {
    let n = data.len().min(buf.len());
    buf[..n].copy_from_slice(&data[..n]);
    n
}

fn put_dec(buf: &mut [u8], mut v: u32) -> usize {
    let mut tmp = [0u8; 10];
    let mut i = 0;
    loop {
        tmp[i] = b'0' + (v % 10) as u8;
        i += 1;
        v /= 10;
        if v == 0 {
            break;
        }
    }
    let mut n = 0;
    for &d in tmp[..i].iter().rev() {
        if n < buf.len() {
            buf[n] = d;
            n += 1;
        }
    }
    n
}

fn put_hex8(buf: &mut [u8], v: u32) -> usize {
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    for (i, slot) in buf.iter_mut().enumerate().take(8) {
        *slot = HEX[((v >> (28 - 4 * i)) & 0xF) as usize];
    }
    8
}

// --- protocol state (single-task access) ---

/// Phase marker: Text = line commands; Block = consuming the raw block
/// stream entered by BEGIN. The accumulator lives in [`Proto::block`]
/// (kept out of the enum so the feed loop borrows fields disjointly).
#[derive(Clone, Copy, PartialEq)]
enum Phase {
    Text,
    Block,
}

/// Block-stream accumulator: exactly `remaining` successful blocks are
/// expected; a NAK'd block is retried in place (remaining does not
/// advance), and the last successful block returns the device to Text.
struct BlockAcc {
    buf: [u8; BLOCK_WIRE],
    len: usize,
    remaining: u32,
    /// Last fed byte, for the idle timeout.
    last_ms: u64,
}

struct Proto {
    line: [u8; 96],
    line_len: usize,
    phase: Phase,
    /// The size committed to by BEGIN; governs which block seqs are legal.
    begin_size: u32,
    block: BlockAcc,
}

static mut PROTO: Proto = Proto {
    line: [0; 96],
    line_len: 0,
    phase: Phase::Text,
    begin_size: 0,
    block: BlockAcc {
        buf: [0; BLOCK_WIRE],
        len: 0,
        remaining: 0,
        last_ms: 0,
    },
};

fn proto() -> &'static mut Proto {
    unsafe { &mut *core::ptr::addr_of_mut!(PROTO) }
}

fn err_line(what: &[u8]) {
    let mut b = [0u8; 96];
    let mut n = 0;
    n += put(&mut b[n..], b"[bl] ERR ");
    n += put(&mut b[n..], what);
    say(&b[..n]);
    say(b"\r\n");
}

/// Feed raw CDC bytes into the state machine.
fn proto_feed(data: &[u8]) {
    let p = proto();
    for &byte in data {
        if matches!(p.phase, Phase::Block) {
            p.block.last_ms = embassy_time::Instant::now().as_millis();
            let acc = &mut p.block;
            acc.buf[acc.len] = byte;
            acc.len += 1;
            if acc.len == BLOCK_WIRE {
                let full: [u8; BLOCK_WIRE] = acc.buf;
                let accepted = handle_block(&full);
                if accepted {
                    if acc.remaining > 1 {
                        acc.remaining -= 1;
                        acc.len = 0;
                    } else {
                        // Last announced block accepted: back to Text for
                        // COMMIT / RESET / INFO.
                        p.phase = Phase::Text;
                    }
                } else {
                    // NAK answered; the host retries this same block in
                    // place, so the slot is not consumed.
                    acc.len = 0;
                }
            }
        } else if byte == b'\r' {
            continue;
        } else if byte == b'\n' {
            let len = p.line_len;
            p.line_len = 0;
            let line: [u8; 96] = p.line;
            dispatch(&line[..len]);
        } else if p.line_len >= p.line.len() {
            // Oversized command line: answer loudly, drop it.
            p.line_len = 0;
            err_line(b"line too long");
        } else {
            p.line[p.line_len] = byte;
            p.line_len += 1;
        }
    }
}

/// A Block session left idle past [`BLOCK_IDLE_MS`] returns to Text,
/// loudly — a crashed host must not leave the device swallowing the
/// next session's INFO as block data.
fn block_timeout_check() {
    let p = proto();
    if matches!(p.phase, Phase::Block)
        && embassy_time::Instant::now().as_millis().saturating_sub(p.block.last_ms)
            > BLOCK_IDLE_MS
    {
        p.phase = Phase::Text;
        err_line(b"block idle -> text (send BEGIN to restart)");
    }
}

/// Split `line` at the first space: (token, rest).
fn split_cmd(line: &[u8]) -> (&[u8], &[u8]) {
    match line.iter().position(|&b| b == b' ') {
        Some(i) => (&line[..i], &line[i + 1..]),
        None => (line, &line[line.len()..]),
    }
}

/// Parse a decimal u32 (ASCII).
fn parse_dec(s: &[u8]) -> Option<u32> {
    if s.is_empty() {
        return None;
    }
    let mut v: u32 = 0;
    for &c in s {
        if !c.is_ascii_digit() {
            return None;
        }
        v = v.checked_mul(10)?.checked_add(u32::from(c - b'0'))?;
    }
    Some(v)
}

/// Parse exactly 8 hex digits into u32 (ASCII, any case).
fn parse_hex8(s: &[u8]) -> Option<u32> {
    if s.len() != 8 {
        return None;
    }
    let mut v: u32 = 0;
    for &c in s {
        let d = match c {
            b'0'..=b'9' => c - b'0',
            b'a'..=b'f' => c - b'a' + 10,
            b'A'..=b'F' => c - b'A' + 10,
            _ => return None,
        };
        v = (v << 4) | u32::from(d);
    }
    Some(v)
}

fn dispatch(line: &[u8]) {
    if line.is_empty() {
        return;
    }
    let (cmd, rest) = split_cmd(line);
    match cmd {
        b"INFO" => info(),
        b"BEGIN" => match parse_dec(rest) {
            Some(size) => begin(size),
            None => err_line(b"begin wants size"),
        },
        b"COMMIT" => {
            let (size_s, crc_s) = split_cmd(rest);
            match (parse_dec(size_s), parse_hex8(crc_s)) {
                (Some(size), Some(crc)) => commit(size, crc),
                _ => err_line(b"commit wants size crc8"),
            }
        }
        b"RESET" => reset_flow(),
        _ => err_line(b"unknown cmd"),
    }
}

fn info() {
    let trailer = meta::read_trailer();
    let state = meta::check();
    let mut b = [0u8; 96];
    let mut n = 0;
    n += put(&mut b[..], b"[bl] INFO tag=");
    n += put(&mut b[n..], TAG);
    n += put(&mut b[n..], b" state=");
    n += put(&mut b[n..], state.word().as_bytes());
    n += put(&mut b[n..], b" size=");
    n += put_dec(&mut b[n..], trailer.size);
    n += put(&mut b[n..], b" crc=");
    n += put_hex8(&mut b[n..], trailer.crc);
    say(&b[..n]);
    say(b"\r\n");
}

fn begin(size: u32) {
    if size == 0 || size > APP_CAPACITY {
        err_line(b"begin size out of range");
        return;
    }
    let blocks = size.div_ceil(BLOCK_DATA as u32);
    let p = proto();
    p.begin_size = size;
    // Next bytes on the wire are the raw block stream: `blocks` of them,
    // retried in place on NAK, counted down only on ACK.
    p.phase = Phase::Block;
    p.block = BlockAcc {
        buf: [0; BLOCK_WIRE],
        len: 0,
        remaining: blocks,
        last_ms: embassy_time::Instant::now().as_millis(),
    };
    let mut b = [0u8; 64];
    let mut n = 0;
    n += put(&mut b[..], b"[bl] OK begin size=");
    n += put_dec(&mut b[n..], size);
    n += put(&mut b[n..], b" blocks=");
    n += put_dec(&mut b[n..], blocks);
    say(&b[..n]);
    say(b"\r\n");
}

/// One complete wire block: verify CRC, erase + program the sector with
/// per-word read-back verify, answer ACK or NAK. Returns true only when
/// the block was accepted (the caller counts the slot down on ACK).
fn handle_block(buf: &[u8; BLOCK_WIRE]) -> bool {
    let p = proto();
    let seq = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
    let want = u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]);
    let data = &buf[8..];

    let offset = seq.saturating_mul(BLOCK_DATA as u32);
    // seq*2048 must name a byte inside the BEGIN size (the final block
    // may pad past it with 0xFF) and stay inside the app region.
    if p.begin_size == 0 || offset >= p.begin_size || offset >= APP_CAPACITY {
        nak(seq, b"range");
        return false;
    }
    let got = crc32::crc32(data);
    if got != want {
        nak(seq, b"crc");
        return false;
    }
    match fmc::program_sector(APP_BASE + offset, data) {
        Ok(()) => {
            let mut b = [0u8; 32];
            let mut n = 0;
            n += put(&mut b[..], b"[bl] ACK ");
            n += put_dec(&mut b[n..], seq);
            say(&b[..n]);
            say(b"\r\n");
            true
        }
        Err(e) => {
            nak(seq, e.word().as_bytes());
            false
        }
    }
}

fn nak(seq: u32, reason: &[u8]) {
    let mut b = [0u8; 48];
    let mut n = 0;
    n += put(&mut b[..], b"[bl] NAK ");
    n += put_dec(&mut b[n..], seq);
    n += put(&mut b[n..], b" ");
    n += put(&mut b[n..], reason);
    say(&b[..n]);
    say(b"\r\n");
}

/// COMMIT <size> <crc8hex>: verify the written region matches the host's
/// CRC, and only then write the trailer. A mismatch leaves the trailer
/// untouched — the app stays invalid and update mode stays reachable.
fn commit(size: u32, want: u32) {
    if size == 0 || size > APP_CAPACITY {
        err_line(b"commit size out of range");
        return;
    }
    let got = meta::image_crc(size);
    if got != want {
        let mut b = [0u8; 64];
        let mut n = 0;
        n += put(&mut b[..], b"[bl] ERR commit crc got=");
        n += put_hex8(&mut b[n..], got);
        n += put(&mut b[n..], b" want=");
        n += put_hex8(&mut b[n..], want);
        say(&b[..n]);
        say(b"\r\n");
        return;
    }
    if let Err(e) = fmc::erase_sector(meta::META_BASE) {
        err_line(e.word().as_bytes());
        return;
    }
    let words = [meta::MAGIC, size, want, !want];
    for (i, &word) in words.iter().enumerate() {
        if let Err(e) = fmc::word_program(meta::META_BASE + (i as u32) * 4, word) {
            err_line(e.word().as_bytes());
            return;
        }
    }
    // Verify the metadata exactly, then confirm to the host.
    let t = meta::read_trailer();
    if t.magic != meta::MAGIC
        || t.size != size
        || t.crc != want
        || t.crc_inv != !want
    {
        err_line(b"commit verify");
        return;
    }
    let mut b = [0u8; 48];
    let mut n = 0;
    n += put(&mut b[..], b"[bl] OK committed crc=");
    n += put_hex8(&mut b[n..], want);
    say(&b[..n]);
    say(b"\r\n");
}

fn reset_flow() {
    say(b"[bl] OK reset\r\n");
    // Bounded drain: a configured host gets the line; an unconfigured
    // one (the ring can never drain) just costs two seconds, then the
    // reset happens either way.
    for _ in 0..2000 {
        tx_ring().flush_step();
        if tx_ring().pending() == 0 {
            break;
        }
        cortex_m::asm::delay(192_000); // ~1 ms at 192 MHz
    }
    cortex_m::peripheral::SCB::sys_reset();
}

// --- boot flow ---

/// OK = PB4, to-GND switch with an internal pull-up — sampled exactly
/// like the app samples its boot-mode button (`main.rs` pc_link block).
fn sample_ok() -> bool {
    let ok = unsafe { InputPin::new_pull_up(Port::B, 4).expect("PB4 claim") };
    let mut held = true;
    for _ in 0..8 {
        if !ok.is_low() {
            held = false;
            break;
        }
        cortex_m::asm::delay(192 * 50); // ~50 us pull-up settle
    }
    held
}

/// Disable IRQs, stop the SysTick, clear every NVIC enable/pending bit,
/// then load the app's stack pointer and reset handler and branch. The
/// app sets its own VTOR as its first act (kws-x1 pattern).
fn jump_to_app() -> ! {
    unsafe {
        cortex_m::interrupt::disable();
        // Stop the SysTick the time driver left running: a tick through
        // the old vector table after the MSP switch would be chaos.
        let syst = &*cortex_m::peripheral::SYST::PTR;
        syst.csr.modify(|csr| csr & !0b111);
        // Clear every external IRQ enable and pending bit.
        let nvic = &*cortex_m::peripheral::NVIC::PTR;
        for reg in nvic.icer.iter() {
            reg.write(!0);
        }
        for reg in nvic.icpr.iter() {
            reg.write(!0);
        }
        cortex_m::asm::dsb();
        cortex_m::asm::isb();

        let sp = core::ptr::read_volatile(APP_BASE as *const u32);
        let pc = core::ptr::read_volatile((APP_BASE + 4) as *const u32);
        // Refuse garbage vectors: SP must sit in the 96 KiB SRAM and PC in
        // the app region with the Thumb bit. The CRC check makes this
        // near-impossible, but a stall here is recoverable (OK-held reset
        // enters update mode) while a wild jump is not.
        const RAM_END: u32 = 0x2000_0000 + 96 * 1024;
        const APP_PC_END: u32 = fnb58_bootloader::meta::APP_END;
        let pc_thumb = pc & 1 == 1;
        let pc_in_app = (pc & !1) >= APP_BASE && (pc & !1) < APP_PC_END;
        if sp < 0x2000_0000 || sp > RAM_END || !pc_thumb || !pc_in_app {
            loop {
                cortex_m::asm::nop();
            }
        }
        core::arch::asm!(
            "msr msp, {sp}",
            "bx {pc}",
            sp = in(reg) sp,
            pc = in(reg) pc,
            options(noreturn)
        );
    }
}

/// No clocks → no USB, no LCD, no timers. The only useful act: check the
/// trailer directly (works at the reset 8 MHz HSI) and jump if the image
/// is intact; otherwise this is unrecoverable except by BOOT0 DFU.
fn clockless_path() -> ! {
    if matches!(meta::check(), State::Valid { .. }) {
        jump_to_app();
    }
    loop {
        cortex_m::asm::nop();
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Crystal clock: 192 MHz PLL, USB /4 — the round-15 recipe. Falls
    // back to HICK48 without a crystal (USB out of spec but alive).
    let Ok((clocks, _)) =
        (unsafe { embassy_at32::clocks::configure_usb(HEXT_HZ) })
    else {
        clockless_path()
    };
    unsafe { embassy_at32::init(clocks) };
    unsafe { cortex_m::interrupt::enable() };

    let ok_held = sample_ok();

    if !ok_held && matches!(meta::check(), State::Valid { .. }) {
        jump_to_app();
    }
    // OK held (explicit update request) or the app is empty/invalid
    // (recovery): update mode either way.
    update_mode().await;
}

/// LCD banner + beep + CDC + the command loop. Never returns.
async fn update_mode() -> ! {
    // LCD first: visible state before anything else can fail.
    let spi = unsafe { embassy_at32::spi::Spi::new_spi2(Mode::Mode0, Divider::Div4) };
    let mut lcd = unsafe { blcd::Lcd::new(spi) };
    lcd.init().await;
    let app_ok = matches!(meta::check(), State::Valid { .. });
    let l2: &[u8] = if app_ok {
        b"UPDATE MODE\0"
    } else {
        b"NO APP - UPDATE\0"
    };
    lcd.banner(b"FNB58 BL4", l2, b"CONNECT USB\0").await;

    // One distinct beep marks update mode (the app's boot beep is
    // 500 Hz / 30 ms).
    let mut buzzer = unsafe { buzzer::Buzzer::new() };
    buzzer.buzz(1000, 80);

    // Vendor C CDC (round-15 recipe): init, then unmask the two USB lines
    // — the same order the app's main.rs uses.
    unsafe { usb::init() };
    unsafe {
        cortex_m::peripheral::NVIC::unmask(interrupt::USBFS_L_CAN1_RX0);
        cortex_m::peripheral::NVIC::unmask(interrupt::USBFS_H_CAN1_TX);
    }
    say(b"[bl] bl4 update mode ready\r\n");

    loop {
        tx_ring().flush_step();
        let mut pkt = [0u8; 64];
        let n = usb::recv(&mut pkt);
        if n > 0 {
            proto_feed(&pkt[..n]);
        } else {
            Timer::after_millis(1).await;
            block_timeout_check();
        }
    }
}

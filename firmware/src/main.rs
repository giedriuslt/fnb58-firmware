//! FNB58 r61: button tracer v3. Phase 1 (6 s+): floating inputs report
//! externally-pulled buttons (LEFT = PA15 found this way). Phase 2
//! (30 s): 50 ms internal pull-up windows cycle the main candidates.
//! Phase 3 (forever): the same sweep over PB4/5/8/9. Heartbeats every
//! ~3 s distinguish hang (COM alive, beats stop) from fault (COM drops).

#![no_std]
#![no_main]

mod board;
mod buttons;
mod pd;
mod pin_trace;

use embassy_executor::Spawner;
use embassy_time::Timer;

// --- Vendor C stack glue (compiled by build.rs into libvendorusb.a) ---

unsafe extern "C" {
    fn fnb58_usb_init();
    fn fnb58_usb_send(data: *const u8, len: u16) -> i32;
    fn fnb58_usb_configured() -> u32;
    fn fnb58_lcd_init();
    fn fnb58_lcd_fill(color: u16);
    fn fnb58_lcd_draw_text(
        x: u16,
        y: u16,
        s: *const u8,
        scale: u8,
        fg: u16,
        bg: u16,
    ) -> u16;
}

/// Called from the vendor IRQ handlers.
#[unsafe(no_mangle)]
extern "C" fn fnb58_usbd_event_hook() {}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Crystal clock: 192 MHz PLL, USB /4.
    let Ok((clocks, _)) =
        (unsafe { embassy_at32::clocks::configure_usb(board::HEXT_HZ) })
    else {
        loop {
            cortex_m::asm::nop();
        }
    };
    unsafe { embassy_at32::init(clocks) };
    unsafe { cortex_m::interrupt::enable() };

    // Unmask USB interrupt lines.
    use embassy_at32::pac::at32f403a::interrupt;
    unsafe {
        cortex_m::peripheral::NVIC::unmask(interrupt::USBFS_L_CAN1_RX0);
        cortex_m::peripheral::NVIC::unmask(interrupt::USBFS_H_CAN1_TX);
    }

    // Vendor USB init (CDC class).
    unsafe {
        fnb58_usb_init();
    }

    // Wait for USB CDC to be configured (max 15 seconds).
    for _ in 0..150 {
        unsafe {
            if fnb58_usb_configured() == 1 {
                break;
            }
        }
        Timer::after_millis(100).await;
    }

    async fn say(msg: &'static [u8]) {
        unsafe { fnb58_usb_send(msg.as_ptr(), msg.len() as u16) };
        Timer::after_millis(500).await;
    }

    say(b"[lcd] FNB58 r61: button trace\r\n").await;

    unsafe { fnb58_lcd_init() };
    unsafe { fnb58_lcd_fill(0x0000) };
    unsafe {
        fnb58_lcd_draw_text(20, 60, b"FNB58 r61\0".as_ptr(), 3, 0x07E0, 0x0000)
    };
    unsafe {
        fnb58_lcd_draw_text(20, 120, b"BTN TRACE\0".as_ptr(), 3, 0xFFFF, 0x0000)
    };

    // Configure candidate pins as floating inputs first; the ~2.5 s of
    // banner messages below let floating pins settle before the baseline.
    let mut tracer = buttons::Tracer::new();

    say(b"[btn] phase 1 (6 s+): press buttons now, one at a time\r\n").await;
    say(b"[btn] phase 2: HOLD each button ~4 s, in order:\r\n").await;
    say(b"[btn]   back, left, ok, right  (left=PA15 already known)\r\n")
        .await;
    say(b"[btn] phase 3 sweeps PB4/5/8/9 with 50 ms pull-up pulses\r\n")
        .await;
    say(b"[btn] alive lines every ~3 s; if they stop but COM stays,\r\n")
        .await;
    say(b"[btn] it hung; if COM drops, it faulted - note which!\r\n").await;

    // Prints the settled idle levels and arms the change baselines.
    tracer.snapshot().await;

    loop {
        tracer.poll().await;
        Timer::after_millis(20).await;
    }
}

#[cortex_m_rt::exception]
fn SysTick() {
    embassy_at32::time_driver::systick_tick();
}

/// Drop the D+ pull-up on fault. The device VANISHES from the host.
fn fault_halt() -> ! {
    unsafe {
        let ctrl = (0x4000_5c00usize + 0x40) as *mut u32;
        core::ptr::write_volatile(ctrl, core::ptr::read_volatile(ctrl) | 0b10);
        let cfg = (0x4000_5c00usize + 0x60) as *mut u32;
        core::ptr::write_volatile(cfg, core::ptr::read_volatile(cfg) | 0b10);
    }
    loop {
        cortex_m::asm::nop();
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    fault_halt()
}

#[cortex_m_rt::exception]
unsafe fn HardFault(_frame: &cortex_m_rt::ExceptionFrame) -> ! {
    fault_halt()
}

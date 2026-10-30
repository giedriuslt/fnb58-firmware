//! FNB58 LCD bring-up: ST7789V 320x240 on SPI2 (verified pin map in
//! c/fnb58_lcd.c), color-bar test pattern drawn at boot. CDC echo after.

#![no_std]
#![no_main]

mod board;
mod buttons;
mod pin_trace;

static TICK: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

use embassy_executor::Spawner;
use embassy_time::Timer;

// --- Vendor C stack glue (compiled by build.rs into libvendorusb.a) ---

unsafe extern "C" {
    fn fnb58_usb_init();
    fn fnb58_usb_recv(buffer: *mut u8) -> u16;
    fn fnb58_usb_send(data: *const u8, len: u16) -> i32;
    fn fnb58_usb_configured() -> u32;
    fn fnb58_lcd_init();
    fn fnb58_lcd_test_pattern();
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

    // Button monitor init.
    buttons::init();

    // Wait for USB CDC to be configured (max 15 seconds).
    // The host needs time to enumerate and open the COM port.
    for _ in 0..150 {
        unsafe {
            if fnb58_usb_configured() == 1 {
                break;
            }
        }
        Timer::after_millis(100).await;
    }

    // LCD bring-up: ST7789V init + color-bar test pattern.
    async fn say(msg: &'static [u8]) {
        unsafe { fnb58_usb_send(msg.as_ptr(), msg.len() as u16) };
        Timer::after_millis(500).await;
    }
    say(b"[lcd] FNB58 r45: BLK PB3 high + swjtag remap\r\n").await;
    unsafe { fnb58_lcd_init() };
    say(b"[lcd] init done, drawing bars\r\n").await;
    unsafe { fnb58_lcd_test_pattern() };
    say(b"[lcd] pattern drawn\r\n").await;

    // After the sweep: normal echo operation.
    let mut rx = [0u8; 64];
    loop {
        let n = unsafe { fnb58_usb_recv(rx.as_mut_ptr()) } as usize;
        if n > 0 && n <= rx.len() - 2 {
            rx[n] = b'\r';
            rx[n + 1] = b'\n';
            unsafe {
                fnb58_usb_send(rx.as_ptr(), (n + 2) as u16);
            }
        }
        Timer::after_millis(2).await;
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

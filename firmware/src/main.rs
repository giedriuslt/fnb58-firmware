//! FNB58 open-firmware scaffold: clock bring-up and an ST7789 320x240
//! panel smoke test over SPI1.
//!
//! Draw stages: a full-screen fill, then colored bars with a moving white
//! stripe, blinking the backlight at the end of each pass. Everything
//! hardware-specific is in [`board`]; the pin map is UNVERIFIED until the
//! FNB58's display connector is traced.

#![no_std]
#![no_main]

mod board;
mod panel;

use embassy_at32::pac::at32f403a::interrupt;
use embassy_executor::Spawner;
use embassy_time::Timer;
use panic_halt as _;

use crate::board::{HEIGHT, WIDTH};

/// One RGB565 row, built in place and streamed through DMA.
const BARS: [u16; 8] = [
    rgb565(255, 0, 0),
    rgb565(255, 85, 0),
    rgb565(255, 255, 0),
    rgb565(0, 255, 0),
    rgb565(0, 255, 255),
    rgb565(0, 85, 255),
    rgb565(128, 0, 255),
    rgb565(255, 255, 255),
];

const fn rgb565(r: u8, g: u8, b: u8) -> u16 {
    ((r >> 3) as u16) << 11 | ((g >> 2) as u16) << 5 | (b >> 3) as u16
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Clocks first: everything else measures time against HCLK.
    let clocks = match unsafe { embassy_at32::clocks::configure(board::CLOCK) } {
        Ok(clocks) => clocks,
        Err(_) => loop {
            cortex_m::asm::nop();
        },
    };
    unsafe { embassy_at32::init(clocks) };
    unsafe { cortex_m::interrupt::enable() };

    // Panel interrupt sources: the SPI flag waits and the DMA completion.
    unsafe {
        cortex_m::peripheral::NVIC::unmask(match board::PANEL_DMA.0 {
            embassy_at32::dma::Unit::Dma1 => interrupt::DMA1_CHANNEL1,
            embassy_at32::dma::Unit::Dma2 => interrupt::DMA2_CHANNEL1,
        });
    }

    let Some(mut panel) = unsafe { panel::init() }.await else {
        // Transport dead: blink nothing, but stay alive for a debugger.
        loop {
            cortex_m::asm::nop();
        }
    };

    let mut row = [0u16; WIDTH as usize];
    let mut stripe = 0usize;
    loop {
        if !panel.full_window().await {
            continue;
        }
        for y in 0..HEIGHT as usize {
            let bar_width = WIDTH as usize / BARS.len();
            for (i, pixel) in row.iter_mut().enumerate() {
                *pixel = if i == stripe && y % 8 < 4 {
                    // A moving white marker stripe, four pixels tall.
                    BARS[7]
                } else {
                    BARS[i / bar_width % BARS.len()]
                };
            }
            if !panel
                .write_row(unsafe {
                    core::slice::from_raw_parts(row.as_ptr().cast(), panel::ROW_BYTES)
                })
                .await
            {
                break;
            }
        }
        stripe = (stripe + 1) % WIDTH as usize;
        panel.set_backlight(true);
        Timer::after_millis(500).await;
        panel.set_backlight(false);
        Timer::after_millis(500).await;
    }
}

#[cortex_m_rt::exception]
fn SysTick() {
    embassy_at32::time_driver::systick_tick();
}

#[cortex_m_rt::interrupt]
fn DMA1_CHANNEL1() {
    embassy_at32::dma::on_interrupt(embassy_at32::dma::Unit::Dma1, 1);
}

#[cortex_m_rt::interrupt]
fn DMA2_CHANNEL1() {
    embassy_at32::dma::on_interrupt(embassy_at32::dma::Unit::Dma2, 1);
}

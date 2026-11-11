//! Ripple capture analysis for the FNB58 (r199): a 1:1 port of kws-x1's
//! `firmware-core/src/ripple.rs` + `spectrum.rs` (the kws-x1 standalone
//! tree is the reference; see the fidelity notes on each deviation).
//!
//! Ported deviations from kws-x1, both hardware-forced:
//! - `PGA_GAIN` is 1, not 2: the HC32F460's internal PGA has no AT32F403A
//!   equivalent. Voltage estimates are pin-referred (uncalibrated against
//!   the bus, exactly as kws-x1's ripple deliberately is).
//! - Sampling times are the AT32's eight discrete windows (1.5..239.5
//!   cycles) instead of the HC32's free 0..255 cycle count, and a 12-bit
//!   conversion takes 12.5 cycles (the HC32 takes 13). Both are handled
//!   in half-cycle integers, so the rate math stays exact.

#![cfg_attr(not(test), no_std)]

pub mod ripple;
pub mod spectrum;

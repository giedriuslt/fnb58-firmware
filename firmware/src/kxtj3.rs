//! KXTJ3-1057 3-axis accelerometer (I2C 0x0E, WHO_AM_I 0x35 — verified on
//! this board in r20) used as the screen-flip sensor.
//!
//! The KWS-X1 used an LIS3DH with hardware 6D position interrupts; the
//! KXTJ3 has no such engine, so orientation is decided in software from
//! the gravity vector: the tester is calibrated lying however it rests at
//! boot, and "flipped" means the device was turned over — the gravity
//! axis sign reversed while the horizontal axes read near zero.

use embassy_at32::i2c::I2c;
use embedded_hal_async::i2c::{I2c as _, Operation};

/// 7-bit address with SA0 = 0 (verified in the r20 scan).
pub const ADDRESS: u8 = 0x0E;
/// WHO_AM_I (0x0F) = 0x35 for the KXTJ3-1057.
const WHO_AM_I: u8 = 0x0F;
const WHO_AM_I_VALUE: u8 = 0x35;

/// CTRL_REG1 (0x1B): PC1 = powered, RES = high resolution, DR = 50 Hz.
const CTRL_REG1: u8 = 0x1B;
const CTRL_REG1_POWERED_50HZ: u8 = 0b1 << 7 | 0b1 << 6 | 0b0011;
/// DCST_RESP (0x06): the first of the six output bytes (XOUT_L).
const XOUT_L: u8 = 0x06;

/// One gravity vector in ~counts (±2 g range: ~1024 counts per g after
/// the 12-bit left-justified values are shifted down).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Gravity {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

impl Gravity {
    /// True when the vector is dominated by one axis (a settled, flat or
    /// upright device — not mid-tilt).
    pub fn settled(&self) -> bool {
        // ~0.5 g per count band at ±2 g / 12 bits ≈ 512 counts.
        let big = |v: i16| v.abs() > 400;
        let small = |v: i16| v.abs() < 300;
        (big(self.x) && small(self.y) && small(self.z))
            || (big(self.y) && small(self.x) && small(self.z))
            || (big(self.z) && small(self.x) && small(self.y))
    }
}

/// Configure the KXTJ3: high resolution, 50 Hz output. Returns false when
/// the WHO_AM_I or the configuration write fails.
pub async fn init(bus: &mut I2c) -> bool {
    let mut id = [0u8; 1];
    if bus
        .transaction(
            ADDRESS,
            &mut [Operation::Write(&mut [WHO_AM_I]), Operation::Read(&mut id)],
        )
        .await
        .is_err()
        || id[0] != WHO_AM_I_VALUE
    {
        return false;
    }
    write(bus, CTRL_REG1, CTRL_REG1_POWERED_50HZ)
        .await
        .is_ok()
}

async fn write(bus: &mut I2c, reg: u8, val: u8) -> Result<(), ()> {
    bus.transaction(ADDRESS, &mut [Operation::Write(&mut [reg, val])])
        .await
        .map_err(|_| ())
}

/// Read one gravity sample. The outputs are 12-bit values left-justified
/// in XOUT_L..ZOUT_H (KX DS register map): each axis is (msb << 8 | lsb)
/// shifted down by 4 as a signed 12-bit value.
pub async fn read(bus: &mut I2c) -> Result<Gravity, ()> {
    let mut buf = [0u8; 6];
    bus.transaction(ADDRESS, &mut [Operation::Write(&mut [XOUT_L]), Operation::Read(&mut buf)])
        .await
        .map_err(|_| ())?;
    let axis = |l: u8, h: u8| -> i16 { (((h as u16) << 8 | l as u16) >> 4) as u16 as i16 };
    Ok(Gravity {
        x: axis(buf[0], buf[1]),
        y: axis(buf[2], buf[3]),
        z: axis(buf[4], buf[5]),
    })
}

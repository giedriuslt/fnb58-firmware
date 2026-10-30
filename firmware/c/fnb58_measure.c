/*!
 * @file fnb58_measure.c
 * TPA626 (INA226-protocol) V/I/P reader on I2C1 (PB6/PB7, addr 0x40).
 *
 * Register map (INA226-compatible; die id 0x2260 confirmed earlier):
 *   0x00 config, 0x01 shunt_v (2.5 uV/lsb), 0x02 bus_v (1.25 mV/lsb),
 *   0x03 power, 0x04 current, 0x05 calibration.
 *
 * Shunt resistance is NOT yet measured on this board: R_SHUNT_UOHM is a
 * guess (10 mOhm). Bus voltage is exact regardless; current/power scale
 * linearly with the real value -> calibrate against a known load later.
 */

#include "at32f403a_407_conf.h"

#define TPA_ADDR 0x40

/* Assumed shunt (micro-ohms) - ADJUST after calibrating with known load. */
#define R_SHUNT_UOHM 10000

/* CURRENT_LSB chosen for ~0-6 A range headroom: 50 uA. */
#define CURRENT_LSB_UA 50

extern uint32_t fnb58_i2c_bitbang_read16(uint16_t addr, uint8_t reg, uint16_t *val);
extern uint32_t fnb58_i2c_bitbang_write16(uint16_t addr, uint8_t reg, uint16_t val);

static uint8_t tpa_ok;

/* Write one 16-bit BE register (bit-bang). 0 = OK. */
static uint32_t tpa_write16(uint8_t reg, uint16_t val)
{
    return fnb58_i2c_bitbang_write16(TPA_ADDR, reg, val);
}

/* Read one 16-bit BE register (bit-bang). 0 = OK. */
static uint32_t tpa_read16(uint8_t reg, uint16_t *val)
{
    return fnb58_i2c_bitbang_read16(TPA_ADDR, reg, val);
}

void fnb58_measure_init(void)
{
    /* Hardware I2C1 stays unused: its peripheral wedges (BUSY stuck,
     * r49/r50 findings). All TPA626 traffic is bit-banged on PB6/PB7. */

    /* Manufacturer/die id check (informational only). */
    uint16_t mfr = 0, die = 0;
    tpa_read16(0xFE, &mfr);
    tpa_read16(0xFF, &die);

    /* Config: AVG=64, VBUS/VSHT CT=588 us, shunt+bus continuous.
     * (INA226 encoding: 0100 1000 1001 0111) */
    tpa_write16(0x00, 0x4897);

    /* Calibration: CAL = 0.00512 / (R_shunt * CURRENT_LSB) */
    uint32_t cal = 5120000UL / ((uint32_t)R_SHUNT_UOHM * CURRENT_LSB_UA / 1000UL);
    tpa_write16(0x05, (uint16_t)cal);

    tpa_ok = 1;
}

/* Read bus voltage (mV), current (uA), power (mW). 0 = OK. */
uint32_t fnb58_measure_read(uint32_t *mv, int32_t *ua, int32_t *mw)
{
    uint16_t bus = 0, cur = 0, pwr = 0;
    uint32_t e;

    if (!tpa_ok) {
        return 1;
    }
    e = tpa_read16(0x02, &bus);
    if (e) {
        return e;
    }
    e = tpa_read16(0x04, &cur);
    if (e) {
        return e;
    }
    e = tpa_read16(0x03, &pwr);
    if (e) {
        return e;
    }

    /* Bus: LSB 1.25 mV -> mv = raw * 5 / 4. */
    *mv = ((uint32_t)bus * 5U) / 4U;
    /* Current reg is signed, LSB = CURRENT_LSB_UA. */
    int32_t scur = (int16_t)cur;
    *ua = scur * (int32_t)CURRENT_LSB_UA;
    /* Power reg: LSB = 25 * CURRENT_LSB = 1.25 mW. */
    int32_t spwr = (int16_t)pwr;
    *mw = (spwr * 5) / 4;
    return 0;
}

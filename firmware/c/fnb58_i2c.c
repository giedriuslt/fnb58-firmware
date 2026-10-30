//! Vendor I2C glue: I2C1 on PB6/PB7 (SCL/SDA — pin 42/43, user-traced)
//! at 100 kHz through the vendor `i2c_application` polling library.
//!
//! The Rust scanner calls these; the actual bus work is 100% vendor code.

#include "at32f403a_407_conf.h"
#include "i2c_application.h"

#define FNB_I2C_SPEED               100000
#define FNB_I2C_TIMEOUT             20000

i2c_handle_type fnb_hi2c;

/* Vendor i2c_config calls this: GPIO + peripheral clock + i2c_init. */
void i2c_lowlevel_init(i2c_handle_type* hi2c)
{
  gpio_init_type gpio_initstructure;

  /* i2c1 + gpiob clocks */
  crm_periph_clock_enable(CRM_I2C1_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);

  gpio_initstructure.gpio_out_type       = GPIO_OUTPUT_OPEN_DRAIN;
  gpio_initstructure.gpio_pull           = GPIO_PULL_NONE;
  gpio_initstructure.gpio_mode           = GPIO_MODE_MUX;
  gpio_initstructure.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;

  gpio_initstructure.gpio_pins = GPIO_PINS_6; /* SCL, pin 42 */
  gpio_init(GPIOB, &gpio_initstructure);

  gpio_initstructure.gpio_pins = GPIO_PINS_7; /* SDA, pin 43 */
  gpio_init(GPIOB, &gpio_initstructure);

  i2c_init(hi2c->i2cx, I2C_FSMODE_DUTY_2_1, FNB_I2C_SPEED);
  i2c_own_address1_set(hi2c->i2cx, I2C_ADDRESS_MODE_7BIT, 0x00);
}

void fnb58_i2c_init(void)
{
  fnb_hi2c.i2cx = I2C1;
  fnb_hi2c.dma_tx_channel = 0;
  fnb_hi2c.dma_rx_channel = 0;
  i2c_config(&fnb_hi2c);
}

/* 0 = ACK (device present), nonzero = vendor error status.
 * Artery's i2c_application expects SHIFTED (8-bit) addresses - their
 * examples pass 0xA0 for a 0x50 EEPROM (i2c_7bit_address_send writes
 * the value raw into DT). Shift our 7-bit input. */
uint32_t fnb58_i2c_probe(uint16_t address)
{
  uint8_t b;
  return i2c_master_receive(&fnb_hi2c, address << 1, &b, 1, FNB_I2C_TIMEOUT);
}

/* Read `len` bytes starting at 8-bit register `reg`. 0 = OK. */
uint32_t fnb58_i2c_read(uint16_t address, uint8_t reg, uint8_t *out, uint16_t len)
{
  return i2c_memory_read(&fnb_hi2c, I2C_MEM_ADDR_WIDIH_8, address << 1, reg, out, len, FNB_I2C_TIMEOUT);
}

/* Write `len` bytes to 8-bit register `reg`. 0 = OK. */
uint32_t fnb58_i2c_write(uint16_t address, uint8_t reg, const uint8_t *data, uint16_t len)
{
  return i2c_memory_write(&fnb_hi2c, I2C_MEM_ADDR_WIDIH_8, address << 1, reg,
                          (uint8_t *)data, len, FNB_I2C_TIMEOUT);
}

/* Raw I2C1 status register (bit1 = BUSYF: bus physically busy). */
uint32_t fnb58_i2c_sts(void)
{
  return (I2C1->sts2 << 16) | I2C1->sts1;
}

/* ~5 us at 192 MHz (volatile so it survives optimization). */
static void bb_delay(void)
{
  volatile uint32_t n = 1000;
  while (n--) { __asm volatile("nop"); }
}

/*!
 * Raw bit-bang probe: bypasses the I2C peripheral entirely.
 * PB6=SCL PB7=SDA as open-drain GPIO; START, address+R, sample ACK bit.
 * Returns: bit0-1 idle SCL/SDA levels, bit8 = ACK seen.
 * Leaves pins released (high); caller should re-init hw I2C after.
 */
uint32_t fnb58_i2c_bitbang(uint16_t addr)
{
  gpio_type *gb = GPIOB;
  gpio_init_type g;
  uint32_t idle_scl, idle_sda, ack;
  uint8_t byte;
  int i;

  gpio_default_para_init(&g);
  g.gpio_pins = GPIO_PINS_6 | GPIO_PINS_7;
  g.gpio_mode = GPIO_MODE_OUTPUT;
  g.gpio_out_type = GPIO_OUTPUT_OPEN_DRAIN;
  g.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  g.gpio_pull = GPIO_PULL_NONE;
  gpio_init(gb, &g);
  gpio_bits_set(gb, GPIO_PINS_6 | GPIO_PINS_7);
  bb_delay();

  idle_scl = (gb->idt >> 6) & 1u;
  idle_sda = (gb->idt >> 7) & 1u;

  /* START: SDA falls while SCL high */
  gpio_bits_reset(gb, GPIO_PINS_7);
  bb_delay();
  gpio_bits_reset(gb, GPIO_PINS_6);
  bb_delay();

  byte = (uint8_t)((addr << 1) | 1u); /* read */
  for (i = 7; i >= 0; i--) {
    if (byte & (1u << i)) gpio_bits_set(gb, GPIO_PINS_7);
    else                  gpio_bits_reset(gb, GPIO_PINS_7);
    bb_delay();
    gpio_bits_set(gb, GPIO_PINS_6);  /* SCL high: slave samples */
    bb_delay();
    gpio_bits_reset(gb, GPIO_PINS_6);
    bb_delay();
  }

  /* ACK slot: release SDA, raise SCL, sample */
  gpio_bits_set(gb, GPIO_PINS_7);
  bb_delay();
  gpio_bits_set(gb, GPIO_PINS_6);
  bb_delay();
  ack = ((gb->idt >> 7) & 1u) ? 0u : 0x100u;
  gpio_bits_reset(gb, GPIO_PINS_6);
  bb_delay();

  /* STOP: SDA rises while SCL high */
  gpio_bits_reset(gb, GPIO_PINS_7);
  bb_delay();
  gpio_bits_set(gb, GPIO_PINS_6);
  bb_delay();
  gpio_bits_set(gb, GPIO_PINS_7);
  bb_delay();

  return (idle_sda << 1) | idle_scl | ack;
}

/* Hard-reset the I2C1 peripheral via CRM, then full re-init. Unwedges
 * BUSY-stuck / SDA-held states. */
void fnb58_i2c_hard_reset(void)
{
  crm_periph_reset(CRM_I2C1_PERIPH_RESET, TRUE);
  crm_periph_reset(CRM_I2C1_PERIPH_RESET, FALSE);
  fnb58_i2c_init();
}

/* Probe after a hard reset. Returns hw status. */
uint32_t fnb58_i2c_probe2(uint16_t address)
{
  fnb58_i2c_hard_reset();
  return fnb58_i2c_probe(address);
}

/* Raw status words (sts1 low 16 bits, sts2 high 16 bits). */
uint32_t fnb58_i2c_sts12(void)
{
  return ((uint32_t)I2C1->sts2 << 16) | I2C1->sts1;
}

/*!
 * Bit-bang a full 16-bit register read (INA226-style): START, addr+W,
 * reg, RESTART, addr+R, 2 data bytes (ACK,NAK), STOP.
 * Returns 0 on success, nonzero on NAK; *val gets the big-endian value.
 * Steals PB6/PB7 as GPIO; caller should re-init hw I2C afterwards.
 */
uint32_t fnb58_i2c_bitbang_read16(uint16_t addr, uint8_t reg, uint16_t *val)
{
  gpio_type *gb = GPIOB;
  gpio_init_type g;
  uint8_t i;
  uint16_t v = 0;

  gpio_default_para_init(&g);
  g.gpio_pins = GPIO_PINS_6 | GPIO_PINS_7;
  g.gpio_mode = GPIO_MODE_OUTPUT;
  g.gpio_out_type = GPIO_OUTPUT_OPEN_DRAIN;
  g.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  g.gpio_pull = GPIO_PULL_NONE;
  gpio_init(gb, &g);
  gpio_bits_set(gb, GPIO_PINS_6 | GPIO_PINS_7);
  bb_delay();

  /* helper via macros for brevity */
#define BB_SCL_HIGH() gpio_bits_set(gb, GPIO_PINS_6)
#define BB_SCL_LOW()  gpio_bits_reset(gb, GPIO_PINS_6)
#define BB_SDA_HIGH() gpio_bits_set(gb, GPIO_PINS_7)
#define BB_SDA_LOW()  gpio_bits_reset(gb, GPIO_PINS_7)
#define BB_SDA_READ() ((gb->idt >> 7) & 1u)

  /* send one byte, return ACK (1 = acked) */
  int send_byte(uint8_t b)
  {
    int bit, ack;
    for (bit = 7; bit >= 0; bit--) {
      if (b & (1u << bit)) BB_SDA_HIGH(); else BB_SDA_LOW();
      bb_delay();
      BB_SCL_HIGH(); bb_delay();
      BB_SCL_LOW(); bb_delay();
    }
    BB_SDA_HIGH(); bb_delay();
    BB_SCL_HIGH(); bb_delay();
    ack = BB_SDA_READ() ? 0 : 1;
    BB_SCL_LOW(); bb_delay();
    return ack;
  }

  int read_byte(int last)
  {
    int bit, r = 0;
    BB_SDA_HIGH();
    for (bit = 7; bit >= 0; bit--) {
      BB_SCL_HIGH(); bb_delay();
      r = (r << 1) | (int)BB_SDA_READ();
      BB_SCL_LOW(); bb_delay();
    }
    if (last) BB_SDA_HIGH(); else BB_SDA_LOW();
    bb_delay();
    BB_SCL_HIGH(); bb_delay();
    BB_SCL_LOW(); bb_delay();
    BB_SDA_HIGH();
    return r;
  }

  uint32_t fail = 0;

  /* START */
  BB_SDA_LOW(); bb_delay();
  BB_SCL_LOW(); bb_delay();

  if (!send_byte((uint8_t)(addr << 1))) fail = 1;      /* addr + W */
  if (!fail && !send_byte(reg)) fail = 2;               /* reg */

  if (!fail) {
    /* repeated START */
    BB_SDA_HIGH(); bb_delay();
    BB_SCL_HIGH(); bb_delay();
    BB_SDA_LOW(); bb_delay();
    BB_SCL_LOW(); bb_delay();

    if (!send_byte((uint8_t)((addr << 1) | 1u))) fail = 3; /* addr + R */
    else {
      int hi = read_byte(0);
      int lo = read_byte(1);
      v = (uint16_t)((hi << 8) | (uint8_t)lo);
    }
  }

  /* STOP */
  BB_SDA_LOW(); bb_delay();
  BB_SCL_HIGH(); bb_delay();
  BB_SDA_HIGH(); bb_delay();

  if (!fail && val) *val = v;

#undef BB_SCL_HIGH
#undef BB_SCL_LOW
#undef BB_SDA_HIGH
#undef BB_SDA_LOW
#undef BB_SDA_READ
  return fail;
}

/*!
 * Bit-bang a 16-bit register write: START, addr+W, reg, hi, lo, STOP.
 * Returns 0 on success. Same pin discipline as bitbang_read16.
 */
uint32_t fnb58_i2c_bitbang_write16(uint16_t addr, uint8_t reg, uint16_t val)
{
  gpio_type *gb = GPIOB;
  gpio_init_type g;
  uint32_t fail = 0;

  gpio_default_para_init(&g);
  g.gpio_pins = GPIO_PINS_6 | GPIO_PINS_7;
  g.gpio_mode = GPIO_MODE_OUTPUT;
  g.gpio_out_type = GPIO_OUTPUT_OPEN_DRAIN;
  g.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  g.gpio_pull = GPIO_PULL_NONE;
  gpio_init(gb, &g);
  gpio_bits_set(gb, GPIO_PINS_6 | GPIO_PINS_7);
  bb_delay();

#define BB_SCL_HIGH() gpio_bits_set(gb, GPIO_PINS_6)
#define BB_SCL_LOW()  gpio_bits_reset(gb, GPIO_PINS_6)
#define BB_SDA_HIGH() gpio_bits_set(gb, GPIO_PINS_7)
#define BB_SDA_LOW()  gpio_bits_reset(gb, GPIO_PINS_7)
#define BB_SDA_READ() ((gb->idt >> 7) & 1u)

  int send_byte(uint8_t b)
  {
    int bit, ack;
    for (bit = 7; bit >= 0; bit--) {
      if (b & (1u << bit)) BB_SDA_HIGH(); else BB_SDA_LOW();
      bb_delay();
      BB_SCL_HIGH(); bb_delay();
      BB_SCL_LOW(); bb_delay();
    }
    BB_SDA_HIGH(); bb_delay();
    BB_SCL_HIGH(); bb_delay();
    ack = BB_SDA_READ() ? 0 : 1;
    BB_SCL_LOW(); bb_delay();
    return ack;
  }

  /* START */
  BB_SDA_LOW(); bb_delay();
  BB_SCL_LOW(); bb_delay();

  if (!send_byte((uint8_t)(addr << 1))) fail = 1;
  else if (!send_byte(reg)) fail = 2;
  else if (!send_byte((uint8_t)(val >> 8))) fail = 3;
  else if (!send_byte((uint8_t)(val & 0xFF))) fail = 4;

  /* STOP */
  BB_SDA_LOW(); bb_delay();
  BB_SCL_HIGH(); bb_delay();
  BB_SDA_HIGH(); bb_delay();

#undef BB_SCL_HIGH
#undef BB_SCL_LOW
#undef BB_SDA_HIGH
#undef BB_SDA_LOW
#undef BB_SDA_READ
  return fail;
}

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

/* 0 = ACK (device present), nonzero = vendor error status. */
uint32_t fnb58_i2c_probe(uint16_t address)
{
  uint8_t b;
  return i2c_master_receive(&fnb_hi2c, address, &b, 1, FNB_I2C_TIMEOUT);
}

/* Read `len` bytes starting at 8-bit register `reg`. 0 = OK. */
uint32_t fnb58_i2c_read(uint16_t address, uint8_t reg, uint8_t *out, uint16_t len)
{
  return i2c_memory_read(&fnb_hi2c, I2C_MEM_ADDR_WIDIH_8, address, reg, out, len, FNB_I2C_TIMEOUT);
}

/* Write `len` bytes to 8-bit register `reg`. 0 = OK. */
uint32_t fnb58_i2c_write(uint16_t address, uint8_t reg, const uint8_t *data, uint16_t len)
{
  return i2c_memory_write(&fnb_hi2c, I2C_MEM_ADDR_WIDIH_8, address, reg,
                          (uint8_t *)data, len, FNB_I2C_TIMEOUT);
}

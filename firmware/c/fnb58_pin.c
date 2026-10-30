/*! @file fnb58_pin.c
 * Vendor GPIO driver glue for pin tracing.
 * Uses Artery's official gpio.c driver - no register guessing.
 */

#include "at32f403a_407.h"

/*! Configure pin(s) as output push-pull (moderate drive) */
void fnb58_pin_output(uint32_t gpio_base, uint16_t pin_mask)
{
  gpio_init_type init;
  gpio_default_para_init(&init);
  init.gpio_pins = pin_mask;
  init.gpio_mode = GPIO_MODE_OUTPUT;
  init.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  init.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  init.gpio_pull = GPIO_PULL_NONE;
  gpio_init((gpio_type *)gpio_base, &init);
}

/*! Set pin(s) HIGH */
void fnb58_pin_high(uint32_t gpio_base, uint16_t pin_mask)
{
  gpio_type *p = (gpio_type *)gpio_base;
  p->scr = pin_mask; /* Set bits via SCR register */
}

/*! Set pin(s) LOW */
void fnb58_pin_low(uint32_t gpio_base, uint16_t pin_mask)
{
  gpio_type *p = (gpio_type *)gpio_base;
  p->clr = pin_mask; /* Clear bits via CLR register */
}

/*! Configure pin(s) as floating input (reset state) */
void fnb58_pin_input(uint32_t gpio_base, uint16_t pin_mask)
{
  gpio_init_type init;
  gpio_default_para_init(&init);
  init.gpio_pins = pin_mask;
  init.gpio_mode = GPIO_MODE_INPUT;
  init.gpio_pull = GPIO_PULL_NONE;
  gpio_init((gpio_type *)gpio_base, &init);
}

/*! Enable GPIOA/GPIOB clocks via vendor CRM driver */
void fnb58_gpio_clocks(void)
{
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
}

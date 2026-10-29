//! Vendor USB stack glue for the FNB58 bring-up.
//!
//! Mirrors the vendor `virtual_comport` example: CDC class on the USBFS
//! cell, HICK-48 MHz USB clock with ACC auto-calibration (crystalless,
//! the vendor's own recipe), interrupt-driven (USBFS lines L/H both
//! wired to the same handler). The Rust side calls `fnb58_usb_init`
//! once and `fnb58_usb_poll` from its main loop; echo is done via the
//! vendor `usb_vcp_get_rxdata`/`usb_vcp_send_data` API.

#include "at32f403a_407_conf.h"
#include "usb_conf.h"
#include "usbd_core.h"
#include "usbd_int.h"
#include "cdc_class.h"
#include "cdc_desc.h"

usbd_core_type usb_core_dev;

extern void fnb58_usbd_event_hook(void);

static void usb_clock48m_hick_acc(void)
{
  /* vendor crm_usb_clock_source_select(HICK) equivalent, done by the Rust
     clock setup (HICKDIV=NODIV + HICK_TO_SCLK + HICK_TO_USB). Here: only
     the ACC trim, exactly as the vendor virtual_comport example does. */
  crm_periph_clock_enable(CRM_ACC_PERIPH_CLOCK, TRUE);

  /* update the c1\c2\c3 value (vendor constants for 48 MHz HICK) */
  acc_write_c1(7980);
  acc_write_c2(8000);
  acc_write_c3(8020);

  /* open acc calibration */
  acc_calibration_mode_enable(ACC_CAL_HICKTRIM, TRUE);
}

void fnb58_usb_init(void)
{
  usb_clock48m_hick_acc();

  /* enable usb clock */
  crm_periph_clock_enable(CRM_USB_PERIPH_CLOCK, TRUE);

  /* reset the usb cell (the vendor BSP does this between clock setup and
     usbd_core_init when switching clock sources) */
  crm_periph_reset(CRM_USB_PERIPH_CLOCK, TRUE);
  crm_periph_reset(CRM_USB_PERIPH_CLOCK, FALSE);

  usbd_core_init(&usb_core_dev, USB, &cdc_class_handler, &cdc_desc_handler, 0);

  usbd_connect(&usb_core_dev);
}

uint16_t fnb58_usb_recv(uint8_t *buffer)
{
  return usb_vcp_get_rxdata(&usb_core_dev, buffer);
}

error_status fnb58_usb_send(const uint8_t *data, uint16_t len)
{
  return usb_vcp_send_data(&usb_core_dev, (uint8_t *)data, len);
}

/* USB interrupt lines: the vendor example uses the L line; H is wired to
   the same handler so either remap state works. */
void USBFS_L_CAN1_RX0(void)
{
  usbd_irq_handler(&usb_core_dev);
  fnb58_usbd_event_hook();
}

void USBFS_H_CAN1_TX(void)
{
  usbd_irq_handler(&usb_core_dev);
  fnb58_usbd_event_hook();
}

/* ACC calibration-ready / reference-lost service (vendor acc example).
   The vector symbol must be named ACC, which collides with the vendor's
   peripheral-pointer macro - drop the macro for this translation unit. */
#undef ACC
void ACC(void)
{
  if(acc_interrupt_flag_get(ACC_CALRDY_FLAG) != RESET)
  {
    acc_flag_clear(ACC_CALRDY_FLAG);
  }
  if(acc_interrupt_flag_get(ACC_RSLOST_FLAG) != RESET)
  {
    acc_flag_clear(ACC_RSLOST_FLAG);
  }
}

/* The vendor CDC class applies new line coding through this example-side
   hook; the FNB58 bridge has no USART behind the CDC, so it is a no-op. */
void usb_usart_config(linecoding_type linecoding)
{
  (void)linecoding;
}

/* Vendor usbd_core.c calls usb_delay_ms between init steps. */
volatile uint32_t fnb58_delay_ticks = 0;

void usb_delay_ms(uint32_t ms)
{
  /* ~ms at 48-192 MHz; the vendor uses this for a 10 ms settle wait. */
  while(ms--)
  {
    for(volatile uint32_t i = 0; i < 48000; i++)
    {
      __asm volatile ("nop");
    }
  }
}

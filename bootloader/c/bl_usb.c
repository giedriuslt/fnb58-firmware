//! Vendor USB CDC glue for the bootloader — `firmware/c/fnb58_usb.c`
//! with the HID mode, the ACC service and the event hook removed. Init
//! order is the vendor virtual_comport example verbatim (clock enable,
//! usbd_core_init, connect), which is the recipe proven on this board
//! since round 15.

#include "at32f403a_407_conf.h"
#include "usb_conf.h"
#include "usbd_core.h"
#include "usbd_int.h"
#include "cdc_class.h"
#include "cdc_desc.h"

usbd_core_type usb_core_dev;

void bl_usb_init(void)
{
  /* The 48 MHz USB clock is already programmed by the Rust clock setup:
     192 MHz PLL from the 8 MHz crystal with CRM_USB_DIV_4. */
  crm_periph_clock_enable(CRM_USB_PERIPH_CLOCK, TRUE);

  usbd_core_init(&usb_core_dev, USB, &cdc_class_handler, &cdc_desc_handler, 0);

  usbd_connect(&usb_core_dev);
}

uint16_t bl_usb_recv(uint8_t *buffer)
{
  return usb_vcp_get_rxdata(&usb_core_dev, buffer);
}

/* Rust contract: 0 = packet accepted, nonzero = endpoint busy
 * (usb_vcp_send_data returns the vendor error_status encoding, where
 * ERROR = 0 and SUCCESS = 1 — translated here, same as the app glue). */
int32_t bl_usb_send(const uint8_t *data, uint16_t len)
{
  return (usb_vcp_send_data(&usb_core_dev, (uint8_t *)data, len) == SUCCESS) ? 0 : 1;
}

/* USB interrupt lines: the vendor example uses the L line; H is wired to
   the same handler so either remap state works. */
void USBFS_L_CAN1_RX0(void)
{
  usbd_irq_handler(&usb_core_dev);
}

void USBFS_H_CAN1_TX(void)
{
  usbd_irq_handler(&usb_core_dev);
}

/* The vendor CDC class applies new line coding through this example-side
   hook; the bootloader has no USART behind the CDC, so it is a no-op. */
void usb_usart_config(linecoding_type linecoding)
{
  (void)linecoding;
}

/* Returns 1 when the USB host has configured the device (CDC ready). */
uint32_t bl_usb_configured(void)
{
  return (usb_core_dev.conn_state == USB_CONN_STATE_CONFIGURED) ? 1 : 0;
}

/* Vendor usbd_core.c calls usb_delay_ms between init steps. */
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

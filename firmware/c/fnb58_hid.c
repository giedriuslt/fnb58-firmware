//! FNB58 vendor HID class (r134) — the PC-link USB device.
//!
//! Enumerates as the stock FNB58 measurement interface so the OpenFNB58
//! tools (github.com/yeckel/OpenFNB58) work unmodified: VID 0x2E3C,
//! PID 0x5558, one HID interface, interrupt endpoints 0x83 (IN) and
//! 0x03 (OUT), 64-byte un-numbered input/output reports.
//!
//! Protocol (64-byte reports, CRC-8 poly 0x39 init 0x42 over bytes
//! 1..62 in byte 63): host commands AA 81 (INIT1) / AA 82 (INIT2) /
//! AA 83 (POLL); device streams AA 04 + 4 x 15-byte samples per report
//! (voltage u32/100000 V, current u32/100000 A, D+ u16/1000 V,
//! D- u16/1000 V, unknown u8, temp u16/10 C).
//!
//! This file is USB mechanics only: it owns descriptors, endpoints and
//! a command mailbox. The Rust main loop samples the meter, builds the
//! AA 04 reports and calls fnb58_hid_tx / fnb58_hid_rx.

#include "at32f403a_407_conf.h"
#include "usb_conf.h"
#include "usbd_core.h"
#include "usbd_int.h"
#include "usbd_sdr.h"

/* Defined in fnb58_usb.c — one USB core, shared by both classes. */
extern usbd_core_type usb_core_dev;

/* Endpoints mirror the stock device (per the OpenFNB58 pyusb notes:
   interface 3, endpoints 0x03/0x83). EP3 buffer slots reuse the CDC
   EP1 slots — in HID mode only EP0 and EP3 exist, and the 512-byte
   packet buffer holds EP0 (0x40..0xBF) + these two 64-byte slots. */
#define FNB58_HID_IN_EPT   0x83
#define FNB58_HID_OUT_EPT  0x03
#define FNB58_HID_IN_ADDR  0xC0
#define FNB58_HID_OUT_ADDR 0x100
#define FNB58_HID_PS       64

#define FNB58_VID 0x2E3C
#define FNB58_PID 0x5558

#define MCU_ID1 (0x1FFFF7E8)
#define MCU_ID2 (0x1FFFF7EC)
#define MCU_ID3 (0x1FFFF7F0)

static usb_sts_type hid_init_handler(void *udev);
static usb_sts_type hid_clear_handler(void *udev);
static usb_sts_type hid_setup_handler(void *udev, usb_setup_type *setup);
static usb_sts_type hid_ept0_tx_handler(void *udev);
static usb_sts_type hid_ept0_rx_handler(void *udev);
static usb_sts_type hid_in_handler(void *udev, uint8_t ept_num);
static usb_sts_type hid_out_handler(void *udev, uint8_t ept_num);
static usb_sts_type hid_sof_handler(void *udev);
static usb_sts_type hid_event_handler(void *udev, usbd_event_type event);

static usbd_desc_t *hid_get_device_descriptor(void);
static usbd_desc_t *hid_get_device_qualifier(void);
static usbd_desc_t *hid_get_device_configuration(void);
static usbd_desc_t *hid_get_device_other_speed(void);
static usbd_desc_t *hid_get_device_lang_id(void);
static usbd_desc_t *hid_get_device_manufacturer_string(void);
static usbd_desc_t *hid_get_device_product_string(void);
static usbd_desc_t *hid_get_device_serial_string(void);
static usbd_desc_t *hid_get_device_interface_string(void);
static usbd_desc_t *hid_get_device_config_string(void);

/* ------------------------------------------------------------------ */
/* class state                                                         */
/* ------------------------------------------------------------------ */

typedef struct
{
  volatile uint8_t tx_completed; /* IN report handed to the engine      */
  volatile uint8_t rx_ready;     /* OUT report waiting in cmd_mailbox   */
  uint8_t cmd_mailbox[FNB58_HID_PS]; /* latest host command (OUT EP)    */
  uint8_t ctrl_rx[FNB58_HID_PS];    /* SET_REPORT data stage buffer     */
  volatile uint16_t ctrl_rx_len;    /* pending SET_REPORT length        */
  uint8_t alt_setting;
} fnb58_hid_state_type;

static fnb58_hid_state_type g_hid;

/* usbd_core_init copies these pointers; the struct must outlive it. */
usbd_class_handler fnb58_hid_class_handler =
{
  hid_init_handler,
  hid_clear_handler,
  hid_setup_handler,
  hid_ept0_tx_handler,
  hid_ept0_rx_handler,
  hid_in_handler,
  hid_out_handler,
  hid_sof_handler,
  hid_event_handler,
  &g_hid
};

/* ------------------------------------------------------------------ */
/* descriptors                                                         */
/* ------------------------------------------------------------------ */

/* Generic 64-byte in/out reports, vendor usage page, no report IDs
   (matches what both OpenFNB58 tools expect to read and write). */
static const uint8_t g_hid_report_desc[34] =
{
  0x06, 0x00, 0xFF,       /* Usage Page (Vendor Defined 0xFF00)   */
  0x09, 0x01,             /* Usage (0x01)                          */
  0xA1, 0x01,             /* Collection (Application)              */
  0x09, 0x02,             /*   Usage (0x02) - input report         */
  0x15, 0x00,             /*   Logical Minimum (0)                 */
  0x26, 0xFF, 0x00,       /*   Logical Maximum (255)               */
  0x75, 0x08,             /*   Report Size (8)                     */
  0x95, 0x40,             /*   Report Count (64)                   */
  0x81, 0x00,             /*   Input (Data,Var,Abs)                */
  0x09, 0x03,             /*   Usage (0x03) - output report        */
  0x15, 0x00,             /*   Logical Minimum (0)                 */
  0x26, 0xFF, 0x00,       /*   Logical Maximum (255)               */
  0x75, 0x08,             /*   Report Size (8)                     */
  0x95, 0x40,             /*   Report Count (64)                   */
  0x91, 0x00,             /*   Output (Data,Var,Abs)               */
  0xC0                    /* End Collection                        */
};

#define FNB58_HID_CONFIG_DESC_SIZE 41

static uint8_t g_hid_device_desc[18] =
{
  0x12,                   /* bLength */
  0x01,                   /* bDescriptorType (DEVICE) */
  0x00, 0x02,             /* bcdUSB 2.00 */
  0x00,                   /* bDeviceClass (per-interface) */
  0x00,                   /* bDeviceSubClass */
  0x00,                   /* bDeviceProtocol */
  0x40,                   /* bMaxPacketSize0 */
  0x3C, 0x2E,             /* idVendor  0x2E3C (FNIRSI) */
  0x58, 0x55,             /* idProduct 0x5558 (FNB58) */
  0x00, 0x01,             /* bcdDevice 1.00 */
  0x01,                   /* iManufacturer */
  0x02,                   /* iProduct */
  0x03,                   /* iSerialNumber */
  0x01                    /* bNumConfigurations */
};

static uint8_t g_hid_config_desc[FNB58_HID_CONFIG_DESC_SIZE] =
{
  /* configuration */
  0x09,                   /* bLength */
  0x02,                   /* bDescriptorType (CONFIGURATION) */
  0x29, 0x00,             /* wTotalLength 41 */
  0x01,                   /* bNumInterfaces */
  0x01,                   /* bConfigurationValue */
  0x04,                   /* iConfiguration */
  0x80,                   /* bmAttributes (bus powered) */
  0x32,                   /* MaxPower 100 mA */

  /* interface 0: HID */
  0x09,                   /* bLength */
  0x04,                   /* bDescriptorType (INTERFACE) */
  0x00,                   /* bInterfaceNumber */
  0x00,                   /* bAlternateSetting */
  0x02,                   /* bNumEndpoints */
  0x03,                   /* bInterfaceClass (HID) */
  0x00,                   /* bInterfaceSubClass (none) */
  0x00,                   /* bInterfaceProtocol (none) */
  0x05,                   /* iInterface */

  /* HID class descriptor */
  0x09,                   /* bLength */
  0x21,                   /* bDescriptorType (HID) */
  0x11, 0x01,             /* bcdHID 1.11 */
  0x00,                   /* bCountryCode */
  0x01,                   /* bNumDescriptors */
  0x22,                   /* bDescriptorType (REPORT) */
  0x22, 0x00,             /* wDescriptorLength 34 */

  /* endpoint 0x83 IN, interrupt */
  0x07,                   /* bLength */
  0x05,                   /* bDescriptorType (ENDPOINT) */
  FNB58_HID_IN_EPT,       /* bEndpointAddress */
  0x03,                   /* bmAttributes (interrupt) */
  FNB58_HID_PS, 0x00,     /* wMaxPacketSize 64 */
  0x01,                   /* bInterval 1 ms */

  /* endpoint 0x03 OUT, interrupt */
  0x07,                   /* bLength */
  0x05,                   /* bDescriptorType (ENDPOINT) */
  FNB58_HID_OUT_EPT,      /* bEndpointAddress */
  0x03,                   /* bmAttributes (interrupt) */
  FNB58_HID_PS, 0x00,     /* wMaxPacketSize 64 */
  0x01                    /* bInterval 1 ms */
};

static uint8_t g_string_lang_id[4] =
{
  0x04, 0x03, 0x09, 0x04
};

#define FNB58_HID_SIZ_STRING_SERIAL 26
static uint8_t g_string_serial[FNB58_HID_SIZ_STRING_SERIAL] =
{
  FNB58_HID_SIZ_STRING_SERIAL, 0x03
};

static uint8_t g_desc_scratch[64];

static usbd_desc_t hid_device_descriptor = { 18, g_hid_device_desc };
static usbd_desc_t hid_config_descriptor = { FNB58_HID_CONFIG_DESC_SIZE, g_hid_config_desc };
static usbd_desc_t hid_langid_descriptor = { 4, g_string_lang_id };
static usbd_desc_t hid_serial_descriptor = { FNB58_HID_SIZ_STRING_SERIAL, g_string_serial };
static usbd_desc_t hid_report_descriptor = { sizeof(g_hid_report_desc), (uint8_t *)g_hid_report_desc };
static usbd_desc_t hid_class_descriptor = { 9, &g_hid_config_desc[18] };
static usbd_desc_t hid_scratch_descriptor;

usbd_desc_handler fnb58_hid_desc_handler =
{
  hid_get_device_descriptor,
  hid_get_device_qualifier,
  hid_get_device_configuration,
  hid_get_device_other_speed,
  hid_get_device_lang_id,
  hid_get_device_manufacturer_string,
  hid_get_device_product_string,
  hid_get_device_serial_string,
  hid_get_device_interface_string,
  hid_get_device_config_string
};

/* ------------------------------------------------------------------ */
/* class handlers                                                      */
/* ------------------------------------------------------------------ */

static usb_sts_type hid_init_handler(void *udev)
{
  usbd_core_type *pudev = (usbd_core_type *)udev;

  usbd_ept_buf_custom_define(pudev, FNB58_HID_IN_EPT, FNB58_HID_IN_ADDR);
  usbd_ept_buf_custom_define(pudev, FNB58_HID_OUT_EPT, FNB58_HID_OUT_ADDR);
  usbd_ept_open(pudev, FNB58_HID_IN_EPT, EPT_INT_TYPE, FNB58_HID_PS);
  usbd_ept_open(pudev, FNB58_HID_OUT_EPT, EPT_INT_TYPE, FNB58_HID_PS);

  g_hid.tx_completed = 1;
  g_hid.rx_ready = 0;
  g_hid.alt_setting = 0;
  usbd_ept_recv(pudev, FNB58_HID_OUT_EPT, g_hid.cmd_mailbox, FNB58_HID_PS);

  return USB_OK;
}

static usb_sts_type hid_clear_handler(void *udev)
{
  usbd_core_type *pudev = (usbd_core_type *)udev;
  usbd_ept_close(pudev, FNB58_HID_IN_EPT);
  usbd_ept_close(pudev, FNB58_HID_OUT_EPT);
  return USB_OK;
}

static usb_sts_type hid_setup_handler(void *udev, usb_setup_type *setup)
{
  usbd_core_type *pudev = (usbd_core_type *)udev;

  switch(setup->bmRequestType & USB_REQ_TYPE_RESERVED)
  {
    case USB_REQ_TYPE_STANDARD:
      switch(setup->bRequest)
      {
        case USB_STD_REQ_GET_DESCRIPTOR:
          /* wValue high byte selects the HID descriptor type. These
             arrive as interface-recipient requests and land here via
             usbd_interface_request; the device-recipient descriptor
             path in usbd_sdr.c never sees them. */
          if((setup->bmRequestType & 0x1F) == 0x01)
          {
            switch(setup->wValue >> 8)
            {
              case 0x21: /* HID class descriptor */
                usbd_ctrl_send(pudev, hid_class_descriptor.descriptor,
                               MIN(hid_class_descriptor.length, setup->wLength));
                break;
              case 0x22: /* HID report descriptor */
                usbd_ctrl_send(pudev, hid_report_descriptor.descriptor,
                               MIN(hid_report_descriptor.length, setup->wLength));
                break;
              default:
                usbd_ctrl_unsupport(pudev);
                break;
            }
          }
          else
          {
            usbd_ctrl_unsupport(pudev);
          }
          break;
        case USB_STD_REQ_GET_INTERFACE:
        {
          static uint8_t if_alt = 0;
          if_alt = g_hid.alt_setting;
          usbd_ctrl_send(pudev, &if_alt, 1);
          break;
        }
        case USB_STD_REQ_SET_INTERFACE:
          g_hid.alt_setting = (uint8_t)setup->wValue;
          break;
        default:
          usbd_ctrl_unsupport(pudev);
          break;
      }
      break;

    case USB_REQ_TYPE_CLASS:
      switch(setup->bRequest)
      {
        case 0x01: /* GET_REPORT: hand back the last stream report slot */
          if(setup->bmRequestType & USB_REQ_DIR_DTH)
          {
            static uint8_t report[FNB58_HID_PS];
            usbd_ctrl_send(pudev, report, MIN(FNB58_HID_PS, setup->wLength));
          }
          else
          {
            usbd_ctrl_unsupport(pudev);
          }
          break;
        case 0x02: /* GET_IDLE */
        {
          static uint8_t idle = 0;
          if(setup->bmRequestType & USB_REQ_DIR_DTH)
          {
            usbd_ctrl_send(pudev, &idle, 1);
          }
          else
          {
            usbd_ctrl_unsupport(pudev);
          }
          break;
        }
        case 0x0A: /* SET_IDLE: accept, no data stage */
          break;
        case 0x09: /* SET_REPORT: hosts without an OUT endpoint write
                      commands here; feed them to the same mailbox */
          if(setup->wLength)
          {
            uint16_t len = setup->wLength;
            if(len > FNB58_HID_PS)
            {
              len = FNB58_HID_PS;
            }
            g_hid.ctrl_rx_len = len;
            usbd_ctrl_recv(pudev, g_hid.ctrl_rx, len);
          }
          break;
        default:
          usbd_ctrl_unsupport(pudev);
          break;
      }
      break;

    default:
      usbd_ctrl_unsupport(pudev);
      break;
  }
  return USB_OK;
}

static usb_sts_type hid_ept0_tx_handler(void *udev)
{
  (void)udev;
  return USB_OK;
}

static usb_sts_type hid_ept0_rx_handler(void *udev)
{
  usbd_core_type *pudev = (usbd_core_type *)udev;
  if(g_hid.ctrl_rx_len)
  {
    /* SET_REPORT data stage completed: deliver it like an OUT report. */
    uint16_t len = usbd_get_recv_len(pudev, 0);
    if(len > g_hid.ctrl_rx_len)
    {
      len = g_hid.ctrl_rx_len;
    }
    if(len == FNB58_HID_PS)
    {
      for(uint16_t i = 0; i < FNB58_HID_PS; i++)
      {
        g_hid.cmd_mailbox[i] = g_hid.ctrl_rx[i];
      }
      g_hid.rx_ready = 1;
    }
    g_hid.ctrl_rx_len = 0;
  }
  return USB_OK;
}

/* usbd_eptn_handler passes the endpoint NUMBER (3), not the address
   (0x83/0x03) - mask both sides so the compare is correct either way
   (usbd_core.c's own `ept_addr & 0x7F` idiom). r134 compared the IN
   handler against 0x83 and never matched: tx_completed never set
   after the first report, so exactly one report was ever sent. */
static usb_sts_type hid_in_handler(void *udev, uint8_t ept_num)
{
  (void)udev;
  if((ept_num & 0x7F) == (FNB58_HID_IN_EPT & 0x7F))
  {
    g_hid.tx_completed = 1;
  }
  return USB_OK;
}

static usb_sts_type hid_out_handler(void *udev, uint8_t ept_num)
{
  usbd_core_type *pudev = (usbd_core_type *)udev;
  if((ept_num & 0x7F) == (FNB58_HID_OUT_EPT & 0x7F))
  {
    uint16_t len = usbd_get_recv_len(pudev, ept_num);
    /* A short packet cannot be a command; re-arm and drop it. */
    if(len == FNB58_HID_PS)
    {
      g_hid.rx_ready = 1; /* mailbox holds the full 64-byte report */
    }
    usbd_ept_recv(pudev, FNB58_HID_OUT_EPT, g_hid.cmd_mailbox, FNB58_HID_PS);
  }
  return USB_OK;
}

static usb_sts_type hid_sof_handler(void *udev)
{
  (void)udev;
  return USB_OK;
}

static usb_sts_type hid_event_handler(void *udev, usbd_event_type event)
{
  (void)udev;
  switch(event)
  {
    case USBD_RESET_EVENT:
      g_hid.tx_completed = 1;
      g_hid.rx_ready = 0;
      break;
    default:
      break;
  }
  return USB_OK;
}

/* ------------------------------------------------------------------ */
/* descriptor handlers                                                 */
/* ------------------------------------------------------------------ */

static uint16_t hid_unicode_convert(const char *string, uint8_t *unicode_buf)
{
  uint16_t str_len = 0, id_pos = 2;
  while(*string != '\0')
  {
    unicode_buf[id_pos++] = (uint8_t)*string++;
    unicode_buf[id_pos++] = 0x00;
    str_len++;
  }
  str_len = str_len * 2 + 2;
  unicode_buf[0] = (uint8_t)str_len;
  unicode_buf[1] = 0x03;
  return str_len;
}

static usbd_desc_t *hid_string(const char *s)
{
  hid_scratch_descriptor.length = hid_unicode_convert(s, g_desc_scratch);
  hid_scratch_descriptor.descriptor = g_desc_scratch;
  return &hid_scratch_descriptor;
}

static usbd_desc_t *hid_get_device_descriptor(void)
{
  return &hid_device_descriptor;
}

static usbd_desc_t *hid_get_device_qualifier(void)
{
  return NULL;
}

static usbd_desc_t *hid_get_device_configuration(void)
{
  return &hid_config_descriptor;
}

static usbd_desc_t *hid_get_device_other_speed(void)
{
  return NULL;
}

static usbd_desc_t *hid_get_device_lang_id(void)
{
  return &hid_langid_descriptor;
}

static usbd_desc_t *hid_get_device_manufacturer_string(void)
{
  return hid_string("FNIRSI");
}

static usbd_desc_t *hid_get_device_product_string(void)
{
  return hid_string("FNB58");
}

static usbd_desc_t *hid_get_device_serial_string(void)
{
  /* Same MCU-unique-ID serial scheme as the vendor CDC descriptors. */
  uint32_t s0 = *(uint32_t *)MCU_ID1 + *(uint32_t *)MCU_ID3;
  uint32_t s1 = *(uint32_t *)MCU_ID2;
  for(uint8_t i = 0; i < 8; i++)
  {
    uint8_t nib = (s0 >> 28) & 0xF;
    g_string_serial[2 + 2 * i] = nib < 10 ? '0' + nib : 'A' + nib - 10;
    g_string_serial[3 + 2 * i] = 0;
    s0 <<= 4;
  }
  for(uint8_t i = 0; i < 4; i++)
  {
    uint8_t nib = (s1 >> 28) & 0xF;
    g_string_serial[18 + 2 * i] = nib < 10 ? '0' + nib : 'A' + nib - 10;
    g_string_serial[19 + 2 * i] = 0;
    s1 <<= 4;
  }
  return &hid_serial_descriptor;
}

static usbd_desc_t *hid_get_device_interface_string(void)
{
  return hid_string("FNB58 PC Link");
}

static usbd_desc_t *hid_get_device_config_string(void)
{
  return hid_string("HID");
}

/* ------------------------------------------------------------------ */
/* Rust-facing API                                                     */
/* ------------------------------------------------------------------ */

/* Send one 64-byte report on the IN endpoint. Returns 0 when accepted,
   1 while the previous report is still in flight (the caller retries
   or drops). Same ERROR/SUCCESS translation contract as
   fnb58_usb_send. */
int32_t fnb58_hid_tx(const uint8_t *report)
{
  if(g_hid.tx_completed)
  {
    g_hid.tx_completed = 0;
    usbd_ept_send(&usb_core_dev, FNB58_HID_IN_EPT, (uint8_t *)report, FNB58_HID_PS);
    return 0;
  }
  return 1;
}

/* Copy out the pending host command (a full 64-byte OUT report).
   Returns 64 when one was waiting, 0 otherwise. */
uint16_t fnb58_hid_rx(uint8_t *buffer)
{
  if(!g_hid.rx_ready)
  {
    return 0;
  }
  g_hid.rx_ready = 0;
  for(uint16_t i = 0; i < FNB58_HID_PS; i++)
  {
    buffer[i] = g_hid.cmd_mailbox[i];
  }
  return FNB58_HID_PS;
}

#ifndef __AT32F403A_407_CONF_H
#define __AT32F403A_407_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "at32f403a_407.h"

/* Clock constants the crm driver expects (vendor conf template values). */
#ifndef HEXT_VALUE
#define HEXT_VALUE               ((uint32_t)8000000)
#endif
#ifndef HICK_VALUE
#define HICK_VALUE               ((uint32_t)8000000)
#endif
#ifndef HEXT_STARTUP_TIMEOUT
#define HEXT_STARTUP_TIMEOUT     ((uint32_t)0x10000)
#endif
#include "at32f403a_407_crm.h"
#include "at32f403a_407_acc.h"
#include "at32f403a_407_usb.h"
#include "at32f403a_407_i2c.h"
#include "at32f403a_407_dma.h"
#include "at32f403a_407_i2c.h"
#include "at32f403a_407_dma.h"
#include "at32f403a_407_gpio.h"
#include "at32f403a_407_misc.h"

#ifdef __cplusplus
}
#endif
#endif

/*! @file fnb58_display.c
 * Display probe: tries SPI1, SPI2, SPI3 to find an ST7789.
 *
 * For each bus: init vendor SPI driver, send ST7789 Software Reset +
 * Sleep Out + Read Display ID, read MISO bytes. Reports over CDC.
 *
 * ST7789 Read Display ID (0x04) returns 4 bytes:
 *   00 85 21 52  (or 00 85 21 9x for some variants)
 *
 * Pin mappings (AT32F403ACGT7 LQFP48):
 *   SPI1: SCK=PA5, MISO=PA6, MOSI=PA7 (default GMUX_0000)
 *   SPI2: SCK=PB13, MISO=PB14, MOSI=PB15 (default GMUX_0000)
 *   SPI3: SCK=PB3, MISO=PB4, MOSI=PB5 (needs JTAG disable remap)
 */

#include "at32f403a_407.h"
#include <stdbool.h>

/* ST7789 commands */
#define ST7789_NOP          0x00
#define ST7789_SWRESET      0x01
#define ST7789_RDDID        0x04
#define ST7789_SLPIN        0x10
#define ST7789_SLPOUT       0x11
#define ST7789_RDDST        0x09

/* SPI register access (direct - simpler than vendor flag functions) */
#define SPI_REG(base, off) (*(volatile uint32_t *)((base) + (off)))
#define SPI_STS  0x08
#define SPI_DT   0x0C
#define SPI_CR1  0x00

/* ST7789 TDBE flag */
#define SPI_STS_TDBE  0x02

/* Simple delay */
static void disp_delay_ms(volatile uint32_t ms) {
    while (ms--) {
        for (volatile uint32_t i = 0; i < 8000; i++) { __asm volatile("nop"); }
    }
}

/* SPI bus descriptors */
typedef struct {
    uint32_t base;          /* SPI register base */
    uint32_t crm_clk;       /* CRM clock enable bit */
    uint32_t gpio_base;     /* GPIO port base for SCK/MISO/MOSI */
    uint32_t pins;          /* Pin mask for SCK|MISO|MOSI */
    const char *name;       /* Human-readable name */
} spi_bus_desc_t;

static const spi_bus_desc_t buses[] = {
    { 0x40013000, CRM_SPI1_PERIPH_CLOCK, 0x40010800, 0x0000E000, "SPI1 PA5/6/7" },
    { 0x40003800, CRM_SPI2_PERIPH_CLOCK, 0x40010C00, 0xE0000000, "SPI2 PB13/14/15" },
    { 0x40003C00, CRM_SPI3_PERIPH_CLOCK, 0x40010C00, 0x00000038, "SPI3 PB3/4/5" },
};
#define NUM_BUSES 3

/* Configure GPIO pins for SPI mux mode (AF push-pull for SCK/MOSI, AF input for MISO) */
static void spi_gpio_init(uint32_t gpio_base, uint32_t pin_mask) {
    volatile uint32_t *crl = (volatile uint32_t *)(gpio_base + 0x00);
    volatile uint32_t *crh = (volatile uint32_t *)(gpio_base + 0x04);

    for (int pin = 0; pin < 16; pin++) {
        uint32_t bit = 1UL << pin;
        if (!(pin_mask & bit)) continue;

        volatile uint32_t *reg = (pin < 8) ? crl : crh;
        uint32_t shift = (pin % 8) * 4;

        uint32_t clear_mask = ~(0xF << shift);
        uint32_t af_pp_50mhz = 0xB << shift; /* MODE=11, CNF=10 = AF push-pull 50MHz */
        *reg = (*reg & clear_mask) | af_pp_50mhz;
    }
}

/* Wait for SPI transmit buffer empty */
static int spi_wait_txe(uint32_t base) {
    for (volatile uint32_t i = 0; i < 10000; i++) {
        if (SPI_REG(base, SPI_STS) & SPI_STS_TDBE) return 0;
    }
    return -1;
}

/* Send one byte and read one byte (full duplex) */
static uint8_t spi_xfer_byte(uint32_t base, uint8_t data) {
    SPI_REG(base, SPI_DT) = data;
    /* Wait for transmit complete */
    for (volatile uint32_t i = 0; i < 10000; i++) {
        if (SPI_REG(base, SPI_STS) & SPI_STS_TDBE) break;
    }
    /* Read the received byte (MISO) */
    return (uint8_t)SPI_REG(base, SPI_DT);
}

/* Initialize one SPI bus for display probing */
static void bus_init(const spi_bus_desc_t *bus) {
    /* Enable clocks */
    crm_periph_clock_enable(bus->crm_clk, TRUE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);

    /* For SPI3, disable JTAG to free PB3/PB4/PB5 */
    if (bus->base == 0x40003C00) {
        gpio_pin_remap_config(0x00000400, TRUE); /* SWJTAG_MUX_100 = full SWJ disabled */
    }

    /* Configure GPIO pins for SPI mux mode */
    spi_gpio_init(bus->gpio_base, bus->pins);

    /* Reset the SPI peripheral */
    volatile uint32_t *cr1 = (volatile uint32_t *)(bus->base + 0x00);
    *cr1 = 0;

    /* Configure SPI: master, 8-bit, mode 0, software CS, slow clock */
    /* CR1 bits: clkpha[0], clkpol[1], msten[2], mdiv_l[5:3], spien[6],
                 ltf[7], swcsil[8], swcsen[9], ora[10], fbn[11] */
    uint32_t cr1_val = 0;
    cr1_val |= (0 << 0);        /* clkpha = 0 (first edge) */
    cr1_val |= (0 << 1);        /* clkpol = 0 (low idle) = mode 0 */
    cr1_val |= (1 << 2);        /* msten = 1 (master) */
    cr1_val |= (7 << 3);        /* mdiv_l = 7 -> DIV_256 */
    cr1_val |= (0 << 7);        /* ltf = 0 (MSB first) */
    cr1_val |= (1 << 8);        /* swcsil = 1 (internal CS high) */
    cr1_val |= (1 << 9);        /* swcsen = 1 (software CS) */
    cr1_val |= (0 << 10);       /* ora = 0 (full duplex) */
    cr1_val |= (0 << 11);       /* fbn = 0 (8-bit) */
    *cr1 = cr1_val;

    /* Enable SPI */
    *cr1 |= (1 << 6);
}

/* Probe one SPI bus for ST7789. Returns bytes read from MISO. */
void fnb58_display_probe_bus(int bus_idx, uint8_t *results) {
    if (bus_idx < 0 || bus_idx >= NUM_BUSES) return;
    const spi_bus_desc_t *bus = &buses[bus_idx];

    bus_init(bus);

    /* Flush: send 8 NOP bytes */
    for (int i = 0; i < 8; i++) {
        spi_xfer_byte(bus->base, 0x00);
    }

    /* Software Reset */
    spi_xfer_byte(bus->base, 0x01);
    disp_delay_ms(200);

    /* Sleep Out */
    spi_xfer_byte(bus->base, 0x11);
    disp_delay_ms(150);

    /* Read Display ID: send command then read 4 bytes */
    spi_xfer_byte(bus->base, 0x04);

    for (int i = 0; i < 4; i++) {
        results[i] = spi_xfer_byte(bus->base, 0x00);
    }

    /* Disable SPI */
    volatile uint32_t *cr1 = (volatile uint32_t *)(bus->base + 0x00);
    *cr1 &= ~(1 << 6);
}

/* Get bus name */
const char *fnb58_display_bus_name(int bus_idx) {
    if (bus_idx < 0 || bus_idx >= NUM_BUSES) return "?";
    return buses[bus_idx].name;
}

/* Get number of buses */
int fnb58_display_num_buses(void) {
    return NUM_BUSES;
}

/*!
 * @file fnb58_spi.c
 * Vendor SPI driver glue for ST7789 panel transport.
 *
 * Uses Artery's official SPI driver functions directly - same code as
 * their reference examples. No register guessing.
 *
 * Connections (AT32F403ACGT7 LQFP48, verified on FNB58 board):
 *   PA5  = SPI1_SCK   (pin 42)
 *   PA7  = SPI1_MOSI  (pin 41)
 *   PB0  = D/C        (panel data/command select)
 *   PB1  = RST        (panel reset)
 *   PB4  = CS         (panel chip select, active low)
 *
 * Vendor function mapping:
 *   spi_init()           -> configure SPI1 as master, 8-bit, MSB first,
 *                             mode 0, software CS
 *   spi_enable()         -> enable SPI peripheral
 *   spi->dr = data       -> transmit one byte
 *   spi->sts & TDBE      -> transmit buffer empty (ready for next byte)
 *
 * Notes:
 *   - DMA NOT used - byte-by-byte polling for command/frame transport
 *   - Software CS: PB4 controlled manually
 *   - D/C line: PB0 controlled per transaction
 */

#include "at32f403a_407.h"
#include "at32f403a_407_spi.h"
#include <stdbool.h>

/* Pin definitions (match Rust side) */
#define PIN_DC_PIN   0   /* PB0 */
#define PIN_RST_PIN  1   /* PB1 */
#define PIN_CS_PIN   4   /* PB4 */

#define GPIOB_BASE   0x40010C00
#define SPI1_BASE    0x40013000

/* GPIO register offsets */
#define GPIO_CRL     0x00   /* Pins 0-7 configuration */
#define GPIO_CRH     0x04   /* Pins 8-15 configuration */
#define GPIO_ODR     0x0C   /* Output data register */
#define GPIO_BSRR    0x10   /* Bit set/reset register */

/* GPIO pin configuration: 0b0010 = output push-pull, 10 MHz */
#define GPIO_MODE_OUTPUT_PP_10MHZ 0x00000001

static void gpio_set_mode(uint32_t base, uint8_t pin, uint32_t mode) {
    volatile uint32_t *reg = (volatile uint32_t *)(base + (pin < 8 ? 0x00 : 0x04));
    uint32_t shift = (pin % 8) * 4;
    uint32_t val = reg_read_volatile(reg);
    val &= ~(0xF << shift);
    val |= (mode << shift);
    reg_write_volatile(reg, val);
}

static void gpio_set_level(uint32_t base, uint8_t pin, bool high) {
    volatile uint32_t *bsrr = (volatile uint32_t *)(base + 0x10);
    if (high) {
        reg_write_volatile(bsrr, 1 << pin);           /* Bit set register */
    } else {
        reg_write_volatile(bsrr, 1 << (pin + 16));    /* Bit clear register */
    }
}

/* SPI register access helpers */
#define SPI_REG(spi, off) (*(volatile uint32_t *)((spi) + (off)))
#define SPI_DR  0x0C   /* Data register */
#define SPI_SR  0x08   /* Status register */
#define SPI_CR1 0x00   /* Control register 1 */

/* Status register bits */
#define SPI_SR_TXE  (1 << 1)   /* Transmit buffer empty */
#define SPI_SR_RXNE (1 << 0)   /* Receive buffer not empty */

/* Write one byte via SPI */
static void spi_write_byte(uint32_t spi_base, uint8_t data) {
    /* Wait for transmit buffer empty */
    while (!(SPI_REG(spi_base, SPI_SR) & SPI_SR_TXE)) {
        /* wait */
    }
    /* Write data */
    SPI_REG(spi_base, SPI_DR) = data;
}

/* Initialize SPI1 for panel transport (100 kHz, 8-bit, mode 0) */
void fnb58_spi_init(void) {
    spi_init_type spi_cfg;
    
    /* Enable SPI1 and GPIO clocks */
    crm_periph_clock_enable(CRM_SPI1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    
    /* Configure GPIO pins */
    gpio_init_type gpio_init_struct;
    gpio_default_para_init(&gpio_init_struct);
    
    /* PB0 = D/C, PB1 = RST, PB4 = CS - all push-pull outputs */
    gpio_init_struct.gpio_pins = GPIO_PINS_0 | GPIO_PINS_1 | GPIO_PINS_4;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(GPIOB, &gpio_init_struct);
    
    /* Set all outputs high initially (inactive) */
    gpio_write(GPIOB, GPIO_PINS_0 | GPIO_PINS_1 | GPIO_PINS_4, TRUE);
    
    /* Initialize SPI1 */
    spi_cfg.transmission_mode = SPI_TRANSMIT_FULL_DUPLEX;
    spi_cfg.mclk_freq_division = SPI_MCLK_DIV_4;   /* ~30 MHz */
    spi_cfg.clock_polarity = SPI_CLOCK_POLARITY_LOW;
    spi_cfg.clock_phase = SPI_CLOCK_PHASE_1EDGE;
    spi_cfg.first_bit_transmission = SPI_FIRST_BIT_MSB;
    spi_cfg.frame_bit_num = SPI_FRAME_8BIT;
    spi_cfg.cs_mode_selection = SPI_CS_SOFTWARE_MODE;
    
    spi_init(SPI1, &spi_cfg);
    spi_enable(SPI1, TRUE);
    
    /* CS high (inactive) */
    gpio_write(GPIOB, GPIO_PINS_4, TRUE);
}

/* Write one byte via SPI */
void fnb58_spi_write_cmd(uint8_t cmd) {
    /* D/C low */
    gpio_write(GPIOB, GPIO_PINS_0, FALSE);
    /* Wait for transmit buffer empty */
    while (!(SPI_REG(SPI1_BASE, SPI_SR) & SPI_SR_TXE)) {
        /* wait */
    }
    /* Write data */
    SPI_REG(SPI1_BASE, SPI_DR) = cmd;
}

/* Write data byte via SPI */
void fnb58_spi_write_data(uint8_t data) {
    /* D/C high */
    gpio_write(GPIOB, GPIO_PINS_0, TRUE);
    /* Write data */
    spi_write_byte(SPI1_BASE, data);
}

/* Chip select control */
void fnb58_spi_cs(bool enable) {
    gpio_write(GPIOB, GPIO_PINS_4, !enable);  /* CS active low */
}

/* Initialize panel transport (called once at startup) */
void fnb58_panel_spi_init(void) {
    fnb58_spi_init();
    /* Ensure CS is high (inactive) */
    fnb58_spi_cs(false);
}

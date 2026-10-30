/*!
 * @file fnb58_lcd.c
 * ST7789V 320x240 LCD driver for FNB58 - vendor SPI driver on SPI2.
 *
 * Verified pin map (user-traced 2026-10-30):
 *   PB13 = SPI2_SCK  (LCD clock, shared bus with W25Q128 flash)
 *   PB15 = SPI2_MOSI (LCD data, write-only - LCD has no MISO)
 *   PB12 = LCD CS    (active low; PB12 low pulls the CS wire low)
 *   PB11 = LCD D/C   (low=command, high=data)
 *   PB10 = LCD RST   (active low)
 *
 * SPI2 is on APB1 (48 MHz); SCK = APB1/4 = 12 MHz. Mode 0 (CPOL=0,
 * CPHA=0): the ST7789V latches SDA on the rising edge of SCK.
 * The flash shares SCK/MISO/MOSI but has its own CS, so as long as we
 * only pull PB12 low, flash stays off the bus.
 */

#include "at32f403a_407.h"
#include <stdbool.h>

#define LCD_CS   GPIO_PINS_12
#define LCD_DC   GPIO_PINS_11
#define LCD_RST  GPIO_PINS_10
#define LCD_CTRL (LCD_CS | LCD_DC | LCD_RST)

/* Backlight enable: PB3, drives a MOSFET gate (user-identified).
 * HIGH did not light the panel -> gate polarity may be inverted
 * (P-FET high-side): try LOW = backlight on. */
#define LCD_BLK  GPIO_PINS_3

/* Screen geometry: landscape 320x240. */
#define LCD_W 320
#define LCD_H 240

/* RGB565 colors for the test pattern. */
#define COL_RED     0xF800
#define COL_GREEN   0x07E0
#define COL_BLUE    0x001F
#define COL_CYAN    0x07FF
#define COL_MAGENTA 0xF81F
#define COL_YELLOW  0xFFE0
#define COL_WHITE   0xFFFF
#define COL_BLACK   0x0000

/* Busy-wait delay. Volatile loop, ~5 cycles per iteration at 192 MHz
 * HCLK; doubled for margin (overshoot is harmless for LCD init). */
static void lcd_delay_ms(uint32_t ms)
{
    volatile uint32_t n = ms * 2U * 19200U;
    while (n--) {
        __asm volatile ("nop");
    }
}

/* --- low-level transport -------------------------------------------- */

/* Stream one byte; TDBE polled so bytes run back-to-back. */
static void lcd_byte(uint8_t b)
{
    while (spi_i2s_flag_get(SPI2, SPI_I2S_TDBE_FLAG) == RESET) {
    }
    SPI2->dt = b;
}

/* Wait until the last bit has left the shifter, then drain RX. */
static void lcd_wait_idle(void)
{
    while (spi_i2s_flag_get(SPI2, SPI_I2S_BF_FLAG) == SET) {
    }
    if (spi_i2s_flag_get(SPI2, SPI_I2S_RDBF_FLAG) == SET) {
        (void)SPI2->dt;
    }
}

/* One command byte (D/C low). */
static void lcd_cmd(uint8_t cmd)
{
    lcd_wait_idle();
    gpio_bits_reset(GPIOB, LCD_DC);
    gpio_bits_reset(GPIOB, LCD_CS);
    lcd_byte(cmd);
    lcd_wait_idle();
    gpio_bits_set(GPIOB, LCD_CS);
    gpio_bits_set(GPIOB, LCD_DC);
}

/* Data bytes (D/C high). CS stays low across the whole buffer. */
static void lcd_data(const uint8_t *d, uint32_t n)
{
    lcd_wait_idle();
    gpio_bits_set(GPIOB, LCD_DC);
    gpio_bits_reset(GPIOB, LCD_CS);
    while (n--) {
        lcd_byte(*d++);
    }
    lcd_wait_idle();
    gpio_bits_set(GPIOB, LCD_CS);
}

/* --- init sequence --------------------------------------------------- */

/* ST7789V power/gamma init (Adafruit-style defaults). */
static void lcd_init_regs(void)
{
    static const uint8_t porctrl[5]  = {0x0C, 0x0C, 0x00, 0x33, 0x33};
    static const uint8_t pwctrl1[2]  = {0xA4, 0xA1};
    static const uint8_t pvgam[14]   = {0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B,
                                        0x3F, 0x54, 0x4C, 0x18, 0x0D, 0x0B,
                                        0x1F, 0x23};
    static const uint8_t nvgam[14]   = {0xD0, 0x04, 0x0E, 0x11, 0x12, 0x2A,
                                        0x3D, 0x51, 0x4F, 0x1B, 0x0E, 0x0B,
                                        0x1F, 0x23};
    uint8_t one;

    lcd_cmd(0xB2);  lcd_data(porctrl, 5);   /* porch control */
    one = 0x35; lcd_cmd(0xB7); lcd_data(&one, 1);  /* gate ctrl */
    one = 0x19; lcd_cmd(0xBB); lcd_data(&one, 1);  /* VCOMS */
    one = 0x2C; lcd_cmd(0xC0); lcd_data(&one, 1);  /* LCMCTRL */
    one = 0x01; lcd_cmd(0xC2); lcd_data(&one, 1);  /* VDV/VRH enable */
    one = 0x12; lcd_cmd(0xC3); lcd_data(&one, 1);  /* VRHS */
    one = 0x20; lcd_cmd(0xC4); lcd_data(&one, 1);  /* VDVS */
    one = 0x0F; lcd_cmd(0xC6); lcd_data(&one, 1);  /* FRCTRL2 60Hz */
    lcd_cmd(0xD0);  lcd_data(pwctrl1, 2);   /* power ctrl 1 */
    lcd_cmd(0xE0);  lcd_data(pvgam, 14);    /* positive gamma */
    lcd_cmd(0xE1);  lcd_data(nvgam, 14);    /* negative gamma */
}

void fnb58_lcd_init(void)
{
    gpio_init_type g;
    spi_init_type s;

    /* Clocks: SPI2 + GPIOB + IOMUX (remap register needs it). */
    crm_periph_clock_enable(CRM_SPI2_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);

    /* Release PB3/PB4/PA15 from JTAG: SWJTAG remap to SWD-only.
     * Without this, PB3 stays JTDO and gpio_init on it is a no-op. */
    gpio_pin_remap_config(SWJTAG_GMUX_010, TRUE);

    /* Control pins: CS/DC/RST push-pull outputs, idle high. */
    gpio_default_para_init(&g);
    g.gpio_pins = LCD_CTRL;
    g.gpio_mode = GPIO_MODE_OUTPUT;
    g.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    g.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
    g.gpio_pull = GPIO_PULL_NONE;
    gpio_init(GPIOB, &g);
    gpio_bits_set(GPIOB, LCD_CTRL);

    /* Backlight MOSFET gate on PB3: output, HIGH = on (r45, remap kept). */
    g.gpio_pins = LCD_BLK;
    gpio_init(GPIOB, &g);
    gpio_bits_set(GPIOB, LCD_BLK);

    /* Bus pins: PB13 = SPI2_SCK, PB15 = SPI2_MOSI (mux). */
    g.gpio_pins = GPIO_PINS_13 | GPIO_PINS_15;
    g.gpio_mode = GPIO_MODE_MUX;
    gpio_init(GPIOB, &g);

    /* SPI2 master, mode 0, 8-bit, MSB, software CS, APB1/4 = 12 MHz. */
    spi_default_para_init(&s);
    s.master_slave_mode = SPI_MODE_MASTER;
    s.transmission_mode = SPI_TRANSMIT_FULL_DUPLEX;
    s.mclk_freq_division = SPI_MCLK_DIV_4;
    s.clock_polarity = SPI_CLOCK_POLARITY_LOW;
    s.clock_phase = SPI_CLOCK_PHASE_1EDGE;
    s.first_bit_transmission = SPI_FIRST_BIT_MSB;
    s.frame_bit_num = SPI_FRAME_8BIT;
    s.cs_mode_selection = SPI_CS_SOFTWARE_MODE;
    spi_init(SPI2, &s);
    spi_enable(SPI2, TRUE);

    /* Hardware reset (generous margins). */
    gpio_bits_set(GPIOB, LCD_RST);
    lcd_delay_ms(50);
    gpio_bits_reset(GPIOB, LCD_RST);
    lcd_delay_ms(20);
    gpio_bits_set(GPIOB, LCD_RST);
    lcd_delay_ms(150);

    /* Wake-up and basic config. */
    lcd_cmd(0x01);            /* SWRESET */
    lcd_delay_ms(150);
    lcd_cmd(0x11);            /* SLPOUT */
    lcd_delay_ms(120);

    lcd_init_regs();

    uint8_t one;
    one = 0x55; lcd_cmd(0x3A); lcd_data(&one, 1);  /* RGB565 */
    one = 0x60; lcd_cmd(0x36); lcd_data(&one, 1);  /* MADCTL: landscape */
    lcd_cmd(0x21);            /* INVON: IPS panels need inversion */

    fnb58_lcd_fill(COL_BLACK);
    lcd_cmd(0x29);            /* DISPON */
    lcd_delay_ms(100);
}

/* --- drawing ---------------------------------------------------------- */

/* Address window (landscape: x 0-319, y 0-239). */
void fnb58_lcd_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t ca[4] = {(uint8_t)(x0 >> 8), (uint8_t)x0,
                     (uint8_t)(x1 >> 8), (uint8_t)x1};
    uint8_t ra[4] = {(uint8_t)(y0 >> 8), (uint8_t)y0,
                     (uint8_t)(y1 >> 8), (uint8_t)y1};
    lcd_cmd(0x2A); lcd_data(ca, 4);   /* CASET */
    lcd_cmd(0x2B); lcd_data(ra, 4);   /* RASET */
    lcd_cmd(0x2C);                    /* RAMWR (data follows) */
}

/* Fill the given window with one RGB565 color. */
void fnb58_lcd_fill_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
                           uint16_t color)
{
    uint32_t n = (uint32_t)(x1 - x0 + 1) * (uint32_t)(y1 - y0 + 1);
    uint8_t hi = (uint8_t)(color >> 8);
    uint8_t lo = (uint8_t)color;

    fnb58_lcd_set_window(x0, y0, x1, y1);
    lcd_wait_idle();
    gpio_bits_set(GPIOB, LCD_DC);
    gpio_bits_reset(GPIOB, LCD_CS);
    while (n--) {
        lcd_byte(hi);
        lcd_byte(lo);
    }
    lcd_wait_idle();
    gpio_bits_set(GPIOB, LCD_CS);
}

/* Full-screen fill. */
void fnb58_lcd_fill(uint16_t color)
{
    fnb58_lcd_fill_window(0, 0, LCD_W - 1, LCD_H - 1, color);
}

/* Eight 40px vertical color bars: W R G B C M Y K. */
void fnb58_lcd_test_pattern(void)
{
    static const uint16_t bars[8] = {COL_WHITE, COL_RED, COL_GREEN, COL_BLUE,
                                     COL_CYAN, COL_MAGENTA, COL_YELLOW,
                                     COL_BLACK};
    for (int i = 0; i < 8; i++) {
        uint16_t x0 = (uint16_t)(i * 40);
        fnb58_lcd_fill_window(x0, 0, x0 + 39, LCD_H - 1, bars[i]);
    }
}

/* --- 5x7 font --------------------------------------------------------- */

/* Glyphs are 5 columns x 7 rows; each byte is one column, bit r = row r
 * (bit 0 = top). Characters: 0-9 . space V A W m u - P S C D */
static const char FONT_CHARS[] = "0123456789. VAWmu-PSCD";
static const uint8_t FONT_5X7[][5] = {
    /* 0 */ {0x3E, 0x51, 0x49, 0x45, 0x3E},
    /* 1 */ {0x00, 0x42, 0x7F, 0x40, 0x00},
    /* 2 */ {0x62, 0x51, 0x49, 0x49, 0x46},
    /* 3 */ {0x41, 0x49, 0x49, 0x49, 0x36},
    /* 4 */ {0x18, 0x14, 0x12, 0x7F, 0x10},
    /* 5 */ {0x27, 0x45, 0x45, 0x45, 0x39},
    /* 6 */ {0x3C, 0x4A, 0x49, 0x49, 0x30},
    /* 7 */ {0x01, 0x71, 0x09, 0x05, 0x03},
    /* 8 */ {0x36, 0x49, 0x49, 0x49, 0x36},
    /* 9 */ {0x06, 0x49, 0x49, 0x29, 0x1E},
    /* . */ {0x00, 0x60, 0x60, 0x00, 0x00},
    /* ' '*/ {0x00, 0x00, 0x00, 0x00, 0x00},
    /* V */ {0x1F, 0x20, 0x40, 0x20, 0x1F},
    /* A */ {0x7E, 0x09, 0x09, 0x09, 0x7E},
    /* W */ {0x3F, 0x40, 0x38, 0x40, 0x3F},
    /* m */ {0x7C, 0x04, 0x04, 0x04, 0x78},
    /* u */ {0x3C, 0x40, 0x40, 0x20, 0x7C},
    /* - */ {0x08, 0x08, 0x08, 0x08, 0x08},
    /* P */ {0x7F, 0x09, 0x09, 0x09, 0x06},
    /* S */ {0x46, 0x49, 0x49, 0x49, 0x31},
    /* C */ {0x3E, 0x41, 0x41, 0x41, 0x22},
    /* D */ {0x7F, 0x41, 0x41, 0x41, 0x3E},
};

/* Draw one character at (x, y), `scale` pixels per font pixel, fg on bg.
 * Returns the character width (5*scale). */
uint16_t fnb58_lcd_draw_char(uint16_t x, uint16_t y, char ch, uint8_t scale,
                             uint16_t fg, uint16_t bg)
{
    /* Map char -> glyph index (no libc on target). */
    int idx = 11; /* default: blank */
    for (int i = 0; FONT_CHARS[i]; i++) {
        if (FONT_CHARS[i] == ch) {
            idx = i;
            break;
        }
    }
    const uint8_t *glyph = FONT_5X7[idx];

    fnb58_lcd_set_window(x, y, x + 5u * scale - 1u, y + 7u * scale - 1u);
    lcd_wait_idle();
    gpio_bits_set(GPIOB, LCD_DC);
    gpio_bits_reset(GPIOB, LCD_CS);
    for (uint8_t r = 0; r < 7u * scale; r++) {
        for (uint8_t c = 0; c < 5u * scale; c++) {
            uint8_t col = glyph[c / scale];
            uint16_t color = (col & (1u << (r / scale))) ? fg : bg;
            lcd_byte((uint8_t)(color >> 8));
            lcd_byte((uint8_t)color);
        }
    }
    lcd_wait_idle();
    gpio_bits_set(GPIOB, LCD_CS);
    return 5u * scale;
}

/* Draw a NUL-terminated string; 1px letter spacing. Unsupported
 * characters render as blanks. Returns final x. */
uint16_t fnb58_lcd_draw_text(uint16_t x, uint16_t y, const char *s,
                             uint8_t scale, uint16_t fg, uint16_t bg)
{
    while (*s) {
        x += fnb58_lcd_draw_char(x, y, *s++, scale, fg, bg) + scale;
    }
    return x;
}

/* --- bisect helpers (r52): add LCD ingredients one at a time ---------- */

/* Ingredient 1: IOMUX clock + SWJTAG remap only. */
void fnb58_bisect_iomux(void)
{
    crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);
    gpio_pin_remap_config(SWJTAG_GMUX_010, TRUE);
}

/* Ingredient 2: PB3 backlight high (requires remap first). */
void fnb58_bisect_blk(void)
{
    gpio_init_type g;
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    gpio_default_para_init(&g);
    g.gpio_pins = GPIO_PINS_3;
    g.gpio_mode = GPIO_MODE_OUTPUT;
    g.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    g.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
    g.gpio_pull = GPIO_PULL_NONE;
    gpio_init(GPIOB, &g);
    gpio_bits_set(GPIOB, GPIO_PINS_3);
}

/* Ingredient 3: SPI2 clock + PB13/PB15 mux + spi init/enable,
 * no control pins, no traffic. */
void fnb58_bisect_spi2(void)
{
    gpio_init_type g;
    spi_init_type s;

    crm_periph_clock_enable(CRM_SPI2_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);

    gpio_default_para_init(&g);
    g.gpio_pins = GPIO_PINS_13 | GPIO_PINS_15;
    g.gpio_mode = GPIO_MODE_MUX;
    g.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    g.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
    g.gpio_pull = GPIO_PULL_NONE;
    gpio_init(GPIOB, &g);

    spi_default_para_init(&s);
    s.master_slave_mode = SPI_MODE_MASTER;
    s.transmission_mode = SPI_TRANSMIT_FULL_DUPLEX;
    s.mclk_freq_division = SPI_MCLK_DIV_4;
    s.clock_polarity = SPI_CLOCK_POLARITY_LOW;
    s.clock_phase = SPI_CLOCK_PHASE_1EDGE;
    s.first_bit_transmission = SPI_FIRST_BIT_MSB;
    s.frame_bit_num = SPI_FRAME_8BIT;
    s.cs_mode_selection = SPI_CS_SOFTWARE_MODE;
    spi_init(SPI2, &s);
    spi_enable(SPI2, TRUE);
}

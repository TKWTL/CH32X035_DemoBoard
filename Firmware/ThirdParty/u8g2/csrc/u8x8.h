/*
 * Minimal u8x8 compatibility layer for the CH32X035 SSD1306 renderer port.
 *
 * This is intentionally not the complete upstream u8x8 API.  It defines the
 * ABI/types and helpers used by the selected u8g2 rendering core and by the
 * upstream u8g2.h/u8g2_font.c files which are supplied separately.
 */
#ifndef U8X8_H
#define U8X8_H

#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>
#include <limits.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __GNUC__
#define U8X8_NOINLINE __attribute__((noinline))
#define U8X8_UNUSED __attribute__((unused))
#define U8X8_SECTION(name) __attribute__((section(name)))
#else
#define U8X8_NOINLINE
#define U8X8_UNUSED
#define U8X8_SECTION(name)
#endif

#ifndef U8X8_FONT_SECTION
#define U8X8_FONT_SECTION(name)
#endif
#ifndef U8X8_PROGMEM
#define U8X8_PROGMEM
#endif
#ifndef u8x8_pgm_read
#define u8x8_pgm_read(addr) (*(const uint8_t *)(addr))
#endif

typedef struct u8x8_struct u8x8_t;
typedef struct u8x8_display_info_struct u8x8_display_info_t;
typedef struct u8x8_tile_struct u8x8_tile_t;
typedef struct u8log_struct u8log_t;
typedef uint8_t (*u8x8_msg_cb)(u8x8_t *, uint8_t, uint8_t, void *);
typedef uint16_t (*u8x8_char_cb)(u8x8_t *, uint8_t);

struct u8x8_tile_struct
{
    uint8_t *tile_ptr;
    uint8_t cnt;
    uint8_t x_pos;
    uint8_t y_pos;
};

struct u8x8_display_info_struct
{
    uint8_t chip_enable_level;
    uint8_t chip_disable_level;
    uint8_t post_chip_enable_wait_ns;
    uint8_t pre_chip_disable_wait_ns;
    uint8_t reset_pulse_width_ms;
    uint8_t post_reset_wait_ms;
    uint8_t sda_setup_time_ns;
    uint8_t sck_pulse_width_ns;
    uint32_t sck_clock_hz;
    uint8_t spi_mode;
    uint8_t i2c_bus_clock_100kHz;
    uint8_t data_setup_time_ns;
    uint8_t write_pulse_width_ns;
    uint8_t tile_width;
    uint8_t tile_height;
    uint8_t default_x_offset;
    uint8_t flipmode_x_offset;
    uint16_t pixel_width;
    uint16_t pixel_height;
};

#define U8X8_PIN_D0              0u
#define U8X8_PIN_SPI_CLOCK       0u
#define U8X8_PIN_D1              1u
#define U8X8_PIN_SPI_DATA        1u
#define U8X8_PIN_D2              2u
#define U8X8_PIN_D3              3u
#define U8X8_PIN_D4              4u
#define U8X8_PIN_D5              5u
#define U8X8_PIN_D6              6u
#define U8X8_PIN_D7              7u
#define U8X8_PIN_E               8u
#define U8X8_PIN_CS              9u
#define U8X8_PIN_DC              10u
#define U8X8_PIN_RESET           11u
#define U8X8_PIN_I2C_CLOCK       12u
#define U8X8_PIN_I2C_DATA        13u
#define U8X8_PIN_CS1             14u
#define U8X8_PIN_CS2             15u
#define U8X8_PIN_OUTPUT_CNT      16u
#define U8X8_PIN_MENU_SELECT     16u
#define U8X8_PIN_MENU_NEXT       17u
#define U8X8_PIN_MENU_PREV       18u
#define U8X8_PIN_MENU_HOME       19u
#define U8X8_PIN_MENU_UP         20u
#define U8X8_PIN_MENU_DOWN       21u
#define U8X8_PIN_INPUT_CNT       6u
#define U8X8_PIN_NONE            255u

struct u8x8_struct
{
    const u8x8_display_info_t *display_info;
    u8x8_char_cb next_cb;
    u8x8_msg_cb display_cb;
    u8x8_msg_cb cad_cb;
    u8x8_msg_cb byte_cb;
    u8x8_msg_cb gpio_and_delay_cb;
    uint32_t bus_clock;
    const uint8_t *font;
    uint16_t encoding;
    uint8_t x_offset;
    uint8_t is_font_inverse_mode;
    uint8_t i2c_address;
    uint8_t i2c_started;
    uint8_t utf8_state;
    uint8_t gpio_result;
    uint8_t debounce_default_pin_state;
    uint8_t debounce_last_pin_state;
    uint8_t debounce_state;
    uint8_t debounce_result_msg;
#ifdef U8X8_WITH_USER_PTR
    void *user_ptr;
#endif
#ifdef U8X8_USE_PINS
    uint8_t pins[U8X8_PIN_OUTPUT_CNT + U8X8_PIN_INPUT_CNT];
#endif
};

#define u8x8_GetCols(u8x8)              ((u8x8)->display_info->tile_width)
#define u8x8_GetRows(u8x8)              ((u8x8)->display_info->tile_height)
#define u8x8_GetI2CAddress(u8x8)        ((u8x8)->i2c_address)
#define u8x8_SetI2CAddress(u8x8, a)     ((u8x8)->i2c_address = (a))
#define u8x8_SetGPIOResult(u8x8, v)     ((u8x8)->gpio_result = (v))
#define u8x8_GetSPIClockPhase(u8x8)     ((u8x8)->display_info->spi_mode & 1u)
#define u8x8_GetSPIClockPolarity(u8x8)  (((u8x8)->display_info->spi_mode >> 1) & 1u)
#define u8x8_GetFontCharWidth(u8x8)     u8x8_pgm_read((u8x8)->font + 2)
#define u8x8_GetFontCharHeight(u8x8)    u8x8_pgm_read((u8x8)->font + 3)

/* Common message IDs retained for callback compatibility. */
#define U8X8_MSG_DISPLAY_SETUP_MEMORY      9u
#define U8X8_MSG_DISPLAY_INIT              10u
#define U8X8_MSG_DISPLAY_SET_POWER_SAVE    11u
#define U8X8_MSG_DISPLAY_SET_FLIP_MODE     13u
#define U8X8_MSG_DISPLAY_SET_CONTRAST      14u
#define U8X8_MSG_DISPLAY_DRAW_TILE         15u
#define U8X8_MSG_DISPLAY_REFRESH           16u
#define U8X8_MSG_DISPLAY_SET_PIXEL_OUTPUT  17u

#define U8X8_MSG_CAD_INIT                  20u
#define U8X8_MSG_CAD_SEND_CMD              21u
#define U8X8_MSG_CAD_SEND_ARG              22u
#define U8X8_MSG_CAD_SEND_DATA             23u
#define U8X8_MSG_CAD_START_TRANSFER        24u
#define U8X8_MSG_CAD_END_TRANSFER          25u
#define U8X8_MSG_BYTE_INIT                 U8X8_MSG_CAD_INIT
#define U8X8_MSG_BYTE_SEND                 U8X8_MSG_CAD_SEND_DATA
#define U8X8_MSG_BYTE_START_TRANSFER       U8X8_MSG_CAD_START_TRANSFER
#define U8X8_MSG_BYTE_END_TRANSFER         U8X8_MSG_CAD_END_TRANSFER
#define U8X8_MSG_BYTE_SET_DC               32u

#define U8X8_MSG_GPIO_AND_DELAY_INIT       40u
#define U8X8_MSG_DELAY_MILLI               41u
#define U8X8_MSG_DELAY_10MICRO             42u
#define U8X8_MSG_DELAY_100NANO             43u
#define U8X8_MSG_DELAY_NANO                44u
#define U8X8_MSG_DELAY_I2C                 45u
#define U8X8_MSG_GPIO(x)                   (64u + (x))
#define U8X8_MSG_GPIO_RESET                U8X8_MSG_GPIO(U8X8_PIN_RESET)
#define U8X8_MSG_GPIO_I2C_CLOCK            U8X8_MSG_GPIO(U8X8_PIN_I2C_CLOCK)
#define U8X8_MSG_GPIO_I2C_DATA             U8X8_MSG_GPIO(U8X8_PIN_I2C_DATA)

/* String decoders used by upstream u8g2_font.c. */
void u8x8_utf8_init(u8x8_t *u8x8);
uint16_t u8x8_ascii_next(u8x8_t *u8x8, uint8_t b);
uint16_t u8x8_utf8_next(u8x8_t *u8x8, uint8_t b);

/* No-panel renderer stubs. They keep generic u8g2 wrapper macros linkable if
 * accidentally referenced; SSD1306 transport itself lives in the BSP. */
uint8_t u8x8_dummy_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
void u8x8_SetupDefaults(u8x8_t *u8x8);
void u8x8_Setup(u8x8_t *u8x8, u8x8_msg_cb display_cb, u8x8_msg_cb cad_cb,
                u8x8_msg_cb byte_cb, u8x8_msg_cb gpio_and_delay_cb);
void u8x8_SetupMemory(u8x8_t *u8x8);
void u8x8_InitInterface(u8x8_t *u8x8);
void u8x8_InitDisplay(u8x8_t *u8x8);
void u8x8_SetPowerSave(u8x8_t *u8x8, uint8_t is_enable);
void u8x8_SetContrast(u8x8_t *u8x8, uint8_t value);
void u8x8_SetFlipMode(u8x8_t *u8x8, uint8_t mode);
void u8x8_RefreshDisplay(u8x8_t *u8x8);
void u8x8_ClearDisplay(u8x8_t *u8x8);
void u8x8_DrawTile(u8x8_t *u8x8, uint8_t x, uint8_t y, uint8_t cnt, uint8_t *tile_ptr);
void u8x8_SetFont(u8x8_t *u8x8, const uint8_t *font_8x8);

#ifdef __cplusplus
}
#endif
#endif /* U8X8_H */

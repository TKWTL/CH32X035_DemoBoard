#include "ssd1306_u8g2.h"
#include "time_api.h"
#include "ch32x035.h"
#include <string.h>

/* The display transport is intentionally not implemented through u8g2's
 * synchronous byte callback. u8g2 owns rendering/framebuffer only; I2C1 DMA
 * is driven by Peripheral/I2C and coroOS. Calling u8g2_SendBuffer() would
 * re-introduce a blocking transport path, so use SSD1306_U8G2_BeginFlush(). */

static u8g2_t s_u8g2;
static uint8_t s_framebuffer[SSD1306_WIDTH * SSD1306_PAGE_COUNT];

/* Renderer-only display geometry. Panel initialization and pixel transport are
 * intentionally handled by this BSP rather than by u8x8 display callbacks. */
static const u8x8_display_info_t s_display_info =
{
    0u, 1u,
    20u, 10u,
    100u, 100u,
    50u, 50u,
    8000000UL,
    0u,
    4u,
    40u, 150u,
    16u, 8u,
    0u, 0u,
    SSD1306_WIDTH, SSD1306_HEIGHT
};
static uint8_t s_address = SSD1306_I2C_ADDR_PRIMARY;
static uint8_t s_initialized;

static const uint8_t s_init_packet[] =
{
    0x00u,       /* Co=0, D/C#=0: following bytes are commands */
    0xAEu,       /* display off */
    0xD5u, 0x80u,
    0xA8u, 0x3Fu,
    0xD3u, 0x00u,
    0x40u,
    0x8Du, 0x14u,
    0x20u, 0x00u, /* horizontal addressing */
    0xA1u,
    0xC8u,
    0xDAu, 0x12u,
    0x81u, 0xCFu,
    0xD9u, 0xF1u,
    0xDBu, 0x40u,
    0xA4u,
    0xA6u,
    0x2Eu,
    0xAFu
};

static uint8_t s_page_cmd[7];
static uint8_t s_page_data[1u + SSD1306_WIDTH];
static uint8_t s_flush_page;
static uint8_t s_flush_phase;
static uint8_t s_flush_busy;
static I2C_API_Result s_flush_result = I2C_API_RESULT_IDLE;

static uint8_t u8x8_byte_async_placeholder(u8x8_t *u8x8,
                                            uint8_t msg,
                                            uint8_t arg_int,
                                            void *arg_ptr)
{
    (void)u8x8;
    (void)msg;
    (void)arg_int;
    (void)arg_ptr;
    /* Deliberately no synchronous I2C access here. */
    return 1u;
}

static uint8_t u8x8_gpio_and_delay_ch32x035(u8x8_t *u8x8,
                                             uint8_t msg,
                                             uint8_t arg_int,
                                             void *arg_ptr)
{
    (void)u8x8;
    (void)arg_ptr;

    switch(msg)
    {
        case U8X8_MSG_DELAY_MILLI:
            /* Setup does not use this callback for panel init in this port.
             * Keep it for generic u8g2 code that asks for a short delay. */
            while(arg_int-- != 0u)
                TIME_DelayUs(1000u);
            break;
#ifdef U8X8_MSG_DELAY_10MICRO
        case U8X8_MSG_DELAY_10MICRO:
            TIME_DelayUs((uint32_t)arg_int * 10u);
            break;
#endif
#ifdef U8X8_MSG_DELAY_100NANO
        case U8X8_MSG_DELAY_100NANO:
            if(arg_int != 0u)
                __NOP();
            break;
#endif
        default:
            break;
    }
    return 1u;
}

void SSD1306_U8G2_Init(void)
{
    u8x8_t *u8x8;

    /* Local full-buffer setup: no generated u8g2 display setup and no
     * synchronous u8x8 transport is linked into this firmware. */
    memset(&s_u8g2, 0, sizeof(s_u8g2));
    u8x8 = u8g2_GetU8x8(&s_u8g2);
    u8x8->display_info = &s_display_info;
    u8x8->display_cb = u8x8_dummy_cb;
    u8x8->cad_cb = u8x8_dummy_cb;
    u8x8->byte_cb = u8x8_byte_async_placeholder;
    u8x8->gpio_and_delay_cb = u8x8_gpio_and_delay_ch32x035;
    u8x8->i2c_address = (uint8_t)(s_address << 1);
    u8x8->debounce_default_pin_state = 0xffu;

    u8g2_SetupBuffer(&s_u8g2,
                     s_framebuffer,
                     SSD1306_PAGE_COUNT,
                     u8g2_ll_hvline_vertical_top_lsb,
                     U8G2_R0);
    u8g2_ClearBuffer(&s_u8g2);
    u8g2_SetFont(&s_u8g2, SSD1306_U8G2_DEFAULT_FONT);
    s_initialized = 1u;
    s_flush_busy = 0u;
    s_flush_result = I2C_API_RESULT_IDLE;
}

u8g2_t *SSD1306_U8G2_Get(void)
{
    return &s_u8g2;
}

void SSD1306_U8G2_SetAddress(uint8_t address_7bit)
{
    if(address_7bit != SSD1306_I2C_ADDR_PRIMARY &&
       address_7bit != SSD1306_I2C_ADDR_ALTERNATE)
        return;

    s_address = address_7bit;
    if(s_initialized)
        u8x8_SetI2CAddress(u8g2_GetU8x8(&s_u8g2), (uint8_t)(s_address << 1));
}

uint8_t SSD1306_U8G2_GetAddress(void)
{
    return s_address;
}

uint8_t SSD1306_U8G2_BeginProbe(uint8_t address_7bit)
{
    if(address_7bit != SSD1306_I2C_ADDR_PRIMARY &&
       address_7bit != SSD1306_I2C_ADDR_ALTERNATE)
        return 0u;
    return I2C_API_TryPing(I2C_API_OWNER_SSD1306_PROBE, address_7bit);
}

uint8_t SSD1306_U8G2_ProbeComplete(void)
{
    I2C_API_Result r = I2C_API_GetResult(I2C_API_OWNER_SSD1306_PROBE);
    return (r != I2C_API_RESULT_IDLE && r != I2C_API_RESULT_ACTIVE) ? 1u : 0u;
}

I2C_API_Result SSD1306_U8G2_TakeProbeResult(void)
{
    return I2C_API_TakeResult(I2C_API_OWNER_SSD1306_PROBE);
}

uint8_t SSD1306_U8G2_BeginPanelInit(void)
{
    return I2C_API_TryWrite(I2C_API_OWNER_SSD1306,
                            s_address,
                            s_init_packet,
                            (uint16_t)sizeof(s_init_packet));
}

uint8_t SSD1306_U8G2_PanelInitComplete(void)
{
    I2C_API_Result r = I2C_API_GetResult(I2C_API_OWNER_SSD1306);
    return (r != I2C_API_RESULT_IDLE && r != I2C_API_RESULT_ACTIVE) ? 1u : 0u;
}

I2C_API_Result SSD1306_U8G2_TakePanelInitResult(void)
{
    return I2C_API_TakeResult(I2C_API_OWNER_SSD1306);
}

static void prepare_page(uint8_t page)
{
    uint8_t *fb = u8g2_GetBufferPtr(&s_u8g2);
    uint16_t offset = (uint16_t)page * SSD1306_WIDTH;
    uint16_t i;

    s_page_cmd[0] = 0x00u;
    s_page_cmd[1] = 0x21u; /* column address */
    s_page_cmd[2] = 0x00u;
    s_page_cmd[3] = 0x7Fu;
    s_page_cmd[4] = 0x22u; /* page address */
    s_page_cmd[5] = page;
    s_page_cmd[6] = page;

    s_page_data[0] = 0x40u; /* data stream */
    for(i = 0u; i < SSD1306_WIDTH; ++i)
        s_page_data[1u + i] = fb[offset + i];
}

uint8_t SSD1306_U8G2_BeginFlush(void)
{
    if(!s_initialized || s_flush_busy)
        return 0u;

    s_flush_page = 0u;
    s_flush_phase = 0u;
    s_flush_busy = 1u;
    s_flush_result = I2C_API_RESULT_ACTIVE;
    prepare_page(0u);
    return 1u;
}

I2C_API_Result SSD1306_U8G2_PollFlush(void)
{
    I2C_API_Result r;

    if(!s_flush_busy)
        return s_flush_result;

    if(s_flush_phase == 0u)
    {
        r = I2C_API_GetResult(I2C_API_OWNER_SSD1306);
        if(r == I2C_API_RESULT_IDLE)
        {
            if(I2C_API_TryWrite(I2C_API_OWNER_SSD1306,
                                s_address,
                                s_page_cmd,
                                (uint16_t)sizeof(s_page_cmd)))
                s_flush_phase = 1u;
            return I2C_API_RESULT_ACTIVE;
        }
        return I2C_API_RESULT_ACTIVE;
    }

    if(s_flush_phase == 1u)
    {
        r = I2C_API_GetResult(I2C_API_OWNER_SSD1306);
        if(r == I2C_API_RESULT_ACTIVE)
            return I2C_API_RESULT_ACTIVE;
        if(r == I2C_API_RESULT_IDLE)
            return I2C_API_RESULT_ACTIVE;

        r = I2C_API_TakeResult(I2C_API_OWNER_SSD1306);
        if(r != I2C_API_RESULT_OK)
        {
            s_flush_busy = 0u;
            s_flush_result = r;
            return r;
        }
        s_flush_phase = 2u;
    }

    if(s_flush_phase == 2u)
    {
        if(I2C_API_TryWrite(I2C_API_OWNER_SSD1306,
                            s_address,
                            s_page_data,
                            (uint16_t)sizeof(s_page_data)))
            s_flush_phase = 3u;
        return I2C_API_RESULT_ACTIVE;
    }

    r = I2C_API_GetResult(I2C_API_OWNER_SSD1306);
    if(r == I2C_API_RESULT_ACTIVE || r == I2C_API_RESULT_IDLE)
        return I2C_API_RESULT_ACTIVE;

    r = I2C_API_TakeResult(I2C_API_OWNER_SSD1306);
    if(r != I2C_API_RESULT_OK)
    {
        s_flush_busy = 0u;
        s_flush_result = r;
        return r;
    }

    s_flush_page++;
    if(s_flush_page >= SSD1306_PAGE_COUNT)
    {
        s_flush_busy = 0u;
        s_flush_result = I2C_API_RESULT_OK;
        return I2C_API_RESULT_OK;
    }

    prepare_page(s_flush_page);
    s_flush_phase = 0u;
    return I2C_API_RESULT_ACTIVE;
}

uint8_t SSD1306_U8G2_IsFlushBusy(void)
{
    return s_flush_busy;
}

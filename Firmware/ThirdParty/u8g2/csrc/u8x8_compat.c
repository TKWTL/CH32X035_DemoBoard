/* Minimal u8x8 compatibility implementation for renderer-only use. */
#include "u8x8.h"
#include <string.h>

uint8_t u8x8_dummy_cb(U8X8_UNUSED u8x8_t *u8x8,
                      U8X8_UNUSED uint8_t msg,
                      U8X8_UNUSED uint8_t arg_int,
                      U8X8_UNUSED void *arg_ptr)
{
    return 1u;
}

void u8x8_SetupDefaults(u8x8_t *u8x8)
{
    memset(u8x8, 0, sizeof(*u8x8));
    u8x8->display_cb = u8x8_dummy_cb;
    u8x8->cad_cb = u8x8_dummy_cb;
    u8x8->byte_cb = u8x8_dummy_cb;
    u8x8->gpio_and_delay_cb = u8x8_dummy_cb;
    u8x8->i2c_address = 0xffu;
    u8x8->debounce_default_pin_state = 0xffu;
}

void u8x8_Setup(u8x8_t *u8x8, u8x8_msg_cb display_cb, u8x8_msg_cb cad_cb,
                u8x8_msg_cb byte_cb, u8x8_msg_cb gpio_and_delay_cb)
{
    u8x8_SetupDefaults(u8x8);
    u8x8->display_cb = display_cb != NULL ? display_cb : u8x8_dummy_cb;
    u8x8->cad_cb = cad_cb != NULL ? cad_cb : u8x8_dummy_cb;
    u8x8->byte_cb = byte_cb != NULL ? byte_cb : u8x8_dummy_cb;
    u8x8->gpio_and_delay_cb = gpio_and_delay_cb != NULL ? gpio_and_delay_cb : u8x8_dummy_cb;
    if(u8x8->display_cb != NULL)
        (void)u8x8->display_cb(u8x8, U8X8_MSG_DISPLAY_SETUP_MEMORY, 0u, NULL);
}

void u8x8_SetupMemory(u8x8_t *u8x8)
{
    if(u8x8->display_cb != NULL)
        (void)u8x8->display_cb(u8x8, U8X8_MSG_DISPLAY_SETUP_MEMORY, 0u, NULL);
}

void u8x8_InitInterface(u8x8_t *u8x8)
{
    if(u8x8->byte_cb != NULL)
        (void)u8x8->byte_cb(u8x8, U8X8_MSG_BYTE_INIT, 0u, NULL);
}

void u8x8_InitDisplay(u8x8_t *u8x8)
{
    if(u8x8->display_cb != NULL)
        (void)u8x8->display_cb(u8x8, U8X8_MSG_DISPLAY_INIT, 0u, NULL);
}

void u8x8_SetPowerSave(u8x8_t *u8x8, uint8_t is_enable)
{
    if(u8x8->display_cb != NULL)
        (void)u8x8->display_cb(u8x8, U8X8_MSG_DISPLAY_SET_POWER_SAVE, is_enable, NULL);
}

void u8x8_SetContrast(u8x8_t *u8x8, uint8_t value)
{
    if(u8x8->display_cb != NULL)
        (void)u8x8->display_cb(u8x8, U8X8_MSG_DISPLAY_SET_CONTRAST, value, NULL);
}

void u8x8_SetFlipMode(u8x8_t *u8x8, uint8_t mode)
{
    if(u8x8->display_cb != NULL)
        (void)u8x8->display_cb(u8x8, U8X8_MSG_DISPLAY_SET_FLIP_MODE, mode, NULL);
}

void u8x8_RefreshDisplay(u8x8_t *u8x8)
{
    if(u8x8->display_cb != NULL)
        (void)u8x8->display_cb(u8x8, U8X8_MSG_DISPLAY_REFRESH, 0u, NULL);
}

void u8x8_ClearDisplay(U8X8_UNUSED u8x8_t *u8x8) {}

void u8x8_DrawTile(u8x8_t *u8x8, uint8_t x, uint8_t y, uint8_t cnt, uint8_t *tile_ptr)
{
    u8x8_tile_t tile;
    if(u8x8->display_cb == NULL)
        return;
    tile.tile_ptr = tile_ptr;
    tile.cnt = cnt;
    tile.x_pos = x;
    tile.y_pos = y;
    (void)u8x8->display_cb(u8x8, U8X8_MSG_DISPLAY_DRAW_TILE, 1u, &tile);
}

void u8x8_SetFont(u8x8_t *u8x8, const uint8_t *font_8x8)
{
    u8x8->font = font_8x8;
}

void u8x8_utf8_init(u8x8_t *u8x8)
{
    u8x8->utf8_state = 0u;
    u8x8->encoding = 0u;
}

uint16_t u8x8_ascii_next(U8X8_UNUSED u8x8_t *u8x8, uint8_t b)
{
    if(b == 0u || b == (uint8_t)'\n')
        return 0xffffu;
    return (uint16_t)b;
}

uint16_t u8x8_utf8_next(u8x8_t *u8x8, uint8_t b)
{
    uint16_t code;

    if(b == 0u || b == (uint8_t)'\n')
    {
        u8x8_utf8_init(u8x8);
        return 0xffffu;
    }

    if(u8x8->utf8_state == 0u)
    {
        if((b & 0x80u) == 0u)
            return (uint16_t)b;
        if((b & 0xe0u) == 0xc0u)
        {
            u8x8->encoding = (uint16_t)(b & 0x1fu);
            u8x8->utf8_state = 1u;
            return 0xfffeu;
        }
        if((b & 0xf0u) == 0xe0u)
        {
            u8x8->encoding = (uint16_t)(b & 0x0fu);
            u8x8->utf8_state = 2u;
            return 0xfffeu;
        }
        /* u8g2's BMP renderer cannot represent four-byte code points. */
        u8x8_utf8_init(u8x8);
        return (uint16_t)'?';
    }

    if((b & 0xc0u) != 0x80u)
    {
        u8x8_utf8_init(u8x8);
        return (uint16_t)'?';
    }

    code = (uint16_t)((u8x8->encoding << 6) | (uint16_t)(b & 0x3fu));
    u8x8->encoding = code;
    u8x8->utf8_state--;
    if(u8x8->utf8_state != 0u)
        return 0xfffeu;
    return code;
}

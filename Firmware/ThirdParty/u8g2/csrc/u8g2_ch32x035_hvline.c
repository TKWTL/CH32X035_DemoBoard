/* Full-buffer pixel/HV-line backend for SSD1306 vertical-top/LSB layout. */
#include "u8g2.h"

static void set_pixel(u8g2_t *u8g2, u8g2_uint_t x, u8g2_uint_t y)
{
    uint32_t idx;
    uint8_t mask;
    uint8_t *p;

    if(x >= u8g2->width || y >= u8g2->height)
        return;
    if(x < u8g2->user_x0 || x >= u8g2->user_x1 ||
       y < u8g2->user_y0 || y >= u8g2->user_y1)
        return;

    idx = ((uint32_t)(y >> 3) * (uint32_t)u8g2->pixel_buf_width) + (uint32_t)x;
    p = &u8g2->tile_buf_ptr[idx];
    mask = (uint8_t)(1u << (y & 7u));

    if(u8g2->draw_color == 0u)
        *p = (uint8_t)(*p & (uint8_t)~mask);
    else if(u8g2->draw_color == 2u)
        *p ^= mask;
    else
        *p |= mask;
}

void u8g2_ll_hvline_vertical_top_lsb(u8g2_t *u8g2,
                                     u8g2_uint_t x,
                                     u8g2_uint_t y,
                                     u8g2_uint_t len,
                                     uint8_t dir)
{
    while(len-- != 0u)
    {
        set_pixel(u8g2, x, y);
        switch(dir & 3u)
        {
            case 0u: x++; break;
            case 1u: y++; break;
            case 2u: x--; break;
            default: y--; break;
        }
    }
}

void u8g2_DrawHVLine(u8g2_t *u8g2,
                     u8g2_uint_t x,
                     u8g2_uint_t y,
                     u8g2_uint_t len,
                     uint8_t dir)
{
    if(len == 0u)
        return;
    u8g2->ll_hvline(u8g2, x, y, len, dir);
}

void u8g2_DrawHLine(u8g2_t *u8g2, u8g2_uint_t x, u8g2_uint_t y, u8g2_uint_t len)
{
    u8g2_DrawHVLine(u8g2, x, y, len, 0u);
}

void u8g2_DrawVLine(u8g2_t *u8g2, u8g2_uint_t x, u8g2_uint_t y, u8g2_uint_t len)
{
    u8g2_DrawHVLine(u8g2, x, y, len, 1u);
}

void u8g2_DrawPixel(u8g2_t *u8g2, u8g2_uint_t x, u8g2_uint_t y)
{
    set_pixel(u8g2, x, y);
}

void u8g2_SetDrawColor(u8g2_t *u8g2, uint8_t color)
{
    u8g2->draw_color = (color < 3u) ? color : 1u;
}

void u8g2_DrawBox(u8g2_t *u8g2,
                  u8g2_uint_t x,
                  u8g2_uint_t y,
                  u8g2_uint_t w,
                  u8g2_uint_t h)
{
    while(h-- != 0u)
        u8g2_DrawHLine(u8g2, x, y++, w);
}

void u8g2_DrawFrame(u8g2_t *u8g2,
                    u8g2_uint_t x,
                    u8g2_uint_t y,
                    u8g2_uint_t w,
                    u8g2_uint_t h)
{
    if(w == 0u || h == 0u)
        return;
    u8g2_DrawHLine(u8g2, x, y, w);
    if(h > 1u)
        u8g2_DrawHLine(u8g2, x, (u8g2_uint_t)(y + h - 1u), w);
    if(h > 2u)
    {
        u8g2_DrawVLine(u8g2, x, (u8g2_uint_t)(y + 1u), (u8g2_uint_t)(h - 2u));
        if(w > 1u)
            u8g2_DrawVLine(u8g2, (u8g2_uint_t)(x + w - 1u),
                           (u8g2_uint_t)(y + 1u), (u8g2_uint_t)(h - 2u));
    }
}

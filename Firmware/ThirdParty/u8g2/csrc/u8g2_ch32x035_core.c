/*
 * Small u8g2 rendering core used by the CH32X035 SSD1306 port.
 * Only R0/full-buffer operation is required by this project.
 */
#include "u8g2.h"
#include <string.h>

static void update_dimension_r0(u8g2_t *u8g2)
{
    const u8x8_display_info_t *di = u8g2_GetU8x8(u8g2)->display_info;
    uint16_t rows = u8g2->tile_buf_height;
    uint16_t remain;

    u8g2->pixel_buf_height = (u8g2_uint_t)(rows * 8u);
    u8g2->pixel_buf_width = (u8g2_uint_t)((uint16_t)di->tile_width * 8u);
    u8g2->pixel_curr_row = (u8g2_uint_t)((uint16_t)u8g2->tile_curr_row * 8u);

    remain = di->tile_height;
    if(u8g2->tile_curr_row < remain)
        remain = (uint16_t)(remain - u8g2->tile_curr_row);
    else
        remain = 0u;
    if(rows > remain)
        rows = remain;

    u8g2->buf_y0 = u8g2->pixel_curr_row;
    u8g2->buf_y1 = (u8g2_uint_t)(u8g2->buf_y0 + rows * 8u);
    u8g2->width = (u8g2_uint_t)di->pixel_width;
    u8g2->height = (u8g2_uint_t)di->pixel_height;
}

static void update_page_r0(u8g2_t *u8g2)
{
    u8g2->user_x0 = 0u;
    u8g2->user_x1 = u8g2->width;
    u8g2->user_y0 = u8g2->buf_y0;
    u8g2->user_y1 = u8g2->buf_y1;
#ifdef U8G2_WITH_CLIP_WINDOW_SUPPORT
    if(u8g2->user_x0 < u8g2->clip_x0) u8g2->user_x0 = u8g2->clip_x0;
    if(u8g2->user_y0 < u8g2->clip_y0) u8g2->user_y0 = u8g2->clip_y0;
    if(u8g2->user_x1 > u8g2->clip_x1) u8g2->user_x1 = u8g2->clip_x1;
    if(u8g2->user_y1 > u8g2->clip_y1) u8g2->user_y1 = u8g2->clip_y1;
#endif
}

static void draw_l90_r0(u8g2_t *u8g2,
                         u8g2_uint_t x,
                         u8g2_uint_t y,
                         u8g2_uint_t len,
                         uint8_t dir)
{
    u8g2_DrawHVLine(u8g2, x, y, len, dir);
}

const u8g2_cb_t u8g2_cb_r0 =
{
    update_dimension_r0,
    update_page_r0,
    draw_l90_r0
};

#ifdef U8G2_WITH_CLIP_WINDOW_SUPPORT
void u8g2_SetMaxClipWindow(u8g2_t *u8g2)
{
    u8g2->clip_x0 = 0u;
    u8g2->clip_y0 = 0u;
    u8g2->clip_x1 = (u8g2_uint_t)~(u8g2_uint_t)0u;
    u8g2->clip_y1 = (u8g2_uint_t)~(u8g2_uint_t)0u;
    u8g2->cb->update_page_win(u8g2);
}

void u8g2_SetClipWindow(u8g2_t *u8g2,
                        u8g2_uint_t x0, u8g2_uint_t y0,
                        u8g2_uint_t x1, u8g2_uint_t y1)
{
    u8g2->clip_x0 = x0;
    u8g2->clip_y0 = y0;
    u8g2->clip_x1 = x1;
    u8g2->clip_y1 = y1;
    u8g2->cb->update_page_win(u8g2);
}
#endif

void u8g2_SetupBuffer(u8g2_t *u8g2,
                      uint8_t *buf,
                      uint8_t tile_buf_height,
                      u8g2_draw_ll_hvline_cb ll_hvline_cb,
                      const u8g2_cb_t *u8g2_cb)
{
    u8g2->font = NULL;
    u8g2->ll_hvline = ll_hvline_cb;
    u8g2->tile_buf_ptr = buf;
    u8g2->tile_buf_height = tile_buf_height;
    u8g2->tile_curr_row = 0u;
    u8g2->font_decode.is_transparent = 0u;
    u8g2->bitmap_transparency = 0u;
    u8g2->font_height_mode = 0u;
    u8g2->draw_color = 1u;
    u8g2->is_auto_page_clear = 1u;
    u8g2->cb = u8g2_cb;
    u8g2->cb->update_dimension(u8g2);
#ifdef U8G2_WITH_CLIP_WINDOW_SUPPORT
    u8g2_SetMaxClipWindow(u8g2);
#else
    u8g2->cb->update_page_win(u8g2);
#endif
    u8g2_SetFontPosBaseline(u8g2);
#ifdef U8G2_WITH_FONT_ROTATION
    u8g2->font_decode.dir = 0u;
#endif
}

void u8g2_SetDisplayRotation(u8g2_t *u8g2, const u8g2_cb_t *cb)
{
    u8g2->cb = cb;
    cb->update_dimension(u8g2);
    cb->update_page_win(u8g2);
}

void u8g2_ClearBuffer(u8g2_t *u8g2)
{
    size_t bytes = (size_t)u8g2_GetU8x8(u8g2)->display_info->tile_width *
                   (size_t)u8g2->tile_buf_height * 8u;
    memset(u8g2->tile_buf_ptr, 0, bytes);
}

uint8_t u8g2_IsIntersection(u8g2_t *u8g2,
                            u8g2_uint_t x0, u8g2_uint_t y0,
                            u8g2_uint_t x1, u8g2_uint_t y1)
{
    if(x1 <= u8g2->user_x0 || x0 >= u8g2->user_x1)
        return 0u;
    if(y1 <= u8g2->user_y0 || y0 >= u8g2->user_y1)
        return 0u;
    return 1u;
}

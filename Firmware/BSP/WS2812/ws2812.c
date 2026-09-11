#include "ws2812.h"
#include "pioc_api.h"

static uint8_t s_last_status = PIOC_API_OK;

uint8_t WS2812_SendGRB(const uint8_t *grb, uint16_t length)
{
    s_last_status = PIOC_API_Send(grb, length);
    return s_last_status;
}

uint8_t WS2812_SendRGB(const WS2812_RGB *pixels, uint8_t count)
{
    uint8_t grb[WS2812_LED_COUNT * 3u];
    uint8_t i;

    if((pixels == 0) || (count == 0u) || (count > WS2812_LED_COUNT))
        return PIOC_API_ERR_PARAM;

    for(i = 0u; i < count; i++)
    {
        uint16_t p = (uint16_t)i * 3u;
        grb[p + 0u] = pixels[i].g;
        grb[p + 1u] = pixels[i].r;
        grb[p + 2u] = pixels[i].b;
    }

    return WS2812_SendGRB(grb, (uint16_t)count * 3u);
}

uint8_t WS2812_Fill(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t grb[WS2812_LED_COUNT * 3u];
    uint8_t i;

    for(i = 0u; i < WS2812_LED_COUNT; i++)
    {
        uint16_t p = (uint16_t)i * 3u;
        grb[p + 0u] = g;
        grb[p + 1u] = r;
        grb[p + 2u] = b;
    }

    return WS2812_SendGRB(grb, (uint16_t)sizeof(grb));
}

uint8_t WS2812_Init(void)
{
    PIOC_API_Init();
    return WS2812_Fill(0u, 0u, 0u);
}

uint8_t WS2812_GetLastStatus(void)
{
    return s_last_status;
}

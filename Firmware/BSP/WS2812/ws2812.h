#ifndef __WS2812_H
#define __WS2812_H

#include <stdint.h>

#define WS2812_LED_COUNT  4u

typedef struct
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
} WS2812_RGB;

/* Stateless RGB-stream interface: no persistent framebuffer is kept in CPU RAM. */
uint8_t WS2812_Init(void);
uint8_t WS2812_SendGRB(const uint8_t *grb, uint16_t length);
uint8_t WS2812_SendRGB(const WS2812_RGB *pixels, uint8_t count);
uint8_t WS2812_Fill(uint8_t r, uint8_t g, uint8_t b);
uint8_t WS2812_GetLastStatus(void);

#endif /* __WS2812_H */

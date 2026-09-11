#include "ws2812_effect.h"
#include "ws2812.h"

/* Smooth RGB chase with sub-LSB temporal dithering.
 *
 * WS2812 stores only 8 bits per colour channel, so one transmitted frame can
 * never represent a step smaller than 1 LSB in one latched frame. Keep the
 * 4.000 s red->green / green->blue / blue->red segment time, but refresh at
 * 4 ms (250 Hz) so temporal dithering sits well above the visible step rate.
 * Retain Q8.8 colour/trail values and use first-order error diffusion at the
 * final 8-bit quantizer. Adjacent frames therefore share the fractional part
 * between neighbouring WS2812 codes instead of rounding it away.
 *
 * The chase head still advances every 48 ms; only physical refresh and colour
 * quantization are refined. */
#define WS2812_EFFECT_FADE_SCALE_Q16   64820u
#define WS2812_EFFECT_MOVE_MS             48u
#define WS2812_EFFECT_MOVE_FRAMES \
    (WS2812_EFFECT_MOVE_MS / WS2812_EFFECT_FRAME_MS)
#define WS2812_EFFECT_SEGMENT_MS       4000u
#define WS2812_EFFECT_SEGMENT_FRAMES \
    (WS2812_EFFECT_SEGMENT_MS / WS2812_EFFECT_FRAME_MS)
#define WS2812_EFFECT_SEGMENT_STEPS \
    (WS2812_EFFECT_SEGMENT_FRAMES - 1u)
#define WS2812_EFFECT_CYCLE_FRAMES \
    (WS2812_EFFECT_SEGMENT_FRAMES * 3u)
#define WS2812_EFFECT_FULL_Q8          (255u << 8)

static WS2812_RGB s_pixels[WS2812_LED_COUNT];
static uint16_t s_r_q8[WS2812_LED_COUNT];
static uint16_t s_g_q8[WS2812_LED_COUNT];
static uint16_t s_b_q8[WS2812_LED_COUNT];
static uint8_t s_r_quant_error[WS2812_LED_COUNT];
static uint8_t s_g_quant_error[WS2812_LED_COUNT];
static uint8_t s_b_quant_error[WS2812_LED_COUNT];
static uint16_t s_color_frame;
static uint8_t s_head;
static uint8_t s_move_phase;

static uint16_t effect_decay_q8(uint16_t value)
{
    return (uint16_t)((((uint32_t)value * WS2812_EFFECT_FADE_SCALE_Q16) + 32768u) >> 16);
}

/* First-order sigma-delta / error-diffusion quantizer.
 * value is Q8.8 (0..255.0). The fractional residue is carried into the next
 * frame, giving a sub-LSB time-average at the 250 Hz physical refresh rate. */
static uint8_t effect_q8_to_u8_dither(uint16_t value, uint8_t *error)
{
    uint16_t integer = (uint16_t)(value >> 8);
    uint16_t residue = (uint16_t)(value & 0x00ffu) + (uint16_t)(*error);

    if((residue >= 256u) && (integer < 255u))
    {
        integer++;
        residue -= 256u;
    }

    *error = (uint8_t)residue;
    return (uint8_t)integer;
}

static void effect_fade_all(void)
{
    uint8_t i;

    for(i = 0u; i < WS2812_LED_COUNT; ++i)
    {
        s_r_q8[i] = effect_decay_q8(s_r_q8[i]);
        s_g_q8[i] = effect_decay_q8(s_g_q8[i]);
        s_b_q8[i] = effect_decay_q8(s_b_q8[i]);
    }
}

static void effect_source_color_q8(uint16_t *r_q8, uint16_t *g_q8, uint16_t *b_q8)
{
    uint16_t segment;
    uint16_t position;
    uint16_t t_q8;

    segment = (uint16_t)(s_color_frame / WS2812_EFFECT_SEGMENT_FRAMES);
    position = (uint16_t)(s_color_frame % WS2812_EFFECT_SEGMENT_FRAMES);

    /* 1000 frames/segment with exact endpoints. Internal interpolation retains
     * 1/256-LSB fractions; the output quantizer time-distributes them. */
    t_q8 = (uint16_t)((((uint32_t)position * WS2812_EFFECT_FULL_Q8) +
                       (WS2812_EFFECT_SEGMENT_STEPS / 2u)) /
                      WS2812_EFFECT_SEGMENT_STEPS);

    if(segment == 0u)
    {
        *r_q8 = (uint16_t)(WS2812_EFFECT_FULL_Q8 - t_q8);
        *g_q8 = t_q8;
        *b_q8 = 0u;
    }
    else if(segment == 1u)
    {
        *r_q8 = 0u;
        *g_q8 = (uint16_t)(WS2812_EFFECT_FULL_Q8 - t_q8);
        *b_q8 = t_q8;
    }
    else
    {
        *r_q8 = t_q8;
        *g_q8 = 0u;
        *b_q8 = (uint16_t)(WS2812_EFFECT_FULL_Q8 - t_q8);
    }
}

static void effect_build_output(void)
{
    uint8_t i;

    for(i = 0u; i < WS2812_LED_COUNT; ++i)
    {
        s_pixels[i].r = effect_q8_to_u8_dither(s_r_q8[i], &s_r_quant_error[i]);
        s_pixels[i].g = effect_q8_to_u8_dither(s_g_q8[i], &s_g_quant_error[i]);
        s_pixels[i].b = effect_q8_to_u8_dither(s_b_q8[i], &s_b_quant_error[i]);
    }
}

void WS2812_EffectInit(void)
{
    uint8_t i;

    for(i = 0u; i < WS2812_LED_COUNT; ++i)
    {
        s_pixels[i].r = 0u;
        s_pixels[i].g = 0u;
        s_pixels[i].b = 0u;
        s_r_q8[i] = 0u;
        s_g_q8[i] = 0u;
        s_b_q8[i] = 0u;
        s_r_quant_error[i] = 0u;
        s_g_quant_error[i] = 0u;
        s_b_quant_error[i] = 0u;
    }

    s_color_frame = 0u;
    s_head = 0u;
    s_move_phase = 0u;
}

uint8_t WS2812_EffectStep(void)
{
    uint16_t source_r_q8;
    uint16_t source_g_q8;
    uint16_t source_b_q8;

    effect_fade_all();
    effect_source_color_q8(&source_r_q8, &source_g_q8, &source_b_q8);

    /* Refresh the current head every 4 ms. The head still advances every
     * 48 ms, so chase speed and the original trail time constant stay unchanged. */
    s_r_q8[s_head] = source_r_q8;
    s_g_q8[s_head] = source_g_q8;
    s_b_q8[s_head] = source_b_q8;

    effect_build_output();

    s_move_phase++;
    if(s_move_phase >= WS2812_EFFECT_MOVE_FRAMES)
    {
        s_move_phase = 0u;
        s_head++;
        if(s_head >= WS2812_LED_COUNT)
            s_head = 0u;
    }

    s_color_frame++;
    if(s_color_frame >= WS2812_EFFECT_CYCLE_FRAMES)
        s_color_frame = 0u;

    return WS2812_SendRGB(s_pixels, WS2812_LED_COUNT);
}

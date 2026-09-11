#ifndef __WS2812_EFFECT_H
#define __WS2812_EFFECT_H

#include <stdint.h>

/* Effect frame period used by both the coroOS task and the colour engine. */
#define WS2812_EFFECT_FRAME_MS  4u

/* Non-blocking running-light effect. Call WS2812_EffectStep() periodically. */
void WS2812_EffectInit(void);
uint8_t WS2812_EffectStep(void);

#endif /* __WS2812_EFFECT_H */

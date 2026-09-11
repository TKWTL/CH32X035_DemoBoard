#ifndef TIME_API_H_
#define TIME_API_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Free-running 64-bit CH32X035 SysTick timebase.
 * Clock source: HCLK/8. No interrupt and no compare/reload are used.
 * This API is shared by drivers, protocol engines and debug delays. */
void TIME_Init(void);
uint32_t TIME_Ticks32(void);
uint64_t TIME_Ticks64(void);
uint32_t TIME_Millis(void);
uint32_t TIME_Micros(void);
uint32_t TIME_UsToTicks(uint32_t us);
void TIME_DelayUs(uint32_t us);
void TIME_DelayMs(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* TIME_API_H_ */

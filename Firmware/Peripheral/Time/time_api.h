#ifndef TIME_API_H_
#define TIME_API_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* CH32X035 SysTick timebase.
 *
 * Clock source: HCLK/8 (48 MHz core -> 6 MHz counter).  The hardware counter
 * keeps free-running so the microsecond APIs stay tick-accurate, while a
 * compare interrupt every 1 ms maintains a software millisecond counter that
 * TIME_Millis() returns.  The handler body is TIME_TickHandler(); the
 * application vector table calls it from SysTick_Handler().
 *
 * This API is shared by drivers, protocol engines and debug delays. */
void TIME_Init(void);
void TIME_TickHandler(void);
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

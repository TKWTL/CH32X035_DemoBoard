/********************************** (C) COPYRIGHT *******************************
 * Minimal debug/time compatibility layer for the CH32X035 board.
 * printf is retargeted only to USART1_Async; SDI support is intentionally absent.
 *******************************************************************************/
#ifndef __DEBUG_H
#define __DEBUG_H

#include <stdio.h>
#include <stdint.h>
#include "ch32x035.h"

#ifdef __cplusplus
extern "C" {
#endif

void Delay_Init(void);
void Delay_Us(uint32_t n);
void Delay_Ms(uint32_t n);
uint8_t Debug_Flush(uint32_t timeout_us);

#ifndef DEBUG_PRINT_ENABLE
#define DEBUG_PRINT_ENABLE 1
#endif

#if DEBUG_PRINT_ENABLE
#define PRINT(format, ...) printf(format, ##__VA_ARGS__)
#else
#define PRINT(...)
#endif

#ifdef __cplusplus
}
#endif

#endif /* __DEBUG_H */

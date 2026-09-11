#include "time_api.h"
#include "ch32x035.h"
#include "system_ch32x035.h"

#define STK_CTLR_STE    (1u << 0)
#define STK_CTLR_STIE   (1u << 1)
#define STK_CTLR_STCLK  (1u << 2)
#define STK_CTLR_STRE   (1u << 3)
#define STK_CTLR_MODE   (1u << 4)

#define STK_CNTL_REG    (*(volatile uint32_t *)0xE000F008u)
#define STK_CNTH_REG    (*(volatile uint32_t *)0xE000F00Cu)

static uint32_t s_ticks_per_us = 1u;
static uint32_t s_ticks_per_ms = 1000u;
static uint8_t s_initialized = 0u;

void TIME_Init(void)
{
    if(s_initialized)
        return;

    /* HCLK/8: 48 MHz core -> 6 MHz SysTick.
     * Keep the counter in continuous up-count mode, with no IRQ/reload. */
    s_ticks_per_us = SystemCoreClock / 8000000u;
    if(s_ticks_per_us == 0u)
        s_ticks_per_us = 1u;
    s_ticks_per_ms = s_ticks_per_us * 1000u;

    SysTick->CTLR &= ~(STK_CTLR_STE | STK_CTLR_STIE | STK_CTLR_STCLK |
                      STK_CTLR_STRE | STK_CTLR_MODE);
    SysTick->SR = 0u;
    SysTick->CTLR = STK_CTLR_STE;   /* STCLK=0 => HCLK/8, MODE=0 => up */

    s_initialized = 1u;
}

uint32_t TIME_Ticks32(void)
{
    return STK_CNTL_REG;
}

uint64_t TIME_Ticks64(void)
{
    uint32_t hi1;
    uint32_t lo;
    uint32_t hi2;

    /* Stable 64-bit read on the 32-bit core. */
    do
    {
        hi1 = STK_CNTH_REG;
        lo = STK_CNTL_REG;
        hi2 = STK_CNTH_REG;
    } while(hi1 != hi2);

    return ((uint64_t)hi1 << 32) | lo;
}

uint32_t TIME_UsToTicks(uint32_t us)
{
    return us * s_ticks_per_us;
}

uint32_t TIME_Micros(void)
{
    return (uint32_t)(TIME_Ticks64() / (uint64_t)s_ticks_per_us);
}

uint32_t TIME_Millis(void)
{
    return (uint32_t)(TIME_Ticks64() / (uint64_t)s_ticks_per_ms);
}

void TIME_DelayUs(uint32_t us)
{
    uint32_t start;
    uint32_t wait_ticks;

    if(us == 0u)
        return;

    start = TIME_Ticks32();
    wait_ticks = TIME_UsToTicks(us);
    while((uint32_t)(TIME_Ticks32() - start) < wait_ticks)
    {
    }
}

void TIME_DelayMs(uint32_t ms)
{
    /* Chunking avoids 32-bit tick multiplication overflow for very long waits. */
    while(ms != 0u)
    {
        uint32_t chunk = (ms > 1000u) ? 1000u : ms;
        TIME_DelayUs(chunk * 1000u);
        ms -= chunk;
    }
}

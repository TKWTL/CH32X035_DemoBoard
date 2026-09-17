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

/* Safe distance (SysTick ticks) between the counter and the compare target
 * written by the tick handler; 32 ticks = ~5.3 us at the 6 MHz SysTick clock. */
#define TIME_TICK_MIN_SLACK_TICKS  32u

static uint32_t s_ticks_per_us = 1u;
static uint32_t s_ticks_per_ms = 1000u;
static uint8_t s_initialized = 0u;

/* 1 ms software tick, advanced by the SysTick compare interrupt. */
static volatile uint32_t s_millis = 0u;
/* Absolute compare target for the next millisecond tick. */
static volatile uint64_t s_cmp_next = 0u;

/* Stable 64-bit counter read on the 32-bit core. */
static uint64_t time_read_ticks(void)
{
    uint32_t hi1;
    uint32_t lo;
    uint32_t hi2;

    do
    {
        hi1 = STK_CNTH_REG;
        lo = STK_CNTL_REG;
        hi2 = STK_CNTH_REG;
    } while(hi1 != hi2);

    return ((uint64_t)hi1 << 32) | lo;
}

void TIME_Init(void)
{
    if(s_initialized)
        return;

    /* HCLK/8: 48 MHz core -> 6 MHz SysTick counter. */
    s_ticks_per_us = SystemCoreClock / 8000000u;
    if(s_ticks_per_us == 0u)
        s_ticks_per_us = 1u;
    s_ticks_per_ms = s_ticks_per_us * 1000u;

    /* Counter: continuous up-count at HCLK/8 without auto-reload, so the raw
     * counter keeps free-running and TIME_Micros()/TIME_DelayUs() stay
     * tick-accurate.  The compare interrupt is one-shot hardware: it fires when
     * CNT reaches CMP and TIME_TickHandler() arms the next target. */
    SysTick->CTLR = 0u;
    SysTick->SR = 0u;
    SysTick->CNT = 0u;
    s_millis = 0u;
    s_cmp_next = (uint64_t)s_ticks_per_ms;
    SysTick->CMP = s_cmp_next;

    /* Lowest preemption: the 1 ms tick must never delay USB-PD (priority 0) or
     * any DMA/I2C event handler.  0xE0 = preemption 1 + sub 3 (see APP_IRQ_Init
     * for the full table; IPRIOR bit4:0 are reserved and must stay zero). */
    NVIC_ClearPendingIRQ(SysTick_IRQn);
    NVIC_SetPriority(SysTick_IRQn, 0xE0u);
    NVIC_EnableIRQ(SysTick_IRQn);

    SysTick->CTLR = STK_CTLR_STE | STK_CTLR_STIE;
    /* STCLK=0 => HCLK/8, MODE=0 => up-count, STRE=0 => manual re-arm */

    s_initialized = 1u;
}

uint32_t TIME_Ticks32(void)
{
    return STK_CNTL_REG;
}

uint64_t TIME_Ticks64(void)
{
    return time_read_ticks();
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
    /* Interrupt-updated counter: far cheaper than dividing the tick counter,
     * which matters because some callers run inside interrupt handlers. */
    return s_millis;
}

/* SysTick compare interrupt body, entered through SysTick_Handler(). */
void TIME_TickHandler(void)
{
    uint64_t now;

    /* The compare flag is sticky: clear it before arming the next target. */
    SysTick->SR = 0u;
    s_millis++;

    /* Absolute schedule: adding whole periods keeps the average tick period
     * exact even when the handler is entered a few microseconds late. */
    s_cmp_next += (uint64_t)s_ticks_per_ms;

    now = time_read_ticks();
    while((int64_t)(s_cmp_next - now) <= 0)
    {
        /* Interrupts stayed blocked for more than one period: count the missed
         * milliseconds so TIME_Millis() still tracks real time, and keep the
         * compare target in the future (an equality compare that already
         * passed would never match again). */
        s_millis++;
        s_cmp_next += (uint64_t)s_ticks_per_ms;
    }

    /* Minimum slack: the compare write must land while the counter is still
     * behind the target.  Only a long stall that ended a few ticks in front of
     * a period boundary can get that close; the period is then measured from
     * here instead of racing the counter. */
    if((int64_t)(s_cmp_next - now) < (int64_t)TIME_TICK_MIN_SLACK_TICKS)
        s_cmp_next = now + (uint64_t)s_ticks_per_ms;

    SysTick->CMP = s_cmp_next;
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

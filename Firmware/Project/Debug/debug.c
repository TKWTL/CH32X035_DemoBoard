/********************************** (C) COPYRIGHT *******************************
 * Delay compatibility wrappers and newlib printf retarget.
 * UART hardware and buffering live in Peripheral/USART/usart_async.c.
 *******************************************************************************/
#include "debug.h"
#include "time_api.h"
#include "usart_async.h"
#include <stddef.h>

void Delay_Init(void)
{
    TIME_Init();
}

void Delay_Us(uint32_t n)
{
    TIME_DelayUs(n);
}

void Delay_Ms(uint32_t n)
{
    TIME_DelayMs(n);
}

uint8_t Debug_Flush(uint32_t timeout_us)
{
    return USART1_Async_Flush(timeout_us);
}

__attribute__((used))
int _write(int fd, char *buf, int size)
{
    (void)fd;

    if((buf != 0) && (size > 0))
        (void)USART1_Async_Write((const uint8_t *)buf, (uint16_t)size);

    /* Keep stdio non-fatal if a burst exceeds the bounded 256-byte TX ring.
     * The UART driver counts dropped bytes for diagnostics. */
    return size;
}

__attribute__((used))
void *_sbrk(ptrdiff_t incr)
{
    extern char _end[];
    extern char _heap_end[];
    static char *curbrk = _end;

    if((curbrk + incr < _end) || (curbrk + incr > _heap_end))
        return (void *)-1;

    curbrk += incr;
    return curbrk - incr;
}

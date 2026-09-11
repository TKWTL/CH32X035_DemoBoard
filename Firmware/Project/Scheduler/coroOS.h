#ifndef CORO_OS_H_
#define CORO_OS_H_

#include <stdint.h>
#include "time_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Tiny cooperative scheduler derived from the Protothreads/coroOS model used
 * in TKWTL/3S1P-21700-Power-Bank.
 *
 * - no task stack and no preemption;
 * - each thread stores only a 16-bit continuation plus its explicit static
 *   state;
 * - delays use the board's free-running SysTick timebase and are rollover-safe.
 *
 * IMPORTANT: an automatic local variable does not survive THRD_DELAY/YIELD.
 * Any value needed after a yield point must be static or live outside the
 * thread function.
 */

typedef struct
{
    uint16_t line;
} coro_pt_t;

typedef uint8_t (*coro_thread_fn_t)(coro_pt_t *pt);

typedef struct
{
    const coro_thread_fn_t *threads;
    coro_pt_t *states;
    uint8_t count;
} coro_scheduler_t;

enum
{
    CORO_WAITING = 0u,
    CORO_YIELDED = 1u,
    CORO_ENDED   = 2u
};

void CoroOS_Init(coro_scheduler_t *scheduler,
                 const coro_thread_fn_t *threads,
                 coro_pt_t *states,
                 uint8_t count);
void CoroOS_RunOnce(coro_scheduler_t *scheduler);

#define CORO_INIT(pt)              do { (pt)->line = 0u; } while(0)
#define CORO_BEGIN(pt)             switch((pt)->line) { case 0:
#define CORO_END(pt)               default:; } CORO_INIT(pt); return CORO_ENDED

#define CORO_WAIT_UNTIL(pt, cond)  \
    do {                            \
        (pt)->line = (uint16_t)__LINE__; \
        case __LINE__:             \
        if(!(cond)) return CORO_WAITING; \
    } while(0)

#define CORO_YIELD(pt)             \
    do {                            \
        (pt)->line = (uint16_t)__LINE__; \
        return CORO_YIELDED;       \
        case __LINE__:;            \
    } while(0)

#define THRD_DECLARE(name)         static uint8_t name(coro_pt_t *pt)
#define THRD_BEGIN                 CORO_BEGIN(pt)
#define THRD_YIELD                 CORO_YIELD(pt)
#define THRD_UNTIL(cond)           CORO_WAIT_UNTIL(pt, (cond))
#define THRD_WHILE(cond)           CORO_WAIT_UNTIL(pt, !(cond))
#define THRD_DELAY(ms)             \
    do {                            \
        static uint32_t _coro_deadline; \
        _coro_deadline = TIME_Millis() + (uint32_t)(ms); \
        CORO_WAIT_UNTIL(pt, (int32_t)(TIME_Millis() - _coro_deadline) >= 0); \
    } while(0)
#define THRD_SPAWN_NOARG(func)     \
    do {                            \
        static coro_pt_t _coro_subpt; \
        CORO_INIT(&_coro_subpt);   \
        CORO_WAIT_UNTIL(pt, (func(&_coro_subpt) == CORO_ENDED)); \
    } while(0)
#define THRD_END                   CORO_END(pt)

#ifdef __cplusplus
}
#endif

#endif /* CORO_OS_H_ */

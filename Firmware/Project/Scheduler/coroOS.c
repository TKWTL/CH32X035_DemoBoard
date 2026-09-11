#include "coroOS.h"

void CoroOS_Init(coro_scheduler_t *scheduler,
                 const coro_thread_fn_t *threads,
                 coro_pt_t *states,
                 uint8_t count)
{
    uint8_t i;

    if((scheduler == 0) || (threads == 0) || (states == 0) || (count == 0u))
        return;

    scheduler->threads = threads;
    scheduler->states = states;
    scheduler->count = count;

    for(i = 0; i < count; i++)
        CORO_INIT(&states[i]);
}

void CoroOS_RunOnce(coro_scheduler_t *scheduler)
{
    uint8_t i;

    if((scheduler == 0) || (scheduler->threads == 0) ||
       (scheduler->states == 0) || (scheduler->count == 0u))
        return;

    for(i = 0; i < scheduler->count; i++)
        (void)scheduler->threads[i](&scheduler->states[i]);
}

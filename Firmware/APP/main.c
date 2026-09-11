/********************************** (C) COPYRIGHT *******************************
 * CH32X035 custom development-board application entry.
 *******************************************************************************/

#include "main.h"
#include "debug.h"
#include "usart_async.h"
#include "app_tasks.h"

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();

    USART1_Async_Init(921600u);
    APP_Tasks_Init();

    while(1)
        APP_Tasks_RunOnce();
}

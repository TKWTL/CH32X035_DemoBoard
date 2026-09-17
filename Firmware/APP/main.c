/********************************** (C) COPYRIGHT *******************************
 * CH32X035 custom development-board application entry.
 *******************************************************************************/

#include "main.h"
#include "debug.h"
#include "usart_async.h"
#include "time_api.h"
#include "pd.h"
#include "app_tasks.h"
#include "ch32x035_it.h"

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    /* Preemption policy + VTF entries (see ch32x035_it.c).  USBPD is the only
     * preemption-0 source; the deepest normal nesting is 2 levels (DMA <- PD),
     * matching the V4C hardware stack. */
    APP_IRQ_Init();

    SystemCoreClockUpdate();
    Delay_Init();
    TIME_Init();

    /* The board is VBUS-powered and presents passive Rd from power-up. Start
     * the USB-PD PHY before UART/I2C/UI setup so the first Source_Capabilities
     * burst is not lost while the MCU is still booting. */
    PD_Init();

    USART1_Async_Init(921600u);
    APP_Tasks_Init();

    while(1)
    {
        APP_Tasks_RunOnce();
        APP_Tasks_Idle();
    }
}

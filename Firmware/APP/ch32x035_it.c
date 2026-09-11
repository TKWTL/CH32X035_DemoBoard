/********************************** (C) COPYRIGHT *******************************
 * CH32X035 application interrupt-vector entry points.
 * Peripheral modules expose handler bodies; vector ownership remains in APP.
 *******************************************************************************/
#include "ch32x035_it.h"
#include "usart_async.h"
#include "ch32x035.h"

void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel5_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void NMI_Handler(void)
{
    while(1) { }
}

static void APP_EmergencyUartWrite(const char *text)
{
    /* Fault-only path: stop TX DMA and use the USART data register directly.
     * This deliberately does not depend on the scheduler/heap/printf. */
    USART_DMACmd(USART1, USART_DMAReq_Tx, DISABLE);
    DMA_Cmd(DMA1_Channel4, DISABLE);

    while(*text)
    {
        uint32_t guard = 200000u;
        while((USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) && guard)
            guard--;
        if(!guard)
            return;
        USART_SendData(USART1, (uint8_t)*text++);
    }
}

void HardFault_Handler(void)
{
    APP_EmergencyUartWrite("\r\n[FAULT] HardFault - halted (no software reset)\r\n");
    while(1) { }
}

void DMA1_Channel4_IRQHandler(void)
{
    USART1_Async_TxDMA_IRQHandler();
}

void DMA1_Channel5_IRQHandler(void)
{
    USART1_Async_RxDMA_IRQHandler();
}

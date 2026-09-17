/********************************** (C) COPYRIGHT *******************************
 * CH32X035 application interrupt-vector entry points.
 * Peripheral modules expose handler bodies; vector ownership remains in APP.
 *******************************************************************************/
#include "ch32x035_it.h"
#include "app_tasks.h"
#include "debug.h"
#include "usart_async.h"
#include "i2c_api.h"
#include "time_api.h"
#include "ch32x035.h"

void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel5_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel6_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel7_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void I2C1_EV_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void I2C1_ER_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void SysTick_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
/* Defined in pd_port.c; declared here for the VTF entry map below. */
void USBPD_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

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

void DMA1_Channel6_IRQHandler(void)
{
    I2C_API_TxDMA_IRQHandler();
}

void DMA1_Channel7_IRQHandler(void)
{
    I2C_API_RxDMA_IRQHandler();
}

void I2C1_EV_IRQHandler(void)
{
    I2C_API_EV_IRQHandler();
}

void I2C1_ER_IRQHandler(void)
{
    I2C_API_ER_IRQHandler();
}

/* 1 ms timebase tick.  It keeps the millisecond counter of the shared time API
 * alive while the main loop is parked in WFI, and refreshes the compare target
 * for the next period. */
void SysTick_Handler(void)
{
    TIME_TickHandler();
}

/* --------------------------------------------------------------------------
 * Interrupt policy for the QingKe V4C core (two hardware stack levels).
 *
 * Preemption level 0 - USBPD only.  It is the single source that may preempt
 * another ISR, so the deepest normal nesting is  DMA(n) <- USBPD(0) = two
 * levels, exactly what the hardware stack supports.
 * Preemption level 1 - every other peripheral interrupt (USART/I2C DMA,
 * I2C EV/ER, SysTick); level-1 handlers never preempt each other.
 *
 * IPRIOR layout while nesting is enabled (CSR 0x804 bit1 = 1, set by the
 * startup file):  bit7 = preemption (0 = high, 1 = low), bit6:5 = sub-priority,
 * bit4:0 = reserved (keep zero).  Keep this table in sync with the per-
 * peripheral NVIC_Init() calls - a NEW interrupt must be added here too,
 * because the reset value (0x00) would otherwise place it at the USBPD level
 * and degrade the PD response time.
 * ------------------------------------------------------------------------ */
void APP_IRQ_Init(void)
{
    NVIC_SetPriority(USBPD_IRQn, 0x00u);         /* preemption 0: PD only */

    NVIC_SetPriority(DMA1_Channel4_IRQn, 0xC0u); /* USART TX DMA, sub 2 */
    NVIC_SetPriority(DMA1_Channel5_IRQn, 0xE0u); /* USART RX DMA, sub 3 */
    NVIC_SetPriority(DMA1_Channel6_IRQn, 0xA0u); /* I2C TX   DMA, sub 1 */
    NVIC_SetPriority(DMA1_Channel7_IRQn, 0xA0u); /* I2C RX   DMA, sub 1 */
    NVIC_SetPriority(I2C1_EV_IRQn,       0x80u); /* I2C events,   sub 0 */
    NVIC_SetPriority(I2C1_ER_IRQn,       0x80u); /* I2C errors,   sub 0 */
    NVIC_SetPriority(SysTick_IRQn,       0xE0u); /* 1 ms tick,    sub 3 */

    /* Vector-table-free entries: on acceptance the PFIC jumps straight to the
     * handler and the vector fetch is skipped.  VTF0..VTF3 are the four VTF
     * channels of the V4C core; USBPD first, then the three latency-relevant
     * completions (USART TX, USART RX, I2C1 events). */
    SetVTFIRQ((uint32_t)USBPD_IRQHandler, USBPD_IRQn, 0, ENABLE);
    SetVTFIRQ((uint32_t)DMA1_Channel4_IRQHandler, DMA1_Channel4_IRQn, 1, ENABLE);
    SetVTFIRQ((uint32_t)DMA1_Channel5_IRQHandler, DMA1_Channel5_IRQn, 2, ENABLE);
    SetVTFIRQ((uint32_t)I2C1_EV_IRQHandler, I2C1_EV_IRQn, 3, ENABLE);
}

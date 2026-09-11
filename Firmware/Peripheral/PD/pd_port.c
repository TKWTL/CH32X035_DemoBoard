/********************************** (C) COPYRIGHT *******************************
 * USB-PD hardware port for CH32X035.
 *
 * This is the only PD file that should directly touch CH32X035 USBPD/RCC/GPIO/
 * AFIO/NVIC registers.  The upper PD protocol/policy layer talks only through
 * pd_port.h so a future MCU port replaces this file instead of the PD protocol state machine.
 ******************************************************************************/

#include "debug.h"
#include "main.h"
#include "pd_port.h"
#include "time_api.h"

#define PD_PORT_GOODCRC_TYPE       0x01u
#define PD_PORT_MIN_RX_FRAME_BYTES 6u

/* CONFIG and PORT_CC1/2 are 16-bit registers on CH32X035.
 * Do not narrow complemented masks to uint8_t: that would clear CONFIG
 * bits 8..15, including IE_RX_ACT/IE_RX_RESET/IE_TX_END. */

static uint8_t *s_rx_buffer;
static volatile uint8_t s_message_pending;
static volatile uint8_t s_hard_reset_pending;
static volatile uint8_t s_auto_ack_pr_role;
static uint8_t s_ack_buf[2] __attribute__((aligned(4)));
static volatile uint16_t s_auto_ack_started;
static volatile uint16_t s_auto_ack_completed;
static volatile uint8_t s_auto_ack_inflight;
static volatile uint32_t s_last_auto_ack_completed_us;

/* Keep the timing-critical USBPD vector in this translation unit.  The
 * WCH reference implementation does the same; avoid an extra APP-layer
 * wrapper between the fast vector entry and the GoodCRC response path. */
void USBPD_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

static void PD_Port_SendRaw(uint8_t wait_complete,
                           const uint8_t *buffer,
                           uint8_t length,
                           uint8_t wch_tx_sel)
{
    if((USBPD->CONFIG & CC_SEL) == CC_SEL)
        USBPD->PORT_CC2 |= CC_LVE;
    else
        USBPD->PORT_CC1 |= CC_LVE;

    USBPD->BMC_CLK_CNT = UPD_TMR_TX_48M;
    USBPD->DMA = (uint32_t)(uintptr_t)buffer;
    USBPD->TX_SEL = wch_tx_sel;
    USBPD->BMC_TX_SZ = length;
    USBPD->CONTROL |= PD_TX_EN;
    USBPD->STATUS &= BMC_AUX_INVALID;
    USBPD->CONTROL |= BMC_START;

    if(wait_complete)
    {
        while((USBPD->STATUS & IF_TX_END) == 0)
        {
        }

        USBPD->STATUS |= IF_TX_END;

        if((USBPD->CONFIG & CC_SEL) == CC_SEL)
            USBPD->PORT_CC2 &= (uint16_t)~(uint16_t)CC_LVE;
        else
            USBPD->PORT_CC1 &= (uint16_t)~(uint16_t)CC_LVE;

        /* Return to RX so the protocol layer can wait for GoodCRC. */
        USBPD->CONFIG |= PD_ALL_CLR;
        USBPD->CONFIG &= (uint16_t)~(uint16_t)PD_ALL_CLR;
        USBPD->CONTROL &= (uint8_t)~PD_TX_EN;
        USBPD->DMA = (uint32_t)(uintptr_t)s_rx_buffer;
        USBPD->BMC_CLK_CNT = UPD_TMR_RX_48M;
        USBPD->CONTROL |= BMC_START;
    }
}

void USBPD_IRQHandler(void)
{
    /* A received Hard Reset terminates the current protocol transaction.
     * Give it priority over packet/TX events so stale flags cannot surface a
     * normal message after the reset. */
    if(USBPD->STATUS & IF_RX_RESET)
    {
        USBPD->STATUS |= IF_RX_RESET;
        s_auto_ack_inflight = 0u;
        s_message_pending = 0u;
        s_hard_reset_pending = 1u;
        NVIC_DisableIRQ(USBPD_IRQn);
        return;
    }

    if(USBPD->STATUS & IF_RX_ACT)
    {
        USBPD->STATUS |= IF_RX_ACT;

        if((USBPD->STATUS & MASK_PD_STAT) == PD_RX_SOP0)
        {
            uint8_t count = USBPD->BMC_BYTE_CNT;

            if((s_rx_buffer != 0) && (count >= PD_PORT_MIN_RX_FRAME_BYTES))
            {
                /* WCH sample behaviour: automatically answer every SOP message
                 * except GoodCRC with GoodCRC after the required short delay. */
                if((count != PD_PORT_MIN_RX_FRAME_BYTES) ||
                   ((s_rx_buffer[0] & 0x1Fu) != PD_PORT_GOODCRC_TYPE))
                {
                    TIME_DelayUs(30);
                    s_ack_buf[0] = 0x41;
                    s_ack_buf[1] = (s_rx_buffer[1] & 0x0Eu) | s_auto_ack_pr_role;

                    /* The policy layer may only consume this RX packet after
                     * its GoodCRC has physically finished.  Clear any stale
                     * TX_END before starting the ACK and mark the ownership
                     * explicitly; otherwise a stale TX_END can create a fake
                     * message event while the new GoodCRC is still on the wire. */
                    USBPD->STATUS |= IF_TX_END;
                    USBPD->CONFIG |= IE_TX_END;
                    s_auto_ack_inflight = 1u;
                    s_auto_ack_started++;
                    PD_Port_SendRaw(0, s_ack_buf, 2, UPD_SOP0);
                    return;
                }
            }
        }
    }

    if(USBPD->STATUS & IF_TX_END)
    {
        USBPD->PORT_CC1 &= (uint16_t)~(uint16_t)CC_LVE;
        USBPD->PORT_CC2 &= (uint16_t)~(uint16_t)CC_LVE;
        USBPD->STATUS |= IF_TX_END;

        if(s_auto_ack_inflight)
        {
            s_auto_ack_inflight = 0u;
            s_auto_ack_completed++;
            s_last_auto_ack_completed_us = TIME_Micros();
            s_message_pending = 1u;
            NVIC_DisableIRQ(USBPD_IRQn);
        }
    }
}

void PD_Port_Init(uint8_t *rx_buffer, uint16_t rx_buffer_size)
{
    GPIO_InitTypeDef gpio = {0};

    s_rx_buffer = rx_buffer;
    s_message_pending = 0;
    s_hard_reset_pending = 0;
    s_auto_ack_pr_role = 0;
    s_auto_ack_started = 0;
    s_auto_ack_completed = 0;
    s_auto_ack_inflight = 0u;
    s_last_auto_ack_completed_us = 0u;

    (void)rx_buffer_size; /* DMA bounds are fixed by the protocol buffer owner. */

    RCC_APB2PeriphClockCmd(CC1_GPIO_CLK | CC2_GPIO_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBPD, ENABLE);

    gpio.GPIO_Pin = CC1_Pin;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(CC1_GPIO_Port, &gpio);

    gpio.GPIO_Pin = CC2_Pin;
    GPIO_Init(CC2_GPIO_Port, &gpio);

    AFIO->CTLR |= USBPD_IN_HVT | USBPD_PHY_V33;

    USBPD->CONFIG = PD_DMA_EN;
    USBPD->STATUS = BUF_ERR | IF_RX_BIT | IF_RX_BYTE |
                    IF_RX_ACT | IF_RX_RESET | IF_TX_END;
}

void PD_Port_SetPowerRole(PD_Port_PowerRole role)
{
    if(role == PD_PORT_ROLE_SOURCE)
    {
        s_auto_ack_pr_role = 1;
        USBPD->PORT_CC1 = CC_CMP_66 | CC_PU_330;
        USBPD->PORT_CC2 = CC_CMP_66 | CC_PU_330;
    }
    else
    {
        s_auto_ack_pr_role = 0;
        USBPD->PORT_CC1 = CC_CMP_66 | CC_PD;
        USBPD->PORT_CC2 = CC_CMP_66 | CC_PD;
    }
}

void PD_Port_RxStart(void)
{
    /* Do not reset the PHY while an interrupt-driven GoodCRC is still being
     * transmitted.  PD_ALL_CLR here would truncate the ACK and the Source can
     * legitimately escalate to Hard Reset. */
    if(s_auto_ack_inflight)
        return;

    USBPD->CONFIG |= PD_ALL_CLR;
    USBPD->CONFIG &= (uint16_t)~(uint16_t)PD_ALL_CLR;
    USBPD->CONFIG |= IE_RX_ACT | IE_RX_RESET | PD_DMA_EN;
    USBPD->DMA = (uint32_t)(uintptr_t)s_rx_buffer;
    USBPD->CONTROL &= (uint8_t)~PD_TX_EN;
    USBPD->BMC_CLK_CNT = UPD_TMR_RX_48M;
    USBPD->CONTROL |= BMC_START;
    NVIC_EnableIRQ(USBPD_IRQn);
}

PD_Port_CC PD_Port_DetectAttach(void)
{
    uint8_t cc1_present = 0;
    uint8_t cc2_present = 0;

    /* This backend currently implements the Sink attach path used by the
     * product: test both CC pins against the same comparator threshold as the
     * original WCH USBPD_SNK example. */
    USBPD->PORT_CC1 &= (uint16_t)~(uint16_t)(CC_CMP_Mask | PA_CC_AI);
    USBPD->PORT_CC1 |= CC_CMP_22;
    TIME_DelayUs(2);
    if(USBPD->PORT_CC1 & PA_CC_AI)
        cc1_present = 1;

    USBPD->PORT_CC2 &= (uint16_t)~(uint16_t)(CC_CMP_Mask | PA_CC_AI);
    USBPD->PORT_CC2 |= CC_CMP_22;
    TIME_DelayUs(2);
    if(USBPD->PORT_CC2 & PA_CC_AI)
        cc2_present = 1;

    if((USBPD->PORT_CC1 & CC_PD) == 0)
        return PD_PORT_CC_NONE;

    if(cc1_present)
        return PD_PORT_CC1;
    if(cc2_present)
        return PD_PORT_CC2;

    return PD_PORT_CC_NONE;
}

void PD_Port_SelectCC(PD_Port_CC cc)
{
    if(cc == PD_PORT_CC2)
        USBPD->CONFIG |= CC_SEL;
    else
        USBPD->CONFIG &= (uint16_t)~(uint16_t)CC_SEL;
}

uint8_t PD_Port_TransactSOP(const uint8_t *buffer,
                            uint8_t length,
                            uint8_t max_attempts,
                            PD_Port_TxDiag *diag)
{
    uint8_t attempt;

    if(diag != 0)
    {
        diag->saw_frame = 0u;
        diag->byte_count = 0u;
        diag->message_type = 0u;
        diag->pd_status = 0u;
        diag->saw_hard_reset = 0u;
        diag->attempts = 0u;
        diag->ack_to_first_tx_us = 0u;
    }

    if((buffer == 0) || (length < 2u) || (max_attempts == 0u))
        return 0u;

    for(attempt = 0u; attempt < max_attempts; attempt++)
    {
        uint8_t cnt = 250u;

        /* A Source can legally enter Hard Reset while we are waiting for the
         * Request transaction.  Check/reset-latch it before PD_ALL_CLR in the
         * next TX attempt can erase the hardware evidence. */
        if(USBPD->STATUS & IF_RX_RESET)
        {
            USBPD->STATUS |= IF_RX_RESET;
            s_hard_reset_pending = 1u;
            if(diag != 0)
            {
                diag->saw_hard_reset = 1u;
                diag->attempts = attempt;
            }
            return 0u;
        }

        NVIC_DisableIRQ(USBPD_IRQn);

        if((attempt == 0u) && (diag != 0) && (s_last_auto_ack_completed_us != 0u))
            diag->ack_to_first_tx_us = (uint32_t)(TIME_Micros() - s_last_auto_ack_completed_us);

        PD_Port_SendRaw(1u, buffer, length, UPD_SOP0);
        if(diag != 0)
            diag->attempts = (uint8_t)(attempt + 1u);

        while(--cnt)
        {
            /* IF_RX_RESET is not an ordinary RX frame.  Preserve it in
             * software immediately; otherwise the next retry's PD_ALL_CLR
             * makes the failure look like a silent GoodCRC timeout. */
            if(USBPD->STATUS & IF_RX_RESET)
            {
                USBPD->STATUS |= IF_RX_RESET;
                s_hard_reset_pending = 1u;
                if(diag != 0)
                    diag->saw_hard_reset = 1u;
                return 0u;
            }

            if((USBPD->STATUS & IF_RX_ACT) == IF_RX_ACT)
            {
                uint8_t status = USBPD->STATUS;
                uint8_t count = USBPD->BMC_BYTE_CNT;
                uint8_t type = 0u;

                if((s_rx_buffer != 0) && (count >= PD_PORT_MIN_RX_FRAME_BYTES))
                    type = (uint8_t)(s_rx_buffer[0] & 0x1Fu);

                if(diag != 0)
                {
                    diag->saw_frame = 1u;
                    diag->byte_count = count;
                    diag->message_type = type;
                    diag->pd_status = (uint8_t)(status & MASK_PD_STAT);
                }

                USBPD->STATUS |= IF_RX_ACT;

                if(((status & MASK_PD_STAT) == PD_RX_SOP0) &&
                   (count == PD_PORT_MIN_RX_FRAME_BYTES) &&
                   (type == PD_PORT_GOODCRC_TYPE))
                {
                    return 1u;
                }
            }

            TIME_DelayUs(3u);
        }
    }

    return 0u;
}


void PD_Port_SendHardReset(void)
{
    /* Hard Reset has no GoodCRC transaction. Keep it separate from normal SOP
     * messages so upper layers cannot accidentally bypass the atomic helper. */
    NVIC_DisableIRQ(USBPD_IRQn);
    PD_Port_SendRaw(1u, 0, 0u, UPD_HARD_RESET);
}

uint8_t PD_Port_AutoAckBusy(void)
{
    return s_auto_ack_inflight;
}

uint8_t PD_Port_WaitAutoAckComplete(uint32_t timeout_us)
{
    uint32_t start = TIME_Micros();

    while(s_auto_ack_inflight)
    {
        if((uint32_t)(TIME_Micros() - start) >= timeout_us)
            return 0u;
    }

    return 1u;
}

void PD_Port_GetAutoAckStats(uint16_t *started, uint16_t *completed)
{
    if(started != 0)
        *started = s_auto_ack_started;
    if(completed != 0)
        *completed = s_auto_ack_completed;
}

uint8_t PD_Port_MessagePending(void)
{
    return s_message_pending;
}

void PD_Port_ClearMessageEvent(void)
{
    s_message_pending = 0;
}

uint8_t PD_Port_HardResetPending(void)
{
    return s_hard_reset_pending;
}

void PD_Port_ClearHardResetEvent(void)
{
    s_hard_reset_pending = 0;
}

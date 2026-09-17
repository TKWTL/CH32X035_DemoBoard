/********************************** (C) COPYRIGHT *******************************
 * USB-PD hardware port for CH32X035 (Sink).
 *
 * - Sink-originated SOP traffic: atomic TX -> TX_END -> immediate RX ->
 *   Source GoodCRC transaction (PD_Port_TransactSOP()).
 * - Source-originated SOP traffic: DMA'd straight into the policy buffer;
 *   auto-GoodCRC is sent in USBPD_IRQHandler() and the packet is published
 *   only after that GoodCRC has physically ended.
 ******************************************************************************/

#include "debug.h"
#include "main.h"
#include "pd_port.h"
#include "time_api.h"
#include <string.h>

#define PD_PORT_GOODCRC_TYPE       0x01u
#define PD_PORT_MIN_RX_FRAME_BYTES 6u
#define PD_PORT_MAX_FRAME_BYTES    34u
#define PD_PORT_TX_MAX_ATTEMPTS    3u

/* CONFIG and PORT_CC1/2 are 16-bit registers on CH32X035. */
enum
{
    PD_PHY_IDLE = 0,
    PD_PHY_TX,
    PD_PHY_WAIT_GOODCRC
};

static uint8_t *s_rx_buffer;
static uint8_t s_tx_buf[PD_PORT_MAX_FRAME_BYTES] __attribute__((aligned(4)));
static uint8_t s_ack_buf[PD_PORT_MAX_FRAME_BYTES] __attribute__((aligned(4)));

static volatile uint8_t  s_phy_state;
static volatile uint8_t  s_tx_result;
static volatile uint8_t  s_message_pending;
static volatile uint8_t  s_hard_reset_pending;
static volatile uint8_t  s_auto_ack_pr_role;
static volatile uint8_t  s_auto_ack_inflight;
static volatile uint8_t  s_rx_armed;

static volatile uint16_t s_auto_ack_started;
static volatile uint16_t s_auto_ack_completed;

static volatile uint8_t  s_last_detect_cc1;
static volatile uint8_t  s_last_detect_cc2;

static volatile uint16_t s_tx_end_timeouts;
static volatile uint16_t s_goodcrc_timeouts;
static volatile uint16_t s_ack_tx_timeouts;

static volatile uint8_t  s_tx_start_status;
static volatile uint8_t  s_tx_start_control;
static volatile uint8_t  s_tx_start_sop;
static volatile uint8_t  s_tx_start_len;
static volatile uint16_t s_tx_start_cc1;
static volatile uint16_t s_tx_start_cc2;
static volatile uint8_t  s_tx_dma_intact;

static volatile uint8_t  s_hr_pd_stat;
static volatile uint8_t  s_hr_byte_count;

void USBPD_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

static void pd_port_latch_hard_reset(void)
{
    s_hard_reset_pending = 1u;
    s_hr_pd_stat = (uint8_t)(USBPD->STATUS & MASK_PD_STAT);
    s_hr_byte_count = USBPD->BMC_BYTE_CNT;
}

static void pd_port_release_cc(void)
{
    USBPD->PORT_CC1 &= (uint16_t)~(uint16_t)CC_LVE;
    USBPD->PORT_CC2 &= (uint16_t)~(uint16_t)CC_LVE;
}

static void pd_port_begin_tx(const uint8_t *buffer,
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

    s_rx_armed = 0u;
}

static void pd_port_enter_rx(void)
{
    pd_port_release_cc();

    USBPD->CONFIG |= PD_ALL_CLR;
    USBPD->CONFIG &= (uint16_t)~(uint16_t)PD_ALL_CLR;
    USBPD->CONTROL &= (uint8_t)~(uint8_t)PD_TX_EN;
    USBPD->DMA = (uint32_t)(uintptr_t)s_rx_buffer;
    USBPD->BMC_CLK_CNT = UPD_TMR_RX_48M;
    USBPD->CONTROL |= BMC_START;

    s_rx_armed = 1u;
}

/* USBPD IRQ - DemoBoard ownership model:
 *   Source packet -> RX DMA into the policy buffer -> 30 us -> GoodCRC from
 *   s_ack_buf -> IF_TX_END -> message_pending = 1 (no intermediate mailbox). */
void USBPD_IRQHandler(void)
{
    uint8_t status;

    status = (uint8_t)USBPD->STATUS;

    if(status & IF_RX_RESET)
    {
        USBPD->STATUS |= IF_RX_RESET;

        pd_port_release_cc();
        USBPD->CONTROL &= (uint8_t)~(uint8_t)PD_TX_EN;

        s_auto_ack_inflight = 0u;
        s_phy_state = PD_PHY_IDLE;
        s_tx_result = PD_PORT_TX_RESULT_NONE;
        s_message_pending = 0u;
        s_rx_armed = 0u;

        pd_port_latch_hard_reset();

        NVIC_DisableIRQ(USBPD_IRQn);
        return;
    }

    if(status & IF_RX_ACT)
    {
        USBPD->STATUS |= IF_RX_ACT;

        if((status & MASK_PD_STAT) == PD_RX_SOP0)
        {
            uint8_t count = USBPD->BMC_BYTE_CNT;
            uint8_t type = 0u;

            if((s_rx_buffer != 0) && (count >= 2u))
                type = (uint8_t)(s_rx_buffer[0] & 0x1Fu);

            /* Foreground SOP traffic uses PD_Port_TransactSOP() with the USBPD
             * NVIC masked, so normally GoodCRC never reaches this branch.
             * Keep the recognition here only for API compatibility. */
            if((s_phy_state == PD_PHY_TX) ||
               (s_phy_state == PD_PHY_WAIT_GOODCRC))
            {
                uint8_t is_goodcrc =
                    ((s_rx_buffer != 0) &&
                     (type == PD_PORT_GOODCRC_TYPE) &&
                     ((s_rx_buffer[1] & 0x70u) == 0u)) ? 1u : 0u;

                if(is_goodcrc != 0u)
                {
                    s_tx_result = PD_PORT_TX_RESULT_OK;
                    s_phy_state = PD_PHY_IDLE;
                }
            }
            else if((s_phy_state == PD_PHY_IDLE) &&
                    (s_auto_ack_inflight == 0u) &&
                    (s_rx_buffer != 0) &&
                    (count >= PD_PORT_MIN_RX_FRAME_BYTES))
            {
                /* WCH/DemoBoard behaviour: reply to every ordinary SOP packet,
                 * but never reply to GoodCRC itself. */
                if((count != PD_PORT_MIN_RX_FRAME_BYTES) ||
                   (type != PD_PORT_GOODCRC_TYPE))
                {
                    TIME_DelayUs(30u);

                    {
                        uint8_t rx_revision = (uint8_t)(s_rx_buffer[0] & 0xC0u);

                        /* GoodCRC uses the partner's Message Header revision.
                         * PD 3.1/3.2 still encode SpecRevision as 10b.  Clamp
                         * the reserved 11b value to Rev3 for robustness. */
                        if(rx_revision == 0xC0u)
                            rx_revision = 0x80u;

                        s_ack_buf[0] = (uint8_t)(PD_PORT_GOODCRC_TYPE | rx_revision);
                        s_ack_buf[1] =
                            (uint8_t)((s_rx_buffer[1] & 0x0Eu) |
                                      s_auto_ack_pr_role);
                    }

                    USBPD->STATUS |= IF_TX_END;
                    USBPD->CONFIG |= IE_TX_END;

                    s_auto_ack_inflight = 1u;
                    s_auto_ack_started++;

                    pd_port_begin_tx(s_ack_buf, 2u, UPD_SOP0);
                    return;
                }
            }
        }
    }

    if(status & IF_TX_END)
    {
        pd_port_release_cc();
        USBPD->STATUS |= IF_TX_END;

        if(s_auto_ack_inflight != 0u)
        {
            s_auto_ack_inflight = 0u;
            s_auto_ack_completed++;

            /* The RX bytes are still in s_rx_buffer because ACK TX used the
             * separate s_ack_buf.  Publish only after GoodCRC is physically
             * finished, then let the policy layer consume the packet before
             * re-arming RX. */
            s_message_pending = 1u;
            s_rx_armed = 0u;
            NVIC_DisableIRQ(USBPD_IRQn);
            return;
        }

        if(s_phy_state == PD_PHY_TX)
        {
            pd_port_enter_rx();
            s_phy_state = PD_PHY_WAIT_GOODCRC;
        }
    }
}

/* -------------------------------------------------------------------------- */

void PD_Port_Init(uint8_t *rx_buffer, uint16_t rx_buffer_size)
{
    GPIO_InitTypeDef gpio = {0};

    s_rx_buffer = rx_buffer;

    s_phy_state = PD_PHY_IDLE;
    s_tx_result = PD_PORT_TX_RESULT_NONE;
    s_message_pending = 0u;
    s_hard_reset_pending = 0u;
    s_auto_ack_pr_role = 0u;
    s_auto_ack_inflight = 0u;
    s_rx_armed = 0u;

    s_auto_ack_started = 0u;
    s_auto_ack_completed = 0u;

    s_last_detect_cc1 = 0u;
    s_last_detect_cc2 = 0u;

    s_tx_end_timeouts = 0u;
    s_goodcrc_timeouts = 0u;
    s_ack_tx_timeouts = 0u;

    s_tx_start_status = 0u;
    s_tx_start_control = 0u;
    s_tx_start_sop = 0u;
    s_tx_start_len = 0u;
    s_tx_start_cc1 = 0u;
    s_tx_start_cc2 = 0u;
    s_tx_dma_intact = 0u;

    s_hr_pd_stat = 0u;
    s_hr_byte_count = 0u;

    (void)rx_buffer_size;

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

    /* Match the stable DemoBoard baseline. */
    USBPD->CONFIG = PD_DMA_EN;
    USBPD->STATUS = BUF_ERR | IF_RX_BIT | IF_RX_BYTE |
                    IF_RX_ACT | IF_RX_RESET | IF_TX_END;
}

void PD_Port_SetPowerRole(PD_Port_PowerRole role)
{
    if(role == PD_PORT_ROLE_SOURCE)
    {
        s_auto_ack_pr_role = 1u;
        USBPD->PORT_CC1 = CC_CMP_66 | CC_PU_330;
        USBPD->PORT_CC2 = CC_CMP_66 | CC_PU_330;
    }
    else
    {
        s_auto_ack_pr_role = 0u;
        USBPD->PORT_CC1 = CC_CMP_66 | CC_PD;
        USBPD->PORT_CC2 = CC_CMP_66 | CC_PD;
    }
}

void PD_Port_RxStart(void)
{
    /* PD_ALL_CLR while GoodCRC is on the wire truncates the ACK. */
    if((s_auto_ack_inflight != 0u) || (s_phy_state != PD_PHY_IDLE))
        return;

    if(s_rx_armed != 0u)
        return;

    USBPD->CONFIG |= PD_ALL_CLR;
    USBPD->CONFIG &= (uint16_t)~(uint16_t)PD_ALL_CLR;
    USBPD->CONFIG |= IE_RX_ACT | IE_RX_RESET | PD_DMA_EN;

    pd_port_release_cc();

    USBPD->DMA = (uint32_t)(uintptr_t)s_rx_buffer;
    USBPD->CONTROL &= (uint8_t)~(uint8_t)PD_TX_EN;
    USBPD->BMC_CLK_CNT = UPD_TMR_RX_48M;
    USBPD->CONTROL |= BMC_START;

    s_rx_armed = 1u;
    NVIC_EnableIRQ(USBPD_IRQn);
}

PD_Port_CC PD_Port_DetectAttach(void)
{
    uint8_t cc1_present = 0u;
    uint8_t cc2_present = 0u;

    USBPD->PORT_CC1 &= (uint16_t)~(uint16_t)(CC_CMP_Mask | PA_CC_AI);
    USBPD->PORT_CC1 |= CC_CMP_22;
    TIME_DelayUs(2u);
    if(USBPD->PORT_CC1 & PA_CC_AI)
        cc1_present = 1u;

    USBPD->PORT_CC2 &= (uint16_t)~(uint16_t)(CC_CMP_Mask | PA_CC_AI);
    USBPD->PORT_CC2 |= CC_CMP_22;
    TIME_DelayUs(2u);
    if(USBPD->PORT_CC2 & PA_CC_AI)
        cc2_present = 1u;

    s_last_detect_cc1 = cc1_present;
    s_last_detect_cc2 = cc2_present;

    if((USBPD->PORT_CC1 & CC_PD) == 0u)
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

/* Normal SOP sender transaction: synchronous, microsecond-scale, DemoBoard-
 * proven.  USBPD IRQ is masked across TX -> RX turnaround -> GoodCRC poll;
 * retries keep the same Message ID. */
uint8_t PD_Port_TransactSOP(const uint8_t *buffer,
                            uint8_t length,
                            uint8_t max_attempts)
{
    uint8_t attempt;
    uint8_t msgid;

    if((buffer == 0) ||
       (length < 2u) ||
       (length > PD_PORT_MAX_FRAME_BYTES) ||
       (max_attempts == 0u))
    {
        return 0u;
    }

    /* Do not cut through an interrupt-driven automatic GoodCRC. */
    {
        uint32_t wait_start = TIME_Micros();

        while(s_auto_ack_inflight != 0u)
        {
            if((uint32_t)(TIME_Micros() - wait_start) >= 1000u)
                return 0u;
        }
    }

    if(s_phy_state != PD_PHY_IDLE)
        return 0u;

    memcpy(s_tx_buf, buffer, length);
    msgid = (uint8_t)(s_tx_buf[1] & 0x0Eu);

    for(attempt = 0u; attempt < max_attempts; attempt++)
    {
        uint8_t cnt = 250u;
        uint32_t tx_start;

        if(USBPD->STATUS & IF_RX_RESET)
        {
            USBPD->STATUS |= IF_RX_RESET;
            pd_port_latch_hard_reset();
            s_phy_state = PD_PHY_IDLE;
            s_rx_armed = 0u;
            NVIC_DisableIRQ(USBPD_IRQn);
            return 0u;
        }

        NVIC_DisableIRQ(USBPD_IRQn);

        s_phy_state = PD_PHY_TX;
        s_rx_armed = 0u;
        s_tx_result = PD_PORT_TX_RESULT_BUSY;

        USBPD->CONTROL &= (uint8_t)~(uint8_t)(PD_TX_EN | BMC_START);
        USBPD->STATUS |= IF_TX_END | IF_RX_ACT;

        s_tx_start_status = (uint8_t)USBPD->STATUS;
        s_tx_start_control = USBPD->CONTROL;
        s_tx_start_sop = UPD_SOP0;
        s_tx_start_len = length;
        s_tx_start_cc1 = USBPD->PORT_CC1;
        s_tx_start_cc2 = USBPD->PORT_CC2;

        pd_port_begin_tx(s_tx_buf, length, UPD_SOP0);

        tx_start = TIME_Micros();
        while((USBPD->STATUS & IF_TX_END) == 0u)
        {
            if(USBPD->STATUS & IF_RX_RESET)
            {
                USBPD->STATUS |= IF_RX_RESET;
                pd_port_latch_hard_reset();

                pd_port_release_cc();
                USBPD->CONTROL &= (uint8_t)~(uint8_t)PD_TX_EN;

                s_phy_state = PD_PHY_IDLE;
                s_tx_result = PD_PORT_TX_RESULT_ERR;
                s_rx_armed = 0u;
                return 0u;
            }

            if((uint32_t)(TIME_Micros() - tx_start) >= 2000u)
            {
                s_tx_end_timeouts++;
                pd_port_release_cc();
                USBPD->CONTROL &= (uint8_t)~(uint8_t)PD_TX_EN;
                s_phy_state = PD_PHY_IDLE;
                s_tx_result = PD_PORT_TX_RESULT_ERR;
                break;
            }
        }

        if((USBPD->STATUS & IF_TX_END) == 0u)
            continue;

        USBPD->STATUS |= IF_TX_END;

        /* Immediate TX -> RX turnaround.  GoodCRC lands directly in the same
         * policy buffer used by normal RX. */
        pd_port_enter_rx();
        s_phy_state = PD_PHY_WAIT_GOODCRC;

        while(--cnt)
        {
            if(USBPD->STATUS & IF_RX_RESET)
            {
                USBPD->STATUS |= IF_RX_RESET;
                pd_port_latch_hard_reset();

                s_phy_state = PD_PHY_IDLE;
                s_tx_result = PD_PORT_TX_RESULT_ERR;
                s_rx_armed = 0u;
                return 0u;
            }

            if((USBPD->STATUS & IF_RX_ACT) != 0u)
            {
                uint8_t rx_status = (uint8_t)USBPD->STATUS;
                uint8_t type =
                    (s_rx_buffer != 0) ?
                    (uint8_t)(s_rx_buffer[0] & 0x1Fu) : 0u;

                uint8_t goodcrc =
                    ((s_rx_buffer != 0) &&
                     ((rx_status & MASK_PD_STAT) == PD_RX_SOP0) &&
                     (type == PD_PORT_GOODCRC_TYPE) &&
                     ((s_rx_buffer[1] & 0x70u) == 0u) &&
                     ((s_rx_buffer[1] & 0x0Eu) == msgid)) ? 1u : 0u;

                USBPD->STATUS |= IF_RX_ACT;

                if(goodcrc != 0u)
                {
                    s_phy_state = PD_PHY_IDLE;
                    s_tx_result = PD_PORT_TX_RESULT_OK;
                    s_rx_armed = 1u;

                    USBPD->CONFIG |= IE_RX_ACT | IE_RX_RESET | PD_DMA_EN;
                    NVIC_EnableIRQ(USBPD_IRQn);
                    return 1u;
                }
            }

            TIME_DelayUs(3u);
        }

        s_goodcrc_timeouts++;
        s_phy_state = PD_PHY_IDLE;
        s_tx_result = PD_PORT_TX_RESULT_ERR;
    }

    /* The last attempt already left the PHY in RX. */
    s_phy_state = PD_PHY_IDLE;
    s_rx_armed = 1u;
    USBPD->CONFIG |= IE_RX_ACT | IE_RX_RESET | PD_DMA_EN;
    NVIC_EnableIRQ(USBPD_IRQn);
    return 0u;
}

/* Compatibility API: the policy layer uses PD_Port_TransactSOP(); these entry
 * points only exist to keep the current pd_port.h surface compiling. */
uint8_t PD_Port_StartTx(const uint8_t *buffer,
                        uint8_t length,
                        uint8_t expect_goodcrc)
{
    if((s_auto_ack_inflight != 0u) || (s_phy_state != PD_PHY_IDLE))
        return 0u;

    if((length != 0u) && (buffer == 0))
        return 0u;

    if(length > PD_PORT_MAX_FRAME_BYTES)
        return 0u;

    if(expect_goodcrc != 0u)
    {
        s_tx_result = PD_PORT_TX_RESULT_BUSY;

        if(PD_Port_TransactSOP(buffer, length, PD_PORT_TX_MAX_ATTEMPTS))
            s_tx_result = PD_PORT_TX_RESULT_OK;
        else
            s_tx_result = PD_PORT_TX_RESULT_ERR;

        return 1u;
    }

    NVIC_DisableIRQ(USBPD_IRQn);

    USBPD->CONTROL &= (uint8_t)~(uint8_t)(PD_TX_EN | BMC_START);
    USBPD->STATUS |= IF_TX_END;

    s_phy_state = PD_PHY_TX;
    s_tx_result = PD_PORT_TX_RESULT_BUSY;

    pd_port_begin_tx(buffer, length, UPD_SOP0);

    {
        uint32_t start = TIME_Micros();

        while((USBPD->STATUS & IF_TX_END) == 0u)
        {
            if((uint32_t)(TIME_Micros() - start) >= 2000u)
            {
                s_tx_end_timeouts++;
                s_tx_result = PD_PORT_TX_RESULT_ERR;
                s_phy_state = PD_PHY_IDLE;
                pd_port_release_cc();
                USBPD->CONTROL &= (uint8_t)~(uint8_t)PD_TX_EN;
                NVIC_EnableIRQ(USBPD_IRQn);
                return 1u;
            }
        }
    }

    USBPD->STATUS |= IF_TX_END;
    pd_port_enter_rx();

    s_phy_state = PD_PHY_IDLE;
    s_tx_result = PD_PORT_TX_RESULT_OK;

    USBPD->CONFIG |= IE_RX_ACT | IE_RX_RESET | PD_DMA_EN;
    NVIC_EnableIRQ(USBPD_IRQn);

    return 1u;
}

uint8_t PD_Port_GetTxResult(void)
{
    return s_tx_result;
}

void PD_Port_ClearTxResult(void)
{
    if(s_tx_result != PD_PORT_TX_RESULT_BUSY)
        s_tx_result = PD_PORT_TX_RESULT_NONE;
}

void PD_Port_Service(void)
{
    /* SOP transactions and auto-GoodCRC complete inside the PHY paths; no
     * foreground timeout state machine lives here. */
}

uint8_t PD_Port_TxBusy(void)
{
    return ((s_auto_ack_inflight != 0u) ||
            (s_phy_state != PD_PHY_IDLE)) ? 1u : 0u;
}

void PD_Port_AbortTx(void)
{
    pd_port_release_cc();
    USBPD->CONTROL &= (uint8_t)~(uint8_t)PD_TX_EN;

    s_auto_ack_inflight = 0u;
    s_phy_state = PD_PHY_IDLE;
    s_tx_result = PD_PORT_TX_RESULT_NONE;
    s_rx_armed = 0u;
}

void PD_Port_SendHardReset(void)
{
    uint32_t start;

    if((s_auto_ack_inflight != 0u) || (s_phy_state != PD_PHY_IDLE))
        return;

    NVIC_DisableIRQ(USBPD_IRQn);

    USBPD->CONTROL &= (uint8_t)~(uint8_t)(PD_TX_EN | BMC_START);
    USBPD->STATUS |= IF_TX_END;

    s_phy_state = PD_PHY_TX;
    pd_port_begin_tx(0, 0u, UPD_HARD_RESET);

    start = TIME_Micros();
    while((USBPD->STATUS & IF_TX_END) == 0u)
    {
        if((uint32_t)(TIME_Micros() - start) >= 2000u)
        {
            s_tx_end_timeouts++;
            break;
        }
    }

    USBPD->STATUS |= IF_TX_END;
    pd_port_release_cc();
    USBPD->CONTROL &= (uint8_t)~(uint8_t)PD_TX_EN;

    s_phy_state = PD_PHY_IDLE;
    s_rx_armed = 0u;
}

uint8_t PD_Port_AutoAckBusy(void)
{
    return s_auto_ack_inflight;
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
    s_message_pending = 0u;
}

uint8_t PD_Port_HardResetPending(void)
{
    return s_hard_reset_pending;
}

void PD_Port_ClearHardResetEvent(void)
{
    s_hard_reset_pending = 0u;
}

void PD_Port_GetPhyDiag(PD_Port_PhyDiag *diag)
{
    if(diag == 0)
        return;

    memset(diag, 0, sizeof(*diag));

    diag->config = USBPD->CONFIG;
    diag->control = USBPD->CONTROL;
    diag->status = USBPD->STATUS;
    diag->port_cc1 = USBPD->PORT_CC1;
    diag->port_cc2 = USBPD->PORT_CC2;
    diag->bmc_byte_count = USBPD->BMC_BYTE_CNT;
    diag->selected_cc = (USBPD->CONFIG & CC_SEL) ? 2u : 1u;
    diag->last_detect_cc1 = s_last_detect_cc1;
    diag->last_detect_cc2 = s_last_detect_cc2;

    diag->tx_end_timeouts = s_tx_end_timeouts;
    diag->goodcrc_timeouts = s_goodcrc_timeouts;
    diag->ack_tx_timeouts = s_ack_tx_timeouts;

    diag->tx_start_status = s_tx_start_status;
    diag->tx_start_control = s_tx_start_control;
    diag->tx_start_sop = s_tx_start_sop;
    diag->tx_start_len = s_tx_start_len;
    diag->tx_start_cc1 = s_tx_start_cc1;
    diag->tx_start_cc2 = s_tx_start_cc2;

    diag->phy_state = s_phy_state;
    diag->tx_dma_intact = s_tx_dma_intact;

    diag->hr_pd_stat = s_hr_pd_stat;
    diag->hr_byte_count = s_hr_byte_count;
}

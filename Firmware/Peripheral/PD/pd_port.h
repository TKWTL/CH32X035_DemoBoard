#ifndef PD_PORT_H_
#define PD_PORT_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * CH32X035 USB-PD PHY/CC port interface.
 *
 * Contract of this layer:
 *   - protocol/policy code must not access USBPD/RCC/GPIO/NVIC registers;
 *   - the MCU-specific implementation owns the timing-critical USBPD vector,
 *     IRQ logic and GoodCRC response directly;
 *   - RX: the USBPD dedicated DMA fills the buffer passed to PD_Port_Init()
 *     directly; the packet is published to the policy layer only after its
 *     automatic GoodCRC has physically finished (no intermediate copy);
 *   - TX: Source-originated RX/auto-GoodCRC remains IRQ-driven; normal
 *     Sink-originated SOP traffic uses a short atomic TX -> RX -> GoodCRC
 *     transaction matching the field-verified DemoBoard/C140 path.
 */

typedef enum
{
    PD_PORT_CC_NONE = 0,
    PD_PORT_CC1     = 1,
    PD_PORT_CC2     = 2
} PD_Port_CC;

typedef enum
{
    PD_PORT_ROLE_SINK   = 0,
    PD_PORT_ROLE_SOURCE = 1
} PD_Port_PowerRole;

/* Hardware/PHY lifecycle. */
void PD_Port_Init(uint8_t *rx_buffer, uint16_t rx_buffer_size);
void PD_Port_SetPowerRole(PD_Port_PowerRole role);
void PD_Port_RxStart(void);

/* Type-C CC attach detection and active-CC selection. */
PD_Port_CC PD_Port_DetectAttach(void);
void PD_Port_SelectCC(PD_Port_CC cc);

typedef struct
{
    uint16_t config;
    uint8_t control;
    uint8_t status;
    uint16_t port_cc1;
    uint16_t port_cc2;
    uint8_t bmc_byte_count;
    uint8_t selected_cc;
    uint8_t last_detect_cc1;
    uint8_t last_detect_cc2;
    /* TX frames that never raised IF_TX_END and were given up on after the
     * bounded wait; a growing count points at a truncated transmit (CC drop,
     * PHY stuck mid-TX) instead of a protocol-level failure. */
    uint16_t tx_end_timeouts;
    /* Failure-mode split.  tx_end_timeouts alone cannot tell a frame the PHY
     * never finished from a frame the partner never acknowledged; these two
     * separate those cases. */
    uint16_t goodcrc_timeouts;   /* frame sent, partner GoodCRC never came */
    uint16_t ack_tx_timeouts;    /* automatic GoodCRC response never finished */
    /* Register snapshot taken when a foreground frame was handed to the PHY.
     * Compared against the live registers at failure time it shows whether the
     * PHY was still settling, still owning the DMA pointer, or had already
     * turned the transmitter around. */
    uint8_t  tx_start_status;
    uint8_t  tx_start_control;
    uint8_t  tx_start_sop;
    uint8_t  tx_start_len;
    uint16_t tx_start_cc1;
    uint16_t tx_start_cc2;
    uint8_t  phy_state;
    uint8_t  tx_dma_intact;      /* 1: USBPD->DMA still points at our TX frame */
    /* Hardware evidence latched at the last reset event: PD status bits and BMC
     * byte count.  A real Hard Reset ordered set carries no data objects, so a
     * large byte count means the reset was actually an RX error. */
    uint8_t hr_pd_stat;
    uint8_t hr_byte_count;
} PD_Port_PhyDiag;

void PD_Port_GetPhyDiag(PD_Port_PhyDiag *diag);

/* Field-verified CH32X035/C140 normal-SOP sender transaction.
 * Only the microsecond-scale TX -> RX turnaround -> GoodCRC window is atomic;
 * ordinary RX and automatic GoodCRC remain USBPD-IRQ driven. */
uint8_t PD_Port_TransactSOP(const uint8_t *buffer,
                            uint8_t length,
                            uint8_t max_attempts);

/* ---- asynchronous SOP transmit engine ---------------------------------
 * PD_Port_StartTx() hands a frame to the USBPD dedicated DMA and returns at
 * once.  The USBPD IRQ completes it (IF_TX_END -> RX turnaround -> Source
 * GoodCRC -> result); PD_Port_Service() - called once per main-loop pass -
 * enforces the TX/GoodCRC deadlines and retries up to 3 attempts.  The result
 * is sticky until the next StartTx or an explicit clear. */
#define PD_PORT_TX_RESULT_NONE   0u
#define PD_PORT_TX_RESULT_BUSY   1u
#define PD_PORT_TX_RESULT_OK     2u
#define PD_PORT_TX_RESULT_ERR    3u

uint8_t PD_Port_StartTx(const uint8_t *buffer, uint8_t length,
                        uint8_t expect_goodcrc);
uint8_t PD_Port_GetTxResult(void);
void    PD_Port_ClearTxResult(void);
void    PD_Port_Service(void);
uint8_t PD_Port_TxBusy(void);
void    PD_Port_AbortTx(void);

/* Hard Reset ordered set: no GoodCRC transaction, fire-and-forget. */
void PD_Port_SendHardReset(void);

uint8_t PD_Port_AutoAckBusy(void);
void PD_Port_GetAutoAckStats(uint16_t *started, uint16_t *completed);

/* IRQ-to-policy event bridge. */
uint8_t PD_Port_MessagePending(void);
void PD_Port_ClearMessageEvent(void);
uint8_t PD_Port_HardResetPending(void);
void PD_Port_ClearHardResetEvent(void);

#ifdef __cplusplus
}
#endif

#endif /* PD_PORT_H_ */

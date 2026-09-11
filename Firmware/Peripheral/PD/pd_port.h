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
 *   - received bytes are DMA'd into the buffer supplied to PD_Port_Init().
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

/* Timing-critical transaction helper.  It snapshots RX metadata before
 * clearing IF_RX_ACT so a following Accept packet cannot overwrite the
 * just-arrived GoodCRC between multiple abstraction-layer calls. */
typedef struct
{
    uint8_t saw_frame;
    uint8_t byte_count;
    uint8_t message_type;
    uint8_t pd_status;
    uint8_t saw_hard_reset;
    uint8_t attempts;
    uint32_t ack_to_first_tx_us;
} PD_Port_TxDiag;

/* Exact WCH/C140 foreground SOP transaction: IRQ masked, blocking TX,
 * immediate RX turnaround, then up to 3 x 750 us GoodCRC polling windows.
 * Keeping these steps in one PHY-layer call prevents scheduler/debug/API gaps
 * from entering the sender-response timing path. */
uint8_t PD_Port_TransactSOP(const uint8_t *buffer,
                            uint8_t length,
                            uint8_t max_attempts,
                            PD_Port_TxDiag *diag);

/* Hard Reset is intentionally a dedicated API: normal SOP traffic must use
 * PD_Port_TransactSOP() so TX->RX->GoodCRC remains one atomic PHY operation. */
void PD_Port_SendHardReset(void);

uint8_t PD_Port_AutoAckBusy(void);
uint8_t PD_Port_WaitAutoAckComplete(uint32_t timeout_us);
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

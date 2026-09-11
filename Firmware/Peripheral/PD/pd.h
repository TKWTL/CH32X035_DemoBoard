#ifndef PD_H_
#define PD_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Board power policy. */
#define PD_REQUEST_MAX_FIXED_MV         20000U
#define PD_SPR_REQUEST_MAX_MA           5000U
#define PD_EPR_ENABLE                   1U
#define PD_EPR_TARGET_MV                28000U
#define PD_EPR_REQUEST_MAX_MA           5000U
#define PD_EPR_SINK_PDP_W               140U

/* Public PD service API.  Protocol state, timers, PHY access and EPR chunking
 * are internal to Peripheral/PD. */
void PD_Init(void);
void PD_Task(uint32_t now_ms);
/* Feed board VBUS measurement into the policy engine.  The current board
 * supplies this from INA226; PD owns detach debounce/contract teardown. */
void PD_SetVbusMillivolts(uint16_t mv);
uint8_t PD_IsConnected(void);
uint8_t PD_IsEPRContractActive(void);
uint8_t PD_IsPowerReady(void);
uint16_t PD_GetContractVoltageMv(void);
uint16_t PD_GetContractCurrentMa(void);

typedef enum
{
    PD_DISPLAY_PDO_SPR = 1,
    PD_DISPLAY_PDO_PPS = 2,
    PD_DISPLAY_PDO_EPR = 3,
    PD_DISPLAY_PDO_AVS = 4
} PD_DisplayPDOType;

typedef struct
{
    uint8_t index;
    uint8_t type;
    uint16_t min_mv;
    uint16_t max_mv;
    uint16_t current_ma;
    uint16_t power_w;
} PD_DisplayPDO;

/* Snapshot up to max_count display-worthy PDOs, sorted for the UI as
 * AVS > EPR Fixed > PPS > SPR Fixed, then by maximum voltage descending.
 * Voltage/current/power are decoded directly from the received Source PDO and
 * are never clamped to the Sink request policy. Detached always returns 0.
 * Battery/Variable PDOs and unsupported APDO subtypes are intentionally omitted. */
uint8_t PD_GetDisplayPDOs(PD_DisplayPDO *out, uint8_t max_count);

#ifdef __cplusplus
}
#endif

#endif /* PD_H_ */

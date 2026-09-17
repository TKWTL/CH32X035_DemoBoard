#ifndef PD_H_
#define PD_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Board power policy - two layers (2026-09-17 request-strategy upgrade):
 *   - nominal: the product contract promised by the UI/manual;
 *   - policy ceilings: what the adaptive request path in pd.c (PD_POLICY_*)
 *     may ask from a non-standard source.  Actual requests always clamp to
 *     what the Source advertises. */
#define PD_SPR_MAX_FIXED_MV             20000U   /* SPR fixed-PDO ceiling */
#define PD_EPR_NOMINAL_MV               28000U   /* nominal EPR contract voltage */
#define PD_NOMINAL_REQUEST_MA            5000U   /* nominal contract current */
#define PD_POLICY_REQUEST_MAX_MA         7000U   /* adaptive current ceiling (pd.c) */
/* Request-strategy ceiling: the highest EPR Fixed voltage this board may ask
 * for (raised to 36 V; the Desktop reference uses 32 V).  Actual requests are
 * always clamped to what the attached Source advertises. */
#define PD_POLICY_EPR_MAX_FIXED_MV      36000U   /* adaptive EPR voltage ceiling */
#define PD_NOMINAL_PDP_W                  140U   /* nominal Sink PDP */
#define PD_EPR_ENABLE                      1U

/* USB identity VDO values, kept for a future identity experiment.
 *
 * The current build deliberately does NOT answer VDM identity requests with
 * these values: it mirrors the field-verified DemoBoard behaviour (short NAK)
 * because the crafted ACK was followed by this charger withholding the EPR
 * grant, while the verified build never supplies a real identity at all.
 * Values remain here (pid.codes open-project test assignment - replace before
 * shipping) for the case that a real identity exchange turns out to be
 * required after all. */
#define PD_IDENTITY_VID                0x1209U
#define PD_IDENTITY_PID                0x0001U
#define PD_IDENTITY_BCD_DEVICE         0x0001U

/* Public PD service API.  Protocol state, timers, PHY access and EPR chunking
 * are internal to Peripheral/PD. */
void PD_Init(void);
void PD_Task(uint32_t now_ms);
/* Feed board VBUS measurement into the policy engine.  This board supplies it from INA226; PD owns
 * detach debounce/contract teardown. */
void PD_SetVbusMillivolts(uint16_t mv);
uint8_t PD_IsConnected(void);
uint8_t PD_IsEPRContractActive(void);
uint8_t PD_IsPowerReady(void);
uint16_t PD_GetContractVoltageMv(void);
uint16_t PD_GetContractCurrentMa(void);
/* Fast-poll request for the scheduler idle hook: true while PD needs main-loop
 * passes at full speed (source attached, GoodCRC in flight).  Sleeping here
 * stretches the Sink sender-response latency outside the PD timing windows. */
uint8_t PD_WantsFastPoll(void);

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

#ifndef SSD1306_U8G2_H_
#define SSD1306_U8G2_H_

#include <stdint.h>
#include "i2c_api.h"
#include "u8g2.h"
#include "fonts.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SSD1306_I2C_ADDR_PRIMARY      0x3Cu
#define SSD1306_I2C_ADDR_ALTERNATE    0x3Du
#define SSD1306_WIDTH                 128u
#define SSD1306_HEIGHT                64u
#define SSD1306_PAGE_COUNT            8u
#define SSD1306_U8G2_DEFAULT_FONT      font_menu_main_h12w6

/* u8g2 full-buffer renderer. Transport to the panel is asynchronous. */
void SSD1306_U8G2_Init(void);
u8g2_t *SSD1306_U8G2_Get(void);
void SSD1306_U8G2_SetAddress(uint8_t address_7bit);
uint8_t SSD1306_U8G2_GetAddress(void);

/* Probe is separated from normal display traffic so hot-plug policy can live
 * in a coroOS thread instead of the BSP. */
uint8_t SSD1306_U8G2_BeginProbe(uint8_t address_7bit);
uint8_t SSD1306_U8G2_ProbeComplete(void);
I2C_API_Result SSD1306_U8G2_TakeProbeResult(void);

/* Init is one asynchronous I2C command transaction. */
uint8_t SSD1306_U8G2_BeginPanelInit(void);
uint8_t SSD1306_U8G2_PanelInitComplete(void);
I2C_API_Result SSD1306_U8G2_TakePanelInitResult(void);

/* Full-buffer flush is split into eight 128-byte pages. PollFlush() advances
 * only one small state-machine step and never spins. */
uint8_t SSD1306_U8G2_BeginFlush(void);
I2C_API_Result SSD1306_U8G2_PollFlush(void);
uint8_t SSD1306_U8G2_IsFlushBusy(void);

#ifdef __cplusplus
}
#endif

#endif /* SSD1306_U8G2_H_ */

#ifndef PIOC_API_H_
#define PIOC_API_H_

#include "ch32x035.h"
#include "PIOC_SFR.h"

#define PIOC_API_SFR_ADDR       ((uint8_t *)&(PIOC->D8_DATA_REG0))
#define PIOC_API_SFR_SIZE       32u
#define PIOC_API_COMMAND        (PIOC->D8_CTRL_WR)

#define PIOC_API_OK         0u
#define PIOC_API_ERR_PARAM       2u
#define PIOC_API_ERR_TIMEOUT    0xFEu
#define PIOC_API_WAIT_TIMEOUT_US  3000UL

void PIOC_API_Init(void);
uint8_t PIOC_API_Send(const uint8_t *data, uint16_t length);

#endif /* PIOC_API_H_ */

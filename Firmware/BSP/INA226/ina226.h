#ifndef INA226_H_
#define INA226_H_

#include <stdint.h>
#include "i2c_api.h"

#define INA226_I2C_ADDR_7BIT  0x40u
#define INA226_I2C_CLOCK_HZ   400000UL
#define INA226_SHUNT_MOHM     10u

#define INA226_REG_SHUNT      0x01u
#define INA226_REG_BUS        0x02u
#define INA226_REG_MFR_ID     0xFEu
#define INA226_REG_DIE_ID     0xFFu

typedef struct
{
    uint16_t bus_raw;
    int16_t  shunt_raw;
    uint32_t bus_uV;
    int32_t  shunt_uV_x10;
    int32_t  current_mA_x100;
} INA226_Measurement;

typedef struct
{
    const char *error_name;
    uint16_t star1;
    uint16_t star2;
    uint8_t scl_high;
    uint8_t sda_high;
    uint32_t recovery_count;
} INA226_Diagnostic;

/* Bus hardware is initialized once by I2C_API_Init(). */
void INA226_Init(void);

/* One outstanding INA226 request at a time. These functions never spin. */
uint8_t INA226_BeginPing(void);
uint8_t INA226_BeginReadReg16(uint8_t reg);
uint8_t INA226_IsTransferComplete(void);
I2C_API_Result INA226_TakePingResult(void);
I2C_API_Result INA226_TakeReadReg16(uint16_t *value);

uint16_t INA226_BusRawToMv(uint16_t bus_raw);
void INA226_ConvertMeasurement(uint16_t bus_raw,
                               uint16_t shunt_u16,
                               INA226_Measurement *m);

const char *INA226_GetLastErrorName(void);
void INA226_GetDiagnostic(INA226_Diagnostic *diag);

#endif /* INA226_H_ */

#include "ina226.h"

static uint8_t s_reg;
static uint8_t s_rx[2];
static uint8_t s_pending_read;

void INA226_Init(void)
{
    s_reg = 0u;
    s_rx[0] = 0u;
    s_rx[1] = 0u;
    s_pending_read = 0u;
}

uint8_t INA226_BeginPing(void)
{
    if(s_pending_read)
        return 0u;
    return I2C_API_TryPing(I2C_API_OWNER_INA226, INA226_I2C_ADDR_7BIT);
}

uint8_t INA226_BeginReadReg16(uint8_t reg)
{
    if(s_pending_read)
        return 0u;

    s_reg = reg;
    s_rx[0] = 0u;
    s_rx[1] = 0u;
    if(!I2C_API_TryWriteRead(I2C_API_OWNER_INA226,
                             INA226_I2C_ADDR_7BIT,
                             &s_reg,
                             1u,
                             s_rx,
                             2u))
        return 0u;

    s_pending_read = 1u;
    return 1u;
}

uint8_t INA226_IsTransferComplete(void)
{
    I2C_API_Result r = I2C_API_GetResult(I2C_API_OWNER_INA226);
    return (r != I2C_API_RESULT_IDLE && r != I2C_API_RESULT_ACTIVE) ? 1u : 0u;
}

I2C_API_Result INA226_TakePingResult(void)
{
    return I2C_API_TakeResult(I2C_API_OWNER_INA226);
}

I2C_API_Result INA226_TakeReadReg16(uint16_t *value)
{
    I2C_API_Result r = I2C_API_TakeResult(I2C_API_OWNER_INA226);

    if(r == I2C_API_RESULT_ACTIVE || r == I2C_API_RESULT_IDLE)
        return r;

    if(r == I2C_API_RESULT_OK && value != 0)
        *value = ((uint16_t)s_rx[0] << 8) | s_rx[1];

    s_pending_read = 0u;
    return r;
}

uint16_t INA226_BusRawToMv(uint16_t bus_raw)
{
    uint32_t mv = ((uint32_t)bus_raw * 1250UL) / 1000UL;
    if(mv > 0xFFFFUL)
        mv = 0xFFFFUL;
    return (uint16_t)mv;
}

void INA226_ConvertMeasurement(uint16_t bus_raw,
                               uint16_t shunt_u16,
                               INA226_Measurement *m)
{
    int32_t shunt_raw;

    if(m == 0)
        return;

    shunt_raw = (int16_t)shunt_u16;
    m->bus_raw = bus_raw;
    m->shunt_raw = (int16_t)shunt_raw;
    m->bus_uV = (uint32_t)bus_raw * 1250UL;
    m->shunt_uV_x10 = shunt_raw * 25L;
#if (INA226_SHUNT_MOHM == 10u)
    m->current_mA_x100 = shunt_raw * 25L;
#else
    m->current_mA_x100 = (shunt_raw * 250L) / (int32_t)INA226_SHUNT_MOHM;
#endif
}

const char *INA226_GetLastErrorName(void)
{
    return I2C_API_GetLastErrorName();
}

void INA226_GetDiagnostic(INA226_Diagnostic *diag)
{
    I2C_API_Diagnostic d;

    if(diag == 0)
        return;

    I2C_API_GetDiagnostic(&d);
    diag->error_name = I2C_API_GetLastErrorName();
    diag->star1 = d.star1;
    diag->star2 = d.star2;
    diag->scl_high = d.scl_high;
    diag->sda_high = d.sda_high;
    diag->recovery_count = d.recovery_count;
}

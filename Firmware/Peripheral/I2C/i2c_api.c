#include "i2c_api.h"
#include "main.h"
#include "ch32x035.h"
#include "time_api.h"

#define I2C_API_DMA_TX_CHANNEL        DMA1_Channel6
#define I2C_API_DMA_RX_CHANNEL        DMA1_Channel7
#define I2C_API_ERROR_MASK            (I2C_STAR1_AF | I2C_STAR1_BERR | I2C_STAR1_ARLO | I2C_STAR1_OVR)
#define I2C_API_BUS_RECOVERY_PULSE_US 5u

typedef struct
{
    volatile I2C_API_State state;
    volatile I2C_API_Owner owner;
    volatile I2C_API_Result result;
    uint8_t address_7bit;
    const uint8_t *tx;
    uint16_t tx_len;
    uint8_t *rx;
    uint16_t rx_len;
    uint32_t last_progress_ms;
} I2C_API_Transaction;

static uint32_t s_clock_hz = I2C_API_DEFAULT_CLOCK_HZ;
static I2C_API_Transaction s_xfer;
static volatile I2C_API_Error s_last_error = I2C_API_ERR_NONE;
static volatile uint16_t s_last_star1;
static volatile uint16_t s_last_star2;
static volatile uint8_t s_recovery_requested;
static volatile uint32_t s_recovery_count;

/*
 * The watchdog measures inactivity in the current hardware state, not total
 * wall-clock transaction lifetime.  A cooperative scheduler can legitimately
 * spend tens of milliseconds in PD/UART work between I2C service passes; if
 * the peripheral made progress meanwhile, that must refresh the watchdog.
 */
static void set_state(I2C_API_State state)
{
    s_xfer.state = state;
    s_xfer.last_progress_ms = TIME_Millis();
}

static void gpio_input_release(GPIO_TypeDef *port, uint32_t pin)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.GPIO_Pin = pin;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(port, &gpio);
}

static void gpio_drive_low(GPIO_TypeDef *port, uint32_t pin)
{
    GPIO_InitTypeDef gpio = {0};
    GPIO_ResetBits(port, pin);
    gpio.GPIO_Pin = pin;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(port, &gpio);
    GPIO_ResetBits(port, pin);
}

static void i2c_gpio_af(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.GPIO_Pin = SCL_Pin;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SCL_GPIO_Port, &gpio);

    gpio.GPIO_Pin = SDA_Pin;
    GPIO_Init(SDA_GPIO_Port, &gpio);
}

static void i2c_hw_init(void)
{
    I2C_InitTypeDef i2c = {0};

    RCC_APB2PeriphClockCmd(SCL_GPIO_CLK | SDA_GPIO_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    i2c_gpio_af();

    I2C_DeInit(I2C1);
    i2c.I2C_ClockSpeed = s_clock_hz;
    i2c.I2C_Mode = I2C_Mode_I2C;
    i2c.I2C_DutyCycle = I2C_DutyCycle_2;
    i2c.I2C_OwnAddress1 = 0x00;
    i2c.I2C_Ack = I2C_Ack_Enable;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &i2c);
    I2C_Cmd(I2C1, ENABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Current);
    I2C_DMACmd(I2C1, DISABLE);
    I2C_DMALastTransferCmd(I2C1, DISABLE);

    DMA_Cmd(I2C_API_DMA_TX_CHANNEL, DISABLE);
    DMA_Cmd(I2C_API_DMA_RX_CHANNEL, DISABLE);
    DMA_ClearFlag(DMA1_FLAG_GL6 | DMA1_FLAG_GL7);
}

static void snapshot(I2C_API_Error error)
{
    s_last_error = error;
    s_last_star1 = I2C1->STAR1;
    s_last_star2 = I2C1->STAR2;
}

static void clear_i2c_error_flags(void)
{
    if((I2C1->STAR1 & I2C_STAR1_AF) != 0u)   I2C_ClearFlag(I2C1, I2C_FLAG_AF);
    if((I2C1->STAR1 & I2C_STAR1_BERR) != 0u) I2C_ClearFlag(I2C1, I2C_FLAG_BERR);
    if((I2C1->STAR1 & I2C_STAR1_ARLO) != 0u) I2C_ClearFlag(I2C1, I2C_FLAG_ARLO);
    if((I2C1->STAR1 & I2C_STAR1_OVR) != 0u)  I2C_ClearFlag(I2C1, I2C_FLAG_OVR);
}

static void stop_dma(void)
{
    I2C_DMACmd(I2C1, DISABLE);
    I2C_DMALastTransferCmd(I2C1, DISABLE);
    DMA_Cmd(I2C_API_DMA_TX_CHANNEL, DISABLE);
    DMA_Cmd(I2C_API_DMA_RX_CHANNEL, DISABLE);
    DMA_ClearFlag(DMA1_FLAG_GL6 | DMA1_FLAG_GL7);
}

static void finish(I2C_API_Result result, I2C_API_Error error, uint8_t request_recovery)
{
    /* Capture the failing hardware state before STOP/DMA cleanup changes the
     * status registers.  In particular, reading STAR2 can clear ADDR after a
     * STAR1 read, so snapshot first when diagnosing a timeout. */
    if(error != I2C_API_ERR_NONE)
        snapshot(error);
    else
    {
        s_last_error = I2C_API_ERR_NONE;
        s_last_star1 = I2C1->STAR1;
        s_last_star2 = I2C1->STAR2;
    }

    stop_dma();
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Current);

    if((I2C1->STAR2 & I2C_STAR2_BUSY) != 0u)
        I2C_GenerateSTOP(I2C1, ENABLE);

    clear_i2c_error_flags();
    s_xfer.state = I2C_API_STATE_IDLE;
    s_xfer.result = result;
    if(request_recovery)
        s_recovery_requested = 1u;
}

static uint8_t begin(I2C_API_Owner owner,
                     uint8_t address_7bit,
                     const uint8_t *tx,
                     uint16_t tx_len,
                     uint8_t *rx,
                     uint16_t rx_len)
{
    if(owner == I2C_API_OWNER_NONE || address_7bit > 0x7Fu)
        return 0u;
    if(s_xfer.owner != I2C_API_OWNER_NONE || s_xfer.result != I2C_API_RESULT_IDLE)
        return 0u;
    if(s_recovery_requested)
        return 0u;
    if(tx_len != 0u && tx == 0)
        return 0u;
    if(rx_len != 0u && rx == 0)
        return 0u;

    s_xfer.owner = owner;
    s_xfer.result = I2C_API_RESULT_ACTIVE;
    s_xfer.address_7bit = address_7bit;
    s_xfer.tx = tx;
    s_xfer.tx_len = tx_len;
    s_xfer.rx = rx;
    s_xfer.rx_len = rx_len;
    set_state(I2C_API_STATE_WAIT_BUS_IDLE);
    s_last_error = I2C_API_ERR_NONE;
    return 1u;
}

static void start_tx_dma(void)
{
    DMA_InitTypeDef dma = {0};

    DMA_Cmd(I2C_API_DMA_TX_CHANNEL, DISABLE);
    DMA_DeInit(I2C_API_DMA_TX_CHANNEL);
    DMA_ClearFlag(DMA1_FLAG_GL6);

    dma.DMA_PeripheralBaseAddr = (uint32_t)&I2C1->DATAR;
    dma.DMA_MemoryBaseAddr = (uint32_t)s_xfer.tx;
    dma.DMA_DIR = DMA_DIR_PeripheralDST;
    dma.DMA_BufferSize = s_xfer.tx_len;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Normal;
    dma.DMA_Priority = DMA_Priority_High;
    dma.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(I2C_API_DMA_TX_CHANNEL, &dma);

    I2C_DMALastTransferCmd(I2C1, DISABLE);
    I2C_DMACmd(I2C1, ENABLE);
    DMA_Cmd(I2C_API_DMA_TX_CHANNEL, ENABLE);
    set_state(I2C_API_STATE_WAIT_TX_DMA);
}

static void start_rx_dma(void)
{
    DMA_InitTypeDef dma = {0};

    DMA_Cmd(I2C_API_DMA_RX_CHANNEL, DISABLE);
    DMA_DeInit(I2C_API_DMA_RX_CHANNEL);
    DMA_ClearFlag(DMA1_FLAG_GL7);

    dma.DMA_PeripheralBaseAddr = (uint32_t)&I2C1->DATAR;
    dma.DMA_MemoryBaseAddr = (uint32_t)s_xfer.rx;
    dma.DMA_DIR = DMA_DIR_PeripheralSRC;
    dma.DMA_BufferSize = s_xfer.rx_len;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Normal;
    dma.DMA_Priority = DMA_Priority_High;
    dma.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(I2C_API_DMA_RX_CHANNEL, &dma);

    /* LAST makes the peripheral NACK the final DMA byte. */
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    I2C_DMALastTransferCmd(I2C1, ENABLE);
    I2C_DMACmd(I2C1, ENABLE);
    DMA_Cmd(I2C_API_DMA_RX_CHANNEL, ENABLE);
    set_state(I2C_API_STATE_WAIT_RX_DMA);
}

/*
 * ADDR is cleared by the mandatory STAR1 -> STAR2 read sequence. Do this
 * explicitly instead of I2C_CheckEvent(): the generic event helper reads both
 * status registers internally, which hides the exact clear point from the DMA
 * receive state machine.
 */
static void clear_addr_flag(void)
{
    volatile uint16_t star1;
    volatile uint16_t star2;

    star1 = I2C1->STAR1;
    star2 = I2C1->STAR2;
    (void)star1;
    (void)star2;
}

static uint8_t handle_error_flags(void)
{
    uint16_t star1 = I2C1->STAR1;

    if((star1 & I2C_STAR1_AF) != 0u)
    {
        finish(I2C_API_RESULT_NACK, I2C_API_ERR_NACK, 0u);
        return 1u;
    }
    if((star1 & I2C_STAR1_BERR) != 0u)
    {
        finish(I2C_API_RESULT_ERROR, I2C_API_ERR_BERR, 1u);
        return 1u;
    }
    if((star1 & I2C_STAR1_ARLO) != 0u)
    {
        finish(I2C_API_RESULT_ERROR, I2C_API_ERR_ARLO, 1u);
        return 1u;
    }
    if((star1 & I2C_STAR1_OVR) != 0u)
    {
        finish(I2C_API_RESULT_ERROR, I2C_API_ERR_OVR, 1u);
        return 1u;
    }
    return 0u;
}


static I2C_API_Error timeout_error_for_state(I2C_API_State state)
{
    switch(state)
    {
        case I2C_API_STATE_WAIT_BUS_IDLE: return I2C_API_ERR_BUS_BUSY;
        case I2C_API_STATE_WAIT_START_TX: return I2C_API_ERR_START_TX;
        case I2C_API_STATE_WAIT_ADDR_TX:  return I2C_API_ERR_ADDR_TX;
        case I2C_API_STATE_WAIT_TX_DMA:   return I2C_API_ERR_TX_DMA;
        case I2C_API_STATE_WAIT_TX_BTF:   return I2C_API_ERR_TX_BTF;
        case I2C_API_STATE_WAIT_START_RX: return I2C_API_ERR_START_RX;
        case I2C_API_STATE_WAIT_ADDR_RX:  return I2C_API_ERR_ADDR_RX;
        case I2C_API_STATE_WAIT_RX_DMA:   return I2C_API_ERR_RX_DMA;
        case I2C_API_STATE_WAIT_RX_SINGLE:return I2C_API_ERR_RX;
        default:                           return I2C_API_ERR_WATCHDOG;
    }
}

static void bus_recover(void)
{
    uint8_t i;

    stop_dma();
    I2C_Cmd(I2C1, DISABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
    TIME_DelayUs(2u);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);

    /* Emulate open-drain recovery: output-low means asserted, floating input
     * means released to the board pull-up. Never actively drive SDA/SCL high. */
    gpio_input_release(SCL_GPIO_Port, SCL_Pin);
    gpio_input_release(SDA_GPIO_Port, SDA_Pin);
    TIME_DelayUs(I2C_API_BUS_RECOVERY_PULSE_US);

    if(GPIO_ReadInputDataBit(SDA_GPIO_Port, SDA_Pin) == Bit_RESET)
    {
        for(i = 0u; i < 9u; ++i)
        {
            gpio_drive_low(SCL_GPIO_Port, SCL_Pin);
            TIME_DelayUs(I2C_API_BUS_RECOVERY_PULSE_US);
            gpio_input_release(SCL_GPIO_Port, SCL_Pin);
            TIME_DelayUs(I2C_API_BUS_RECOVERY_PULSE_US);
            if(GPIO_ReadInputDataBit(SDA_GPIO_Port, SDA_Pin) != Bit_RESET)
                break;
        }
    }

    /* STOP-like release: SDA low while SCL released, then release SDA. */
    gpio_drive_low(SDA_GPIO_Port, SDA_Pin);
    TIME_DelayUs(I2C_API_BUS_RECOVERY_PULSE_US);
    gpio_input_release(SCL_GPIO_Port, SCL_Pin);
    TIME_DelayUs(I2C_API_BUS_RECOVERY_PULSE_US);
    gpio_input_release(SDA_GPIO_Port, SDA_Pin);
    TIME_DelayUs(I2C_API_BUS_RECOVERY_PULSE_US);

    i2c_hw_init();
    s_recovery_count++;
    s_recovery_requested = 0u;
}

void I2C_API_Init(uint32_t clock_hz)
{
    if(clock_hz == 0u)
        clock_hz = I2C_API_DEFAULT_CLOCK_HZ;
    s_clock_hz = clock_hz;

    s_xfer.state = I2C_API_STATE_IDLE;
    s_xfer.owner = I2C_API_OWNER_NONE;
    s_xfer.result = I2C_API_RESULT_IDLE;
    s_recovery_requested = 0u;
    s_recovery_count = 0u;
    s_last_error = I2C_API_ERR_NONE;
    i2c_hw_init();
    s_last_star1 = I2C1->STAR1;
    s_last_star2 = I2C1->STAR2;
}

uint32_t I2C_API_GetClockHz(void)
{
    return s_clock_hz;
}

uint8_t I2C_API_TryPing(I2C_API_Owner owner, uint8_t address_7bit)
{
    return begin(owner, address_7bit, 0, 0u, 0, 0u);
}

uint8_t I2C_API_TryWrite(I2C_API_Owner owner,
                         uint8_t address_7bit,
                         const uint8_t *tx,
                         uint16_t tx_len)
{
    if(tx_len == 0u)
        return 0u;
    return begin(owner, address_7bit, tx, tx_len, 0, 0u);
}

uint8_t I2C_API_TryWriteRead(I2C_API_Owner owner,
                             uint8_t address_7bit,
                             const uint8_t *tx,
                             uint16_t tx_len,
                             uint8_t *rx,
                             uint16_t rx_len)
{
    if(tx_len == 0u || rx_len == 0u)
        return 0u;
    return begin(owner, address_7bit, tx, tx_len, rx, rx_len);
}

void I2C_API_Service(void)
{
    if(s_xfer.result != I2C_API_RESULT_ACTIVE)
        return;

    if(handle_error_flags())
        return;

    switch(s_xfer.state)
    {
        case I2C_API_STATE_WAIT_BUS_IDLE:
            if(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == RESET)
            {
                I2C_AcknowledgeConfig(I2C1, ENABLE);
                I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Current);
                I2C_GenerateSTART(I2C1, ENABLE);
                set_state(I2C_API_STATE_WAIT_START_TX);
            }
            break;

        case I2C_API_STATE_WAIT_START_TX:
            /* SB is the authoritative indication that START completed. Field
             * diagnostics have shown SB=1 while the composite EV5 STAR2 bits
             * still read 0, which otherwise stalls this async FSM. */
            if((I2C1->STAR1 & I2C_STAR1_SB) != 0u)
            {
                I2C_Send7bitAddress(I2C1,
                                    (uint8_t)(s_xfer.address_7bit << 1),
                                    I2C_Direction_Transmitter);
                set_state(I2C_API_STATE_WAIT_ADDR_TX);
            }
            break;

        case I2C_API_STATE_WAIT_ADDR_TX:
            if((I2C1->STAR1 & I2C_STAR1_ADDR) != 0u)
            {
                clear_addr_flag();

                if(s_xfer.tx_len != 0u)
                    start_tx_dma();
                else if(s_xfer.rx_len != 0u)
                {
                    I2C_GenerateSTART(I2C1, ENABLE);
                    set_state(I2C_API_STATE_WAIT_START_RX);
                }
                else
                    finish(I2C_API_RESULT_OK, I2C_API_ERR_NONE, 0u);
            }
            break;

        case I2C_API_STATE_WAIT_TX_DMA:
            if(DMA_GetFlagStatus(DMA1_FLAG_TE6) != RESET)
            {
                finish(I2C_API_RESULT_ERROR, I2C_API_ERR_TX_DMA, 1u);
            }
            else if(DMA_GetFlagStatus(DMA1_FLAG_TC6) != RESET)
            {
                DMA_Cmd(I2C_API_DMA_TX_CHANNEL, DISABLE);
                DMA_ClearFlag(DMA1_FLAG_GL6);
                I2C_DMACmd(I2C1, DISABLE);
                set_state(I2C_API_STATE_WAIT_TX_BTF);
            }
            break;

        case I2C_API_STATE_WAIT_TX_BTF:
            if((I2C1->STAR1 & I2C_STAR1_BTF) != 0u)
            {
                if(s_xfer.rx_len != 0u)
                {
                    I2C_GenerateSTART(I2C1, ENABLE);
                    set_state(I2C_API_STATE_WAIT_START_RX);
                }
                else
                    finish(I2C_API_RESULT_OK, I2C_API_ERR_NONE, 0u);
            }
            break;

        case I2C_API_STATE_WAIT_START_RX:
            if((I2C1->STAR1 & I2C_STAR1_SB) != 0u)
            {
                /* For a one-byte receive ACK must already be low when ADDR is
                 * cleared. For DMA receives keep ACK enabled and use LAST. */
                if(s_xfer.rx_len == 1u)
                    I2C_AcknowledgeConfig(I2C1, DISABLE);
                else
                    I2C_AcknowledgeConfig(I2C1, ENABLE);

                I2C_Send7bitAddress(I2C1,
                                    (uint8_t)(s_xfer.address_7bit << 1),
                                    I2C_Direction_Receiver);
                set_state(I2C_API_STATE_WAIT_ADDR_RX);
            }
            break;

        case I2C_API_STATE_WAIT_ADDR_RX:
            if((I2C1->STAR1 & I2C_STAR1_ADDR) != 0u)
            {
                if(s_xfer.rx_len == 1u)
                {
                    /* Single-byte sequence: ACK=0 -> clear ADDR -> STOP. */
                    clear_addr_flag();
                    I2C_GenerateSTOP(I2C1, ENABLE);
                    set_state(I2C_API_STATE_WAIT_RX_SINGLE);
                }
                else
                {
                    /* Configure DMA + LAST before clearing ADDR so the first
                     * byte cannot outrun DMA setup at 400 kHz. */
                    start_rx_dma();
                    clear_addr_flag();
                }
            }
            break;

        case I2C_API_STATE_WAIT_RX_DMA:
            if(DMA_GetFlagStatus(DMA1_FLAG_TE7) != RESET)
            {
                finish(I2C_API_RESULT_ERROR, I2C_API_ERR_RX_DMA, 1u);
            }
            else if(DMA_GetFlagStatus(DMA1_FLAG_TC7) != RESET)
            {
                DMA_Cmd(I2C_API_DMA_RX_CHANNEL, DISABLE);
                DMA_ClearFlag(DMA1_FLAG_GL7);
                I2C_DMACmd(I2C1, DISABLE);
                I2C_DMALastTransferCmd(I2C1, DISABLE);
                I2C_GenerateSTOP(I2C1, ENABLE);
                I2C_AcknowledgeConfig(I2C1, ENABLE);
                finish(I2C_API_RESULT_OK, I2C_API_ERR_NONE, 0u);
            }
            break;

        case I2C_API_STATE_WAIT_RX_SINGLE:
            if(I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) != RESET)
            {
                s_xfer.rx[0] = I2C_ReceiveData(I2C1);
                I2C_AcknowledgeConfig(I2C1, ENABLE);
                finish(I2C_API_RESULT_OK, I2C_API_ERR_NONE, 0u);
            }
            break;

        default:
            finish(I2C_API_RESULT_ERROR, I2C_API_ERR_WATCHDOG, 1u);
            break;
    }
}

void I2C_API_WatchdogService(uint32_t now_ms)
{
    if(s_xfer.result == I2C_API_RESULT_ACTIVE &&
       (uint32_t)(now_ms - s_xfer.last_progress_ms) > I2C_API_WATCHDOG_TIMEOUT_MS)
    {
        finish(I2C_API_RESULT_TIMEOUT, timeout_error_for_state(s_xfer.state), 1u);
    }

    if(s_recovery_requested && s_xfer.result != I2C_API_RESULT_ACTIVE)
        bus_recover();
}

uint8_t I2C_API_IsBusy(void)
{
    return (s_xfer.owner != I2C_API_OWNER_NONE) ? 1u : 0u;
}

I2C_API_Owner I2C_API_GetOwner(void)
{
    return s_xfer.owner;
}

I2C_API_Result I2C_API_GetResult(I2C_API_Owner owner)
{
    if(owner == I2C_API_OWNER_NONE || s_xfer.owner != owner)
        return I2C_API_RESULT_IDLE;
    return s_xfer.result;
}

I2C_API_Result I2C_API_TakeResult(I2C_API_Owner owner)
{
    I2C_API_Result result;

    if(owner == I2C_API_OWNER_NONE || s_xfer.owner != owner)
        return I2C_API_RESULT_IDLE;
    if(s_xfer.result == I2C_API_RESULT_ACTIVE)
        return I2C_API_RESULT_ACTIVE;

    result = s_xfer.result;
    s_xfer.owner = I2C_API_OWNER_NONE;
    s_xfer.result = I2C_API_RESULT_IDLE;
    s_xfer.state = I2C_API_STATE_IDLE;
    s_xfer.tx = 0;
    s_xfer.rx = 0;
    s_xfer.tx_len = 0u;
    s_xfer.rx_len = 0u;
    return result;
}

uint32_t I2C_API_GetRecoveryCount(void)
{
    return s_recovery_count;
}

I2C_API_Error I2C_API_GetLastError(void)
{
    return s_last_error;
}

const char *I2C_API_GetLastErrorName(void)
{
    switch(s_last_error)
    {
        case I2C_API_ERR_NONE:     return "none";
        case I2C_API_ERR_BUS_BUSY: return "bus busy";
        case I2C_API_ERR_START_TX: return "start TX";
        case I2C_API_ERR_ADDR_TX:  return "address TX";
        case I2C_API_ERR_TX_DMA:   return "TX DMA";
        case I2C_API_ERR_TX_BTF:   return "TX BTF";
        case I2C_API_ERR_START_RX: return "start RX";
        case I2C_API_ERR_ADDR_RX:  return "address RX";
        case I2C_API_ERR_RX_DMA:   return "RX DMA";
        case I2C_API_ERR_RX:       return "RX";
        case I2C_API_ERR_NACK:     return "NACK";
        case I2C_API_ERR_BERR:     return "bus error";
        case I2C_API_ERR_ARLO:     return "arbitration lost";
        case I2C_API_ERR_OVR:      return "overrun";
        case I2C_API_ERR_WATCHDOG: return "watchdog timeout";
        default:                   return "unknown";
    }
}

void I2C_API_GetDiagnostic(I2C_API_Diagnostic *diag)
{
    if(diag == 0)
        return;

    diag->error = s_last_error;
    diag->state = s_xfer.state;
    diag->owner = s_xfer.owner;
    diag->result = s_xfer.result;
    diag->star1 = s_last_star1;
    diag->star2 = s_last_star2;
    diag->dma_tx_remaining = DMA_GetCurrDataCounter(I2C_API_DMA_TX_CHANNEL);
    diag->dma_rx_remaining = DMA_GetCurrDataCounter(I2C_API_DMA_RX_CHANNEL);
    diag->scl_high = GPIO_ReadInputDataBit(SCL_GPIO_Port, SCL_Pin) ? 1u : 0u;
    diag->sda_high = GPIO_ReadInputDataBit(SDA_GPIO_Port, SDA_Pin) ? 1u : 0u;
    diag->transaction_age_ms = (s_xfer.result == I2C_API_RESULT_ACTIVE) ?
                               (uint32_t)(TIME_Millis() - s_xfer.last_progress_ms) : 0u;
    diag->recovery_count = s_recovery_count;
}

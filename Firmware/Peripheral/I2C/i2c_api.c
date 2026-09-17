#include "i2c_api.h"
#include "main.h"
#include "ch32x035.h"
#include "debug.h"
#include "time_api.h"

#define I2C_API_DMA_TX_CHANNEL        DMA1_Channel6
#define I2C_API_DMA_RX_CHANNEL        DMA1_Channel7
#define I2C_API_ERROR_MASK            (I2C_STAR1_AF | I2C_STAR1_BERR | I2C_STAR1_ARLO | I2C_STAR1_OVR)
#define I2C_API_BUS_RECOVERY_PULSE_US 5u
#define I2C_API_BUS_FREE_SPIN_US      250u

/* 事件中断风暴熔断值：同一状态连续被 EV 中断打断超过该次数就强制中止。
 * （正常一条事务的状态切换都伴随 set_state()，绝不会连续同状态打断。） */
#define I2C_API_EV_STORM_LIMIT        64u

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
 * wall-clock transaction lifetime.  Each interrupt-driven state change
 * refreshes the timestamp, so a slow-but-progressing transfer is not killed
 * while a genuine bus stall still times out.
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
    I2C_ITConfig(I2C1, (uint16_t)(I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR), DISABLE);

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

/* 只复位 I2C 外设本身（不含 GPIO 位翻转恢复）：清掉所有标志与状态机。
 *
 * SB / BTF 这两个事件标志只能靠“读/写 DR”清 —— 那会往总线发真实数据，
 * 绝不能为了清标志去做；外设复位（RCC 复位 + 重新初始化）是唯一安全且
 * 彻底的办法。 */
static void i2c_peripheral_reset(void)
{
    I2C_ITConfig(I2C1, (uint16_t)(I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR), DISABLE);
    stop_dma();
    I2C_Cmd(I2C1, DISABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
    TIME_DelayUs(2u);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);
    i2c_hw_init();
}

/* 开始新事务前的“清场”：把上一条事务可能留下的硬件标志处理掉。
 *
 * 返回 1 = 可以发 START；返回 0 = 总线还在别人手里，交给看门狗稍后重试。
 *
 * 哪些残留是“正常”的：
 *   - ADDR / STOPF：读 STAR1→STAR2 就没了；
 *   - BTF（TX 负载发完必然置位）和 SB（START 已生成但地址还没写）
 *     清不掉，必须复位外设；这类残留出现在被看门狗中止的事务、或
 *     TX-DMA 结束后的那一小段窗口里。
 * 旧代码不处理这些，下一次一开 IT_EVT 就被电平触发的事件中断立刻打断，
 * 形成 ~10^6 次/秒 的中断风暴。 */
static uint8_t i2c_prepare_start(void)
{
    uint16_t star1 = I2C1->STAR1;   /* 读 STAR1… */
    uint16_t star2 = I2C1->STAR2;   /* …再读 STAR2：清 ADDR/STOPF */
    uint16_t stale = (uint16_t)(star1 & (uint16_t)(I2C_STAR1_SB | I2C_STAR1_BTF));

    if(stale == 0u)
        return 1u;

    if((star2 & I2C_STAR2_BUSY) != 0u)
        return 0u;

    i2c_peripheral_reset();
    return 1u;
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

    if(((I2C1->STAR2 & I2C_STAR2_BUSY) != 0u) ||
       ((I2C1->STAR1 & I2C_STAR1_BTF) != 0u))
        I2C_GenerateSTOP(I2C1, ENABLE);   /* STOP 同时会清掉 BTF */

    /* Every transaction step is interrupt-driven: shut all sources down before
     * the terminal result is published. */
    I2C_ITConfig(I2C1, (uint16_t)(I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR), DISABLE);

    clear_i2c_error_flags();
    /* STAR1→STAR2 读序列：把可能残留的 ADDR/STOPF 清掉，否则下一次
     * start_transaction() 一使能 IT_EVT 就会被电平触发的事件中断立即重入。 */
    (void)I2C1->STAR1;
    (void)I2C1->STAR2;
    s_xfer.state = I2C_API_STATE_IDLE;
    s_xfer.result = result;
    if(request_recovery)
        s_recovery_requested = 1u;
}

static void start_transaction(void)
{
    /* 先置状态再开中断：SB 可能在 GenerateSTART 之后的一两个指令内就到了，
     * 那时状态机必须已经是 WAIT_START_TX。 */
    set_state(I2C_API_STATE_WAIT_START_TX);
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Current);
    I2C_ITConfig(I2C1, (uint16_t)(I2C_IT_EVT | I2C_IT_ERR), ENABLE);
    I2C_GenerateSTART(I2C1, ENABLE);
}

/* The bus is normally free right after our own STOP.  Wait briefly for BUSY to
 * clear; if it is still busy afterwards the transaction rests in
 * WAIT_BUS_IDLE and the 10 ms watchdog retries the start. */
static uint8_t wait_bus_free(void)
{
    uint32_t elapsed_us = 0u;

    while((I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) != RESET) &&
          (elapsed_us < I2C_API_BUS_FREE_SPIN_US))
    {
        TIME_DelayUs(1u);
        ++elapsed_us;
    }

    return (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == RESET) ? 1u : 0u;
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
    s_last_error = I2C_API_ERR_NONE;

    if(wait_bus_free() && i2c_prepare_start())
    {
        start_transaction();
    }
    else
    {
        set_state(I2C_API_STATE_WAIT_BUS_IDLE);
    }

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
    DMA_ITConfig(I2C_API_DMA_TX_CHANNEL, (uint32_t)(DMA_IT_TC | DMA_IT_TE), ENABLE);
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
    DMA_ITConfig(I2C_API_DMA_RX_CHANNEL, (uint32_t)(DMA_IT_TC | DMA_IT_TE), ENABLE);
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
    NVIC_InitTypeDef nvic = {0};

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

    /* Event/error and both DMA channels carry the transaction; only the
     * watchdog still runs from a coroutine. */
    nvic.NVIC_IRQChannel = I2C1_EV_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1u;
    nvic.NVIC_IRQChannelSubPriority = 0u;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    nvic.NVIC_IRQChannel = I2C1_ER_IRQn;
    NVIC_Init(&nvic);

    nvic.NVIC_IRQChannel = DMA1_Channel6_IRQn;
    nvic.NVIC_IRQChannelSubPriority = 1u;
    NVIC_Init(&nvic);

    nvic.NVIC_IRQChannel = DMA1_Channel7_IRQn;
    NVIC_Init(&nvic);

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

/*
 * Interrupt-driven transaction engine.
 *
 *   I2C1_EV  : SB -> address phase; ADDR -> DMA start / repeated START /
 *              single-byte read; RXNE is not enabled (see WAIT_ADDR_RX).
 *   I2C1_ER  : AF / BERR / ARLO / OVR.
 *   DMA1 CH6 : TX payload done -> bounded BTF wait -> repeated START or STOP.
 *   DMA1 CH7 : RX payload done -> STOP.
 *
 * The former foreground I2C_API_Service() polling loop is gone; only the
 * 10 ms watchdog still runs in a coroutine.
 */
/* 事件标志清不掉时的熔断：关掉全部 I2C 中断源、拉 STOP、请求总线恢复。
 *
 * WCH/STM32 系 I2C 的事件中断是电平触发：只要 SB/ADDR/BTF/STOPF 里有一个
 * 没被清掉，IT_EVT 一开就会无限重入——实测可达 ~10^6 次/秒。I2C1_EV 的
 * 抢占优先级高于 SysTick，所以一旦风暴，1 ms 节拍、IWDG 喂狗全部停摆，
 * 最后表现为“停在某页 → IWDG 复位”。 */
static void i2c_abort_event_storm(I2C_API_Error error)
{
    uint16_t star2 = I2C1->STAR2;

    I2C_ITConfig(I2C1, (uint16_t)(I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR), DISABLE);
    stop_dma();

    if((star2 & I2C_STAR2_BUSY) != 0u)
        I2C_GenerateSTOP(I2C1, ENABLE);

    clear_i2c_error_flags();
    s_recovery_requested = 1u;

    if(s_xfer.result == I2C_API_RESULT_ACTIVE)
    {
        snapshot(error);
        s_xfer.state = I2C_API_STATE_IDLE;
        s_xfer.result = I2C_API_RESULT_TIMEOUT;
    }
}

void I2C_API_EV_IRQHandler(void)
{
    static I2C_API_State storm_state = I2C_API_STATE_IDLE;
    static uint32_t storm_entries;
    I2C_API_State state_now = s_xfer.state;
    uint16_t star1;

    /* ---- 风暴熔断：同一状态被事件中断反复打断 => 有标志清不掉 ---- */
    if(state_now == storm_state)
    {
        storm_entries++;
    }
    else
    {
        storm_state = state_now;
        storm_entries = 0u;
    }

    if(storm_entries > I2C_API_EV_STORM_LIMIT)
    {
        storm_entries = 0u;
        i2c_abort_event_storm(timeout_error_for_state(state_now));
        return;
    }

    star1 = I2C1->STAR1;

    if(s_xfer.result != I2C_API_RESULT_ACTIVE)
    {
        /* 事务已经结束（正常收尾 / 看门狗中止）但还有事件中断进来：
         * ADDR/STOPF 靠上面那次 STAR1 读 + 这里的 STAR2 读清掉；SB/BTF 不能
         * 在这里清（只能靠读/写 DR，会往总线发真数据），它们会在下一次
         * start 前由 i2c_prepare_start() 通过“外设复位”处理。
         * 这里只要把中断源关掉就行——绝不能熔断/请求恢复，否则正常收尾的
         * TX-DMA 事务（结束时 BTF 必然置位）会把 I2C 反复复位。 */
        (void)I2C1->STAR2;                             /* 清 ADDR/STOPF */
        I2C_ITConfig(I2C1, (uint16_t)I2C_IT_EVT, DISABLE);
        return;
    }

    if((star1 & I2C_STAR1_SB) != 0u)
    {
        if(s_xfer.state == I2C_API_STATE_WAIT_START_TX)
        {
            I2C_Send7bitAddress(I2C1,
                                (uint8_t)(s_xfer.address_7bit << 1),
                                I2C_Direction_Transmitter);
            set_state(I2C_API_STATE_WAIT_ADDR_TX);
        }
        else if(s_xfer.state == I2C_API_STATE_WAIT_START_RX)
        {
            /* For a one-byte receive ACK must already be low when ADDR is
             * cleared; DMA receives keep ACK enabled and use LAST. */
            if(s_xfer.rx_len == 1u)
                I2C_AcknowledgeConfig(I2C1, DISABLE);
            else
                I2C_AcknowledgeConfig(I2C1, ENABLE);

            I2C_Send7bitAddress(I2C1,
                                (uint8_t)(s_xfer.address_7bit << 1),
                                I2C_Direction_Receiver);
            set_state(I2C_API_STATE_WAIT_ADDR_RX);
        }
        else
        {
            /* SB 落在非预期状态：不能用写地址来清（会往总线发错误地址），
             * 直接熔断并让10 ms 看门狗做总线/外设恢复。 */
            i2c_abort_event_storm(I2C_API_ERR_START_TX);
        }
        return;
    }

    if((star1 & I2C_STAR1_ADDR) != 0u)
    {
        if(s_xfer.state == I2C_API_STATE_WAIT_ADDR_TX)
        {
            clear_addr_flag();

            if(s_xfer.tx_len != 0u)
            {
                start_tx_dma();
            }
            else if(s_xfer.rx_len != 0u)
            {
                I2C_GenerateSTART(I2C1, ENABLE);
                set_state(I2C_API_STATE_WAIT_START_RX);
            }
            else
            {
                finish(I2C_API_RESULT_OK, I2C_API_ERR_NONE, 0u);
            }
        }
        else if(s_xfer.state == I2C_API_STATE_WAIT_ADDR_RX)
        {
            if(s_xfer.rx_len == 1u)
            {
                /* Single-byte sequence: ACK=0 -> clear ADDR -> STOP, then
                 * read the byte.  Bounded spin (one byte time at 400 kHz)
                 * instead of ITBUFEN: enabling the buffer interrupt here would
                 * also raise the TXE event, which cannot be cleared without
                 * writing the data register. */
                uint32_t guard = 200000u;

                clear_addr_flag();
                I2C_GenerateSTOP(I2C1, ENABLE);
                set_state(I2C_API_STATE_WAIT_RX_SINGLE);

                while(((I2C1->STAR1 & I2C_STAR1_RXNE) == 0u) && guard)
                    --guard;

                if((I2C1->STAR1 & I2C_STAR1_RXNE) != 0u)
                {
                    s_xfer.rx[0] = I2C_ReceiveData(I2C1);
                    I2C_AcknowledgeConfig(I2C1, ENABLE);
                    finish(I2C_API_RESULT_OK, I2C_API_ERR_NONE, 0u);
                }
                else
                {
                    finish(I2C_API_RESULT_ERROR, I2C_API_ERR_RX, 1u);
                }
            }
            else
            {
                /* Configure DMA + LAST before clearing ADDR so the first
                 * byte cannot outrun DMA setup at 400 kHz. */
                start_rx_dma();
                clear_addr_flag();
            }
        }
        else
        {
            /* ADDR 落在非预期状态：读一次 STAR2 清掉即可（状态机会由看门狗收尾） */
            (void)I2C1->STAR2;
        }
        return;
    }

    /* 到这里剩下的事件标志状态机都没在等：STOPF 读一次 STAR2 就没了；
     * BTF 清不掉，但也不该在这里熔断（TX-DMA 正常结束时 BTF 必然置位），
     * 让它随本次事务收尾的 STOP / 重复 START 消失，真赖着不走则由下一次
     * i2c_prepare_start() 的外设复位清掉——上面的风暴熔断只当最后兑底。 */
    if((star1 & I2C_STAR1_STOPF) != 0u)
        (void)I2C1->STAR2;
}

void I2C_API_ER_IRQHandler(void)
{
    if(s_xfer.result != I2C_API_RESULT_ACTIVE)
    {
        /* Stray error with no active transaction: just clear it. */
        clear_i2c_error_flags();
        I2C_ClearITPendingBit(I2C1, I2C_IT_PECERR);
        return;
    }

    if(!handle_error_flags())
    {
        /* PECERR is SMBus-only and unused here; clear it so the error
         * interrupt cannot stay asserted. */
        I2C_ClearITPendingBit(I2C1, I2C_IT_PECERR);
    }
}

void I2C_API_TxDMA_IRQHandler(void)
{
    uint32_t guard = 200000u;

    if((s_xfer.result != I2C_API_RESULT_ACTIVE) ||
       (s_xfer.state != I2C_API_STATE_WAIT_TX_DMA))
    {
        /* No transaction owns this channel any more (for example after a
         * watchdog abort): just silence the channel. */
        DMA_ClearITPendingBit(DMA1_IT_TC6);
        DMA_ClearITPendingBit(DMA1_IT_TE6);
        DMA_Cmd(I2C_API_DMA_TX_CHANNEL, DISABLE);
        return;
    }

    if(DMA_GetITStatus(DMA1_IT_TE6) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_IT_TE6);
        finish(I2C_API_RESULT_ERROR, I2C_API_ERR_TX_DMA, 1u);
        return;
    }

    if(DMA_GetITStatus(DMA1_IT_TC6) == RESET)
        return;

    DMA_ClearITPendingBit(DMA1_IT_TC6);
    DMA_Cmd(I2C_API_DMA_TX_CHANNEL, DISABLE);
    DMA_ClearFlag(DMA1_FLAG_GL6);
    I2C_DMACmd(I2C1, DISABLE);

    /* DMA TC means the last byte reached DATAR.  Wait for BTF (bounded, at
     * most one byte time at 400 kHz) so the following repeated START or STOP
     * is positioned after the final ACK bit.
     *
     * BTF 本身也会产生一次事件中断：等待期间先把 EV 源关掉，否则中断会在
     * 这段窗口里冲进来，把 BTF 留成“事务已结束却还挂着的事件标志”
     * （旧代码下一次开 IT_EVT 时就是被它打成风暴的）。 */
    I2C_ITConfig(I2C1, (uint16_t)I2C_IT_EVT, DISABLE);
    while(((I2C1->STAR1 & I2C_STAR1_BTF) == 0u) && guard)
        --guard;

    if(s_xfer.rx_len != 0u)
    {
        I2C_ITConfig(I2C1, (uint16_t)I2C_IT_EVT, ENABLE);
        I2C_GenerateSTART(I2C1, ENABLE);
        set_state(I2C_API_STATE_WAIT_START_RX);
    }
    else
    {
        finish(I2C_API_RESULT_OK, I2C_API_ERR_NONE, 0u);
    }
}

void I2C_API_RxDMA_IRQHandler(void)
{
    if((s_xfer.result != I2C_API_RESULT_ACTIVE) ||
       (s_xfer.state != I2C_API_STATE_WAIT_RX_DMA))
    {
        DMA_ClearITPendingBit(DMA1_IT_TC7);
        DMA_ClearITPendingBit(DMA1_IT_TE7);
        DMA_Cmd(I2C_API_DMA_RX_CHANNEL, DISABLE);
        return;
    }

    if(DMA_GetITStatus(DMA1_IT_TE7) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_IT_TE7);
        finish(I2C_API_RESULT_ERROR, I2C_API_ERR_RX_DMA, 1u);
        return;
    }

    if(DMA_GetITStatus(DMA1_IT_TC7) == RESET)
        return;

    DMA_ClearITPendingBit(DMA1_IT_TC7);
    DMA_Cmd(I2C_API_DMA_RX_CHANNEL, DISABLE);
    DMA_ClearFlag(DMA1_FLAG_GL7);
    I2C_DMACmd(I2C1, DISABLE);
    I2C_DMALastTransferCmd(I2C1, DISABLE);
    I2C_GenerateSTOP(I2C1, ENABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    finish(I2C_API_RESULT_OK, I2C_API_ERR_NONE, 0u);
}

void I2C_API_WatchdogService(uint32_t now_ms)
{
    if(s_xfer.result == I2C_API_RESULT_ACTIVE)
    {
        if((uint32_t)(now_ms - s_xfer.last_progress_ms) > I2C_API_WATCHDOG_TIMEOUT_MS)
        {
            finish(I2C_API_RESULT_TIMEOUT, timeout_error_for_state(s_xfer.state), 1u);
        }
        else if((s_xfer.state == I2C_API_STATE_WAIT_BUS_IDLE) &&
                (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == RESET))
        {
            /* Rare deferred start: the bus was still busy when the caller
             * requested the transaction and no event interrupt will come. */
            if(i2c_prepare_start())
                start_transaction();
        }
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

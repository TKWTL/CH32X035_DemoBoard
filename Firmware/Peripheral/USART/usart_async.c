#include "usart_async.h"
#include "main.h"
#include "ch32x035.h"
#include "time_api.h"
#include <string.h>

#define USART1_TX_DMA_CH             DMA1_Channel4
#define USART1_RX_DMA_CH             DMA1_Channel5
#define USART1_TX_DMA_TC_IT          DMA1_IT_TC4
#define USART1_RX_DMA_TC_IT          DMA1_IT_TC5
#define USART1_TX_DMA_GL_FLAG        DMA1_FLAG_GL4
#define USART1_RX_DMA_GL_FLAG        DMA1_FLAG_GL5
#define USART1_TX_BACKPRESSURE_US    1000UL

#if ((USART1_ASYNC_TX_BUFFER_SIZE & (USART1_ASYNC_TX_BUFFER_SIZE - 1u)) != 0)
#error "USART1 TX buffer size must be a power of two"
#endif
#if ((USART1_ASYNC_RX_BUFFER_SIZE & (USART1_ASYNC_RX_BUFFER_SIZE - 1u)) != 0)
#error "USART1 RX buffer size must be a power of two"
#endif

/* WCH DMA examples explicitly require DMA buffers to be 4-byte aligned. */
static uint8_t s_tx_buffer[USART1_ASYNC_TX_BUFFER_SIZE] __attribute__((aligned(4)));
static uint8_t s_rx_buffer[USART1_ASYNC_RX_BUFFER_SIZE] __attribute__((aligned(4)));

static volatile uint16_t s_tx_head;
static volatile uint16_t s_tx_tail;
static volatile uint16_t s_tx_count;
static volatile uint16_t s_tx_active_len;
static volatile uint8_t s_tx_busy;
static volatile uint32_t s_tx_dropped;

/* RX uses monotonic byte positions so a full 256-byte DMA turn is not confused
 * with an empty ring. DMA5 TC increments the high-order turn counter. */
static volatile uint32_t s_rx_wrap_count;
static uint32_t s_rx_read_total;
static volatile uint32_t s_rx_overrun;

static void USART1_Async_StartTxDMA(void)
{
    uint16_t count;
    uint16_t head;

    if(s_tx_busy || (s_tx_count == 0u))
        return;

    head = s_tx_head;
    count = s_tx_count;
    if(count > (uint16_t)(USART1_ASYNC_TX_BUFFER_SIZE - head))
        count = (uint16_t)(USART1_ASYNC_TX_BUFFER_SIZE - head);

    /* Match WCH's documented TX-DMA start order:
     *  1. disable USART DMA request and DMA channel;
     *  2. load MADDR/CNTR and clear stale flags;
     *  3. enable DMA channel;
     *  4. enable USART TX DMA request LAST so TXE generates the first request.
     * Keeping DMAT enabled while the channel is disabled can lose the initial
     * request on CH32X035 and leave the UART permanently silent. */
    USART_DMACmd(USART1, USART_DMAReq_Tx, DISABLE);
    DMA_Cmd(USART1_TX_DMA_CH, DISABLE);
    DMA_ClearFlag(USART1_TX_DMA_GL_FLAG);
    USART_ClearFlag(USART1, USART_FLAG_TC);

    USART1_TX_DMA_CH->MADDR = (uint32_t)(uintptr_t)&s_tx_buffer[head];
    DMA_SetCurrDataCounter(USART1_TX_DMA_CH, count);

    s_tx_active_len = count;
    s_tx_busy = 1u;

    DMA_Cmd(USART1_TX_DMA_CH, ENABLE);
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
}

static void USART1_Async_FinishTxDMA(void)
{
    uint16_t finished = s_tx_active_len;

    /* Stop the peripheral request before touching the DMA channel. */
    USART_DMACmd(USART1, USART_DMAReq_Tx, DISABLE);
    DMA_Cmd(USART1_TX_DMA_CH, DISABLE);
    DMA_ClearITPendingBit(USART1_TX_DMA_TC_IT);

    if(s_tx_count >= finished)
        s_tx_count = (uint16_t)(s_tx_count - finished);
    else
        s_tx_count = 0u;

    s_tx_head = (uint16_t)((s_tx_head + finished) &
                           (USART1_ASYNC_TX_BUFFER_SIZE - 1u));
    s_tx_active_len = 0u;
    s_tx_busy = 0u;

    USART1_Async_StartTxDMA();
}

static uint32_t USART1_Async_GetRxWriteTotal(void)
{
    uint32_t wrap_a;
    uint32_t wrap_b;
    uint16_t pos;

    do
    {
        wrap_a = s_rx_wrap_count;
        pos = (uint16_t)(USART1_ASYNC_RX_BUFFER_SIZE -
                         DMA_GetCurrDataCounter(USART1_RX_DMA_CH));
        if(pos >= USART1_ASYNC_RX_BUFFER_SIZE)
            pos = 0u;
        wrap_b = s_rx_wrap_count;
    } while(wrap_a != wrap_b);

    return (wrap_a * USART1_ASYNC_RX_BUFFER_SIZE) + pos;
}

static uint16_t USART1_Async_NormalizeRxAvailable(uint32_t *write_total)
{
    uint32_t available;

    *write_total = USART1_Async_GetRxWriteTotal();
    available = *write_total - s_rx_read_total;

    if(available > USART1_ASYNC_RX_BUFFER_SIZE)
    {
        uint32_t lost = available - USART1_ASYNC_RX_BUFFER_SIZE;
        s_rx_overrun += lost;
        s_rx_read_total = *write_total - USART1_ASYNC_RX_BUFFER_SIZE;
        available = USART1_ASYNC_RX_BUFFER_SIZE;
    }

    return (uint16_t)available;
}

void USART1_Async_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef gpio = {0};
    USART_InitTypeDef usart = {0};
    DMA_InitTypeDef dma = {0};
    NVIC_InitTypeDef nvic = {0};

    s_tx_head = 0u;
    s_tx_tail = 0u;
    s_tx_count = 0u;
    s_tx_active_len = 0u;
    s_tx_busy = 0u;
    s_tx_dropped = 0u;
    s_rx_wrap_count = 0u;
    s_rx_read_total = 0u;
    s_rx_overrun = 0u;

    RCC_APB2PeriphClockCmd(RX_GPIO_CLK | TX_GPIO_CLK | RCC_APB2Periph_USART1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    /* Put TX high before handing PB10 to USART. WCH uses this sequence in its
     * DMA/CDC UART driver to avoid an unwanted low level during re-init. */
    GPIO_SetBits(RX_GPIO_Port, RX_Pin);
    gpio.GPIO_Pin = RX_Pin;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(RX_GPIO_Port, &gpio);

    USART_DeInit(USART1);

    gpio.GPIO_Pin = TX_Pin;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(TX_GPIO_Port, &gpio);

    gpio.GPIO_Pin = RX_Pin;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(RX_GPIO_Port, &gpio);

    usart.USART_BaudRate = baudrate;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &usart);
    USART_ClearFlag(USART1, USART_FLAG_TC);
    USART_Cmd(USART1, ENABLE);

    /* TX DMA: normal mode; each completion advances the software ring. */
    DMA_DeInit(USART1_TX_DMA_CH);
    dma.DMA_PeripheralBaseAddr = (uint32_t)(uintptr_t)&USART1->DATAR;
    dma.DMA_MemoryBaseAddr = (uint32_t)(uintptr_t)s_tx_buffer;
    dma.DMA_DIR = DMA_DIR_PeripheralDST;
    dma.DMA_BufferSize = 1u;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Normal;
    dma.DMA_Priority = DMA_Priority_Low;
    dma.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(USART1_TX_DMA_CH, &dma);
    DMA_ITConfig(USART1_TX_DMA_CH, DMA_IT_TC, ENABLE);

    /* RX DMA: continuously writes one 256-byte ring. TC counts complete turns. */
    DMA_DeInit(USART1_RX_DMA_CH);
    dma.DMA_PeripheralBaseAddr = (uint32_t)(uintptr_t)&USART1->DATAR;
    dma.DMA_MemoryBaseAddr = (uint32_t)(uintptr_t)s_rx_buffer;
    dma.DMA_DIR = DMA_DIR_PeripheralSRC;
    dma.DMA_BufferSize = USART1_ASYNC_RX_BUFFER_SIZE;
    dma.DMA_Mode = DMA_Mode_Circular;
    dma.DMA_Priority = DMA_Priority_Low;
    DMA_Init(USART1_RX_DMA_CH, &dma);
    DMA_ITConfig(USART1_RX_DMA_CH, DMA_IT_TC, ENABLE);

    /* USBPD keeps the default higher priority. UART DMA is deliberately lower. */
    nvic.NVIC_IRQChannel = DMA1_Channel4_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1u;
    nvic.NVIC_IRQChannelSubPriority = 2u;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    nvic.NVIC_IRQChannel = DMA1_Channel5_IRQn;
    nvic.NVIC_IRQChannelSubPriority = 3u;
    NVIC_Init(&nvic);

    /* RX follows the same safe order: channel first, peripheral request last.
     * TX DMAT remains disabled until USART1_Async_StartTxDMA(). */
    DMA_ClearFlag(USART1_RX_DMA_GL_FLAG);
    DMA_Cmd(USART1_RX_DMA_CH, ENABLE);
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
}

void USART1_Async_Service(void)
{
    /* IRQ is the normal completion path. Polling the flag here is a recovery
     * path for a missed/pending IRQ and also makes the driver robust during
     * early boot while the application is rapidly queueing printf output. */
    NVIC_DisableIRQ(DMA1_Channel4_IRQn);
    if(s_tx_busy && (DMA_GetFlagStatus(DMA1_FLAG_TC4) != RESET))
        USART1_Async_FinishTxDMA();
    else if(!s_tx_busy && (s_tx_count != 0u))
        USART1_Async_StartTxDMA();
    NVIC_EnableIRQ(DMA1_Channel4_IRQn);
}

uint8_t USART1_Async_Flush(uint32_t timeout_us)
{
    uint32_t start = TIME_Micros();

    do
    {
        USART1_Async_Service();
        if(!USART1_Async_TxBusy())
        {
            /* DMA TC only means the final byte has been copied into USART1.
             * Wait for USART TC as well so the shift register is physically
             * idle before a timing-critical USB-PD transaction begins. */
            while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
            {
                if((uint32_t)(TIME_Micros() - start) >= timeout_us)
                    return 0u;
            }
            return 1u;
        }
    } while((uint32_t)(TIME_Micros() - start) < timeout_us);

    return 0u;
}

uint16_t USART1_Async_Write(const uint8_t *data, uint16_t length)
{
    uint16_t accepted = 0u;
    uint32_t wait_start;

    if((data == 0) || (length == 0u))
        return 0u;

    wait_start = TIME_Micros();

    while(accepted < length)
    {
        uint16_t free_count;
        uint16_t chunk = 0u;
        uint16_t tail;

        USART1_Async_Service();

        /* Batch-copy data into the software ring. This avoids starting the
         * first DMA transfer with only one byte and greatly reduces DMA IRQs. */
        NVIC_DisableIRQ(DMA1_Channel4_IRQn);
        free_count = (uint16_t)(USART1_ASYNC_TX_BUFFER_SIZE - s_tx_count);
        if(free_count != 0u)
        {
            chunk = (uint16_t)(length - accepted);
            if(chunk > free_count)
                chunk = free_count;

            tail = s_tx_tail;
            if(chunk > (uint16_t)(USART1_ASYNC_TX_BUFFER_SIZE - tail))
            {
                uint16_t first = (uint16_t)(USART1_ASYNC_TX_BUFFER_SIZE - tail);
                memcpy(&s_tx_buffer[tail], &data[accepted], first);
                memcpy(&s_tx_buffer[0], &data[accepted + first], (uint16_t)(chunk - first));
            }
            else
            {
                memcpy(&s_tx_buffer[tail], &data[accepted], chunk);
            }

            s_tx_tail = (uint16_t)((tail + chunk) &
                                   (USART1_ASYNC_TX_BUFFER_SIZE - 1u));
            s_tx_count = (uint16_t)(s_tx_count + chunk);
            accepted = (uint16_t)(accepted + chunk);
        }
        NVIC_EnableIRQ(DMA1_Channel4_IRQn);

        USART1_Async_Service();

        if(chunk != 0u)
        {
            wait_start = TIME_Micros();
        }
        else if((uint32_t)(TIME_Micros() - wait_start) >= USART1_TX_BACKPRESSURE_US)
        {
            s_tx_dropped += (uint32_t)(length - accepted);
            break;
        }
    }

    return accepted;
}

uint8_t USART1_Async_WriteByte(uint8_t data)
{
    return (USART1_Async_Write(&data, 1u) == 1u) ? 1u : 0u;
}

uint16_t USART1_Async_RxAvailable(void)
{
    uint32_t write_total;
    return USART1_Async_NormalizeRxAvailable(&write_total);
}

uint16_t USART1_Async_Read(uint8_t *data, uint16_t max_length)
{
    uint32_t write_total;
    uint16_t available;
    uint16_t count = 0u;

    if((data == 0) || (max_length == 0u))
        return 0u;

    available = USART1_Async_NormalizeRxAvailable(&write_total);
    if(available > max_length)
        available = max_length;

    while(count < available)
    {
        data[count++] = s_rx_buffer[(uint16_t)(s_rx_read_total &
                                      (USART1_ASYNC_RX_BUFFER_SIZE - 1u))];
        s_rx_read_total++;
    }

    return count;
}

uint8_t USART1_Async_ReadByte(uint8_t *data)
{
    return (USART1_Async_Read(data, 1u) == 1u) ? 1u : 0u;
}

uint16_t USART1_Async_TxPending(void)
{
    return s_tx_count;
}

uint8_t USART1_Async_TxBusy(void)
{
    return (s_tx_busy || (s_tx_count != 0u)) ? 1u : 0u;
}

uint32_t USART1_Async_GetTxDropped(void)
{
    return s_tx_dropped;
}

uint32_t USART1_Async_GetRxOverrun(void)
{
    return s_rx_overrun;
}

void USART1_Async_TxDMA_IRQHandler(void)
{
    if(DMA_GetITStatus(USART1_TX_DMA_TC_IT) != RESET)
        USART1_Async_FinishTxDMA();
}

void USART1_Async_RxDMA_IRQHandler(void)
{
    if(DMA_GetITStatus(USART1_RX_DMA_TC_IT) != RESET)
    {
        s_rx_wrap_count++;
        DMA_ClearITPendingBit(USART1_RX_DMA_TC_IT);
    }
}

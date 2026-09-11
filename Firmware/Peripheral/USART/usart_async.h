#ifndef USART_ASYNC_H_
#define USART_ASYNC_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define USART1_ASYNC_TX_BUFFER_SIZE  256u
#define USART1_ASYNC_RX_BUFFER_SIZE  256u

/* USART1 on this board: PB10 TX, PB11 RX.
 * TX: DMA1 Channel4, normal DMA chunks fed from a 256-byte ring.
 * RX: DMA1 Channel5, 256-byte circular DMA plus a monotonic wrap counter.
 */
void USART1_Async_Init(uint32_t baudrate);
/* Cooperative foreground service. DMA IRQs normally advance TX, while this
 * call provides a polling recovery path and should run once per scheduler pass. */
void USART1_Async_Service(void);
uint8_t USART1_Async_Flush(uint32_t timeout_us);

/* Enqueue data for background DMA transmission.
 * The call normally returns immediately. If the TX ring is full it applies a
 * short bounded back-pressure window; if DMA still cannot make progress the
 * unsent remainder is dropped rather than blocking the cooperative scheduler
 * indefinitely. Returns the number of bytes accepted. */
uint16_t USART1_Async_Write(const uint8_t *data, uint16_t length);
uint8_t USART1_Async_WriteByte(uint8_t data);

uint16_t USART1_Async_Read(uint8_t *data, uint16_t max_length);
uint8_t USART1_Async_ReadByte(uint8_t *data);
uint16_t USART1_Async_RxAvailable(void);
uint16_t USART1_Async_TxPending(void);
uint8_t USART1_Async_TxBusy(void);
uint32_t USART1_Async_GetTxDropped(void);
uint32_t USART1_Async_GetRxOverrun(void);

/* Called only by APP/ch32x035_it.c vector wrappers. */
void USART1_Async_TxDMA_IRQHandler(void);
void USART1_Async_RxDMA_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* USART_ASYNC_H_ */

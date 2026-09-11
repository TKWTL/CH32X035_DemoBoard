#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ch32x035.h"

/*
 * CubeMX-style board net aliases.
 *
 * The identifiers follow schematic/net labels rather than MCU signal direction:
 *   RX on PB10 is the board UART-RX net driven by MCU USART1_TX.
 *   TX on PB11 is the board UART-TX net received by MCU USART1_RX.
 *   CCPD is active-low (#CCPD) in the schematic.
 *
 * Pin aliases are centralized here instead of being repeated in drivers.
 * When changing a mapping, also respect the CH32X035 alternate-function/
 * peripheral routing constraints; CC1/CC2 in particular are USBPD PHY pins.
 */

/* UART header nets */
#define RX_Pin                  GPIO_Pin_10
#define RX_GPIO_Port            GPIOB
#define RX_GPIO_CLK             RCC_APB2Periph_GPIOB

#define TX_Pin                  GPIO_Pin_11
#define TX_GPIO_Port            GPIOB
#define TX_GPIO_CLK             RCC_APB2Periph_GPIOB

/* INA226 / OLED I2C bus */
#define SCL_Pin                 GPIO_Pin_10
#define SCL_GPIO_Port           GPIOA
#define SCL_GPIO_CLK            RCC_APB2Periph_GPIOA

#define SDA_Pin                 GPIO_Pin_11
#define SDA_GPIO_Port           GPIOA
#define SDA_GPIO_CLK            RCC_APB2Periph_GPIOA

/* WS2812 chain input */
#define DIN_Pin                 GPIO_Pin_7
#define DIN_GPIO_Port           GPIOC
#define DIN_GPIO_CLK            RCC_APB2Periph_GPIOC

/* USB-PD CC frontend */
#define CC1_Pin                 GPIO_Pin_14
#define CC1_GPIO_Port           GPIOC
#define CC1_GPIO_CLK            RCC_APB2Periph_GPIOC

#define CC2_Pin                 GPIO_Pin_15
#define CC2_GPIO_Port           GPIOC
#define CC2_GPIO_CLK            RCC_APB2Periph_GPIOC

#define CCEN_Pin                GPIO_Pin_9
#define CCEN_GPIO_Port          GPIOB
#define CCEN_GPIO_CLK           RCC_APB2Periph_GPIOB

/* Schematic net is #CCPD: low enables the external 5.1 kOhm Rd network. */
#define CCPD_Pin                GPIO_Pin_2
#define CCPD_GPIO_Port          GPIOB
#define CCPD_GPIO_CLK           RCC_APB2Periph_GPIOB

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

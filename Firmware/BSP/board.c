#include "board.h"

void Board_PD_Frontend_EnableSink(void)
{
    GPIO_InitTypeDef gpio = {0};

    RCC_APB2PeriphClockCmd(CCEN_GPIO_CLK | CCPD_GPIO_CLK, ENABLE);

    gpio.GPIO_Pin = CCEN_Pin;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(CCEN_GPIO_Port, &gpio);

    gpio.GPIO_Pin = CCPD_Pin;
    GPIO_Init(CCPD_GPIO_Port, &gpio);

    /* Q2 is N-channel: high CCEN connects PC14/PC15 to CC1/CC2. */
    GPIO_SetBits(CCEN_GPIO_Port, CCEN_Pin);

    /* Q1 is dual P-channel and drives the external 5.1k Rd network.
     * Gate low => Q1 on => Rd connected => present as a Sink. */
    GPIO_ResetBits(CCPD_GPIO_Port, CCPD_Pin);
}

void Board_Init(void)
{
    Board_PD_Frontend_EnableSink();
}

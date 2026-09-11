#ifndef __BOARD_H
#define __BOARD_H

#include "main.h"

/* Board GPIO aliases are centralized in APP/main.h in CubeMX-style
 * <NET>_Pin / <NET>_GPIO_Port form. */

void Board_Init(void);
void Board_PD_Frontend_EnableSink(void);

#endif

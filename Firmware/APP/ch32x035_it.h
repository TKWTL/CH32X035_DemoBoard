#ifndef __CH32X035_IT_H
#define __CH32X035_IT_H

#include "ch32x035.h"

/* Preemption policy table + VTF entry map (see ch32x035_it.c).  Call once
 * from main() right after NVIC_PriorityGroupConfig(). */
void APP_IRQ_Init(void);

#endif /* __CH32X035_IT_H */

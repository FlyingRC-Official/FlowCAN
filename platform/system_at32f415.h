#ifndef SYSTEM_AT32F415_H
#define SYSTEM_AT32F415_H
#include <stdint.h>
extern uint32_t system_core_clock;
void SystemInit(void);
void SystemCoreClockUpdate(void);
#endif

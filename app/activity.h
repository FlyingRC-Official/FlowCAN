#ifndef FLOWCAN_ACTIVITY_H
#define FLOWCAN_ACTIVITY_H
#include <stdint.h>
#include "platform.h"
void activity_init(void);
void activity_pulse(activity_led_t led,uint32_t now_ms);
void activity_step(uint32_t now_ms);
#endif

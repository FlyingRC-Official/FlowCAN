#ifndef FLOWCAN_STATUS_LED_H
#define FLOWCAN_STATUS_LED_H
#include <stdbool.h>
#include <stdint.h>
#include "flowcan/types.h"
void status_led_step(uint32_t now_ms,health_flags_t health,bool initializing);
#endif

#ifndef FLOWCAN_RTOS_APP_H
#define FLOWCAN_RTOS_APP_H

#include <stdbool.h>

bool rtos_app_init(void);
void rtos_notify_flow_from_isr(void);
void rtos_notify_range_from_isr(void);
void rtos_notify_communication_from_isr(void);
void flowcan_rtos_assert_failed(const char *file, int line);

#endif

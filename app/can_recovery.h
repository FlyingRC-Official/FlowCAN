#ifndef FLOWCAN_CAN_RECOVERY_H
#define FLOWCAN_CAN_RECOVERY_H
#include <stdbool.h>
#include <stdint.h>
typedef struct{bool active;uint32_t retry_at_ms;}can_recovery_t;
void can_recovery_init(can_recovery_t *state);
bool can_recovery_step(can_recovery_t *state,bool bus_off,uint32_t now_ms,uint32_t retry_ms);
#endif

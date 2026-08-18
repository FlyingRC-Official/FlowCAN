#ifndef FLOWCAN_VL53L1X_H
#define FLOWCAN_VL53L1X_H

#include <stdbool.h>
#include <stdint.h>
#include "flowcan/types.h"

typedef enum { TOF_OFF, TOF_BOOT_WAIT, TOF_INITIALIZING, TOF_CALIBRATING, TOF_RUNNING, TOF_RETRY_WAIT } tof_state_t;
typedef struct {
    tof_state_t state;
    uint32_t deadline_ms;
    uint32_t timeout_ms;
    uint8_t init_index;
    bool irq_pending;
} tof_t;

void tof_begin(tof_t *tof, uint32_t now_ms);
bool tof_step(tof_t *tof, uint32_t now_ms, range_sample_t *sample, bool *new_sample);
void tof_irq_notify(tof_t *tof);

#endif

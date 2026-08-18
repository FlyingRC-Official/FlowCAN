#ifndef FLOWCAN_PMW3901_H
#define FLOWCAN_PMW3901_H

#include <stdbool.h>
#include <stdint.h>
#include "flowcan/types.h"

typedef struct {
    int32_t accumulated_x;
    int32_t accumulated_y;
    uint32_t integration_start_us;
    uint8_t quality;
    bool initialized;
    uint8_t init_state;
    uint8_t init_index;
    uint32_t init_deadline_us;
} pmw3901_t;

typedef struct { int16_t dx; int16_t dy; uint8_t quality; bool motion; } pmw_motion_t;

void pmw3901_reset_state(pmw3901_t *dev, uint32_t now_us);
typedef enum { PMW_INIT_BUSY, PMW_INIT_READY, PMW_INIT_FAULT } pmw_init_result_t;
void pmw3901_start_init(pmw3901_t *dev, uint32_t now_us);
pmw_init_result_t pmw3901_init_step(pmw3901_t *dev, uint32_t now_us);
bool pmw3901_read_motion(pmw_motion_t *motion);
void pmw3901_accumulate(pmw3901_t *dev, const pmw_motion_t *motion);
void pmw3901_publish(pmw3901_t *dev, uint32_t now_us, flow_sample_t *sample);
void pmw3901_transform(int16_t raw_x, int16_t raw_y, int32_t *x, int32_t *y);

#endif

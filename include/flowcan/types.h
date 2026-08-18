#ifndef FLOWCAN_TYPES_H
#define FLOWCAN_TYPES_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t timestamp_us;
    uint32_t integration_us;
    int32_t count_x;
    int32_t count_y;
    float integral_x_rad;
    float integral_y_rad;
    uint8_t quality;
    bool valid;
} flow_sample_t;

typedef enum {
    RANGE_STATUS_VALID = 0,
    RANGE_STATUS_TOO_CLOSE,
    RANGE_STATUS_TOO_FAR,
    RANGE_STATUS_INVALID,
    RANGE_STATUS_FAULT
} range_status_t;

typedef struct {
    uint32_t timestamp_us;
    int32_t distance_mm;
    uint8_t quality;
    range_status_t status;
} range_sample_t;

enum {
    HEALTH_PMW_FAULT = 1U << 0,
    HEALTH_TOF_FAULT = 1U << 1,
    HEALTH_I2C_FAULT = 1U << 2,
    HEALTH_CAN_BUS_OFF = 1U << 3,
    HEALTH_CAN_RX_OVERFLOW = 1U << 4,
    HEALTH_CAN_TX_OVERFLOW = 1U << 5,
    HEALTH_SCHEDULER_LATE = 1U << 6,
    HEALTH_WATCHDOG_MISSED = 1U << 7,
    HEALTH_RTOS_STACK_LOW = 1U << 8,
    HEALTH_RTOS_FATAL = 1U << 9,
    HEALTH_UART_TX_OVERFLOW = 1U << 10
};

typedef uint16_t health_flags_t;

#endif

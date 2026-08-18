#ifndef FLOWCAN_DRONECAN_H
#define FLOWCAN_DRONECAN_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "flowcan/types.h"

typedef struct { uint8_t data[32]; uint16_t length; } dronecan_payload_t;

void dronecan_init(void);
void dronecan_poll(uint32_t now_ms, health_flags_t health);
bool dronecan_publish_flow(const flow_sample_t *sample);
bool dronecan_publish_range(const range_sample_t *sample);
bool dronecan_encode_flow(const flow_sample_t *sample, dronecan_payload_t *payload);
bool dronecan_encode_range(const range_sample_t *sample, dronecan_payload_t *payload);
bool dronecan_encode_node_status(uint32_t uptime_sec, health_flags_t health, dronecan_payload_t *payload);
uint8_t dronecan_range_reading_type(range_status_t status);

#endif

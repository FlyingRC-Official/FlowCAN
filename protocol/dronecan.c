#include "dronecan.h"
#include "config.h"
#include "flowcan/time_utils.h"
#include "platform.h"
#include "canard.h"
#include "com.hex.equipment.flow.Measurement.h"
#include "uavcan.equipment.range_sensor.Measurement.h"
#include "uavcan.protocol.GetNodeInfo.h"
#include "uavcan.protocol.GetNodeInfo_res.h"
#include "uavcan.protocol.NodeStatus.h"

#include <math.h>
#include <string.h>

static CanardInstance canard;
static _Alignas(CanardPoolAllocatorBlock) uint8_t arena[CANARD_ARENA_SIZE];
static uint8_t tid_status;
static uint8_t tid_flow;
static uint8_t tid_range;
static health_flags_t current_health;
static bool tx_overflow_pending;
static uint32_t clock_last_us;
static uint64_t clock_extended_us;
static uint64_t next_cleanup_us;
static uint32_t next_status_ms;

static uint64_t monotonic_us(void)
{
    const uint32_t now = platform_micros();
    clock_extended_us += (uint32_t)(now - clock_last_us);
    clock_last_us = now;
    return clock_extended_us;
}

static uint64_t extend_rx_timestamp(uint64_t raw_timestamp, uint64_t now_us)
{
    const uint32_t age = (uint32_t)(clock_last_us - (uint32_t)raw_timestamp);
    return age <= now_us ? now_us - age : 0U;
}

static uint8_t node_health(health_flags_t health)
{
    if ((health & (HEALTH_CAN_BUS_OFF | HEALTH_RTOS_FATAL)) != 0U) {
        return UAVCAN_PROTOCOL_NODESTATUS_HEALTH_CRITICAL;
    }
    if ((health & (HEALTH_PMW_FAULT | HEALTH_TOF_FAULT | HEALTH_I2C_FAULT |
                   HEALTH_RTOS_STACK_LOW)) != 0U) {
        return UAVCAN_PROTOCOL_NODESTATUS_HEALTH_ERROR;
    }
    return health != 0U ? UAVCAN_PROTOCOL_NODESTATUS_HEALTH_WARNING :
                          UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
}

uint8_t dronecan_range_reading_type(range_status_t status)
{
    if (status == RANGE_STATUS_VALID) return UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_VALID_RANGE;
    if (status == RANGE_STATUS_TOO_CLOSE) return UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_TOO_CLOSE;
    if (status == RANGE_STATUS_TOO_FAR) return UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_TOO_FAR;
    return UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_UNDEFINED;
}

bool dronecan_encode_flow(const flow_sample_t *sample, dronecan_payload_t *payload)
{
    struct com_hex_equipment_flow_Measurement message = {0};
    message.integration_interval = (float)sample->integration_us / 1000000.0f;
    message.rate_gyro_integral[0] = NAN;
    message.rate_gyro_integral[1] = NAN;
    message.flow_integral[0] = sample->integral_x_rad;
    message.flow_integral[1] = sample->integral_y_rad;
    message.quality = sample->valid ? sample->quality : 0U;
    payload->length = (uint16_t)com_hex_equipment_flow_Measurement_encode(&message, payload->data);
    return payload->length <= sizeof(payload->data);
}

bool dronecan_encode_range(const range_sample_t *sample, dronecan_payload_t *payload)
{
    struct uavcan_equipment_range_sensor_Measurement message = {0};
    message.timestamp.usec = 0U;
    message.sensor_id = 0U;
    message.beam_orientation_in_body_frame.orientation_defined =
        DRONECAN_RANGE_ORIENTATION_DEFINED != 0;
    message.beam_orientation_in_body_frame.fixed_axis_roll_pitch_yaw[0] = DRONECAN_RANGE_ROLL_COARSE;
    message.beam_orientation_in_body_frame.fixed_axis_roll_pitch_yaw[1] = DRONECAN_RANGE_PITCH_COARSE;
    message.beam_orientation_in_body_frame.fixed_axis_roll_pitch_yaw[2] = DRONECAN_RANGE_YAW_COARSE;
    message.field_of_view = 0.4712f;
    message.sensor_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SENSOR_TYPE_LIDAR;
    message.reading_type = dronecan_range_reading_type(sample->status);
    if (sample->status == RANGE_STATUS_VALID) {
        message.range = (float)sample->distance_mm / 1000.0f;
    } else if (sample->status == RANGE_STATUS_TOO_CLOSE) {
        message.range = (float)RANGE_MIN_MM / 1000.0f;
    } else if (sample->status == RANGE_STATUS_TOO_FAR) {
        message.range = (float)RANGE_MAX_MM / 1000.0f;
    } else {
        message.range = 0.0f;
    }
    payload->length = (uint16_t)uavcan_equipment_range_sensor_Measurement_encode(&message, payload->data);
    return payload->length <= sizeof(payload->data);
}

bool dronecan_encode_node_status(uint32_t uptime, health_flags_t health, dronecan_payload_t *payload)
{
    struct uavcan_protocol_NodeStatus message = {
        .uptime_sec = uptime,
        .health = node_health(health),
        .mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL,
        .sub_mode = 0U,
        .vendor_specific_status_code = health
    };
    payload->length = (uint16_t)uavcan_protocol_NodeStatus_encode(&message, payload->data);
    return payload->length <= sizeof(payload->data);
}

static bool should_accept(const CanardInstance *instance,
                          uint64_t *signature,
                          uint16_t data_type_id,
                          CanardTransferType transfer_type,
                          uint8_t source_node_id)
{
    (void)instance;
    (void)source_node_id;
    if (transfer_type == CanardTransferTypeRequest && data_type_id == UAVCAN_PROTOCOL_GETNODEINFO_ID) {
        *signature = UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE;
        return true;
    }
    return false;
}

static void fill_status(struct uavcan_protocol_NodeStatus *status)
{
    status->uptime_sec = platform_millis() / 1000U;
    status->health = node_health(current_health);
    status->mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    status->sub_mode = 0U;
    status->vendor_specific_status_code = current_health;
}

static void on_reception(CanardInstance *instance, CanardRxTransfer *transfer)
{
    if (transfer->transfer_type != CanardTransferTypeRequest ||
        transfer->data_type_id != UAVCAN_PROTOCOL_GETNODEINFO_ID) {
        return;
    }

    struct uavcan_protocol_GetNodeInfoResponse message = {0};
    fill_status(&message.status);
    message.software_version.major = FW_VERSION_MAJOR;
    message.software_version.minor = FW_VERSION_MINOR;
    message.hardware_version.major = 1U;
    message.hardware_version.minor = 0U;
    const uint8_t *uid = (const uint8_t *)0x1FFFF7E8UL;
    memcpy(message.hardware_version.unique_id, uid, 12U);
    const char name[] = DRONECAN_NODE_NAME;
    message.name.len = (uint8_t)(sizeof(name) - 1U);
    if (message.name.len > sizeof(message.name.data)) {
        message.name.len = sizeof(message.name.data);
    }
    memcpy(message.name.data, name, message.name.len);

    uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];
    const uint16_t length = (uint16_t)uavcan_protocol_GetNodeInfoResponse_encode(&message, buffer);
    const uint8_t source_node_id = transfer->source_node_id;
    uint8_t transfer_id = transfer->transfer_id;
    canardReleaseRxTransferPayload(instance, transfer);
    const int16_t result = canardRequestOrRespond(instance,
                                                  source_node_id,
                                                  UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,
                                                  UAVCAN_PROTOCOL_GETNODEINFO_ID,
                                                  &transfer_id,
                                                  CANARD_TRANSFER_PRIORITY_HIGH,
                                                  CanardResponse,
                                                  buffer,
                                                  length,
                                                  monotonic_us() + CAN_HIGH_PRIORITY_TX_DEADLINE_US);
    if (result < 0) {
        tx_overflow_pending = true;
    }
}

void dronecan_init(void)
{
    memset(arena, 0, sizeof(arena));
    tid_status = 0U;
    tid_flow = 0U;
    tid_range = 0U;
    current_health = 0U;
    tx_overflow_pending = false;
    clock_last_us = platform_micros();
    clock_extended_us = 0U;
    next_cleanup_us = 0U;
    next_status_ms = 0U;
    canardInit(&canard, arena, sizeof(arena), on_reception, should_accept, NULL);
    canardSetLocalNodeID(&canard, CAN_NODE_ID);
}

static bool broadcast(uint64_t signature,
                      uint16_t data_type_id,
                      uint8_t *transfer_id,
                      uint8_t priority,
                      const dronecan_payload_t *payload)
{
    if (priority >= CANARD_TRANSFER_PRIORITY_LOW) {
        const CanardPoolAllocatorStatistics stats = canardGetPoolAllocatorStatistics(&canard);
        const uint16_t free_blocks = (uint16_t)(stats.capacity_blocks - stats.current_usage_blocks);
        if (free_blocks <= CANARD_HIGH_PRIORITY_RESERVE_BLOCKS) {
            tx_overflow_pending = true;
            return false;
        }
    }
    const uint64_t timeout = priority <= CANARD_TRANSFER_PRIORITY_HIGH ?
                             CAN_HIGH_PRIORITY_TX_DEADLINE_US : CAN_TX_DEADLINE_US;
    const int16_t result = canardBroadcast(&canard,
                                           signature,
                                           data_type_id,
                                           transfer_id,
                                           priority,
                                           payload->data,
                                           payload->length,
                                           monotonic_us() + timeout);
    if (result < 0) {
        tx_overflow_pending = true;
        return false;
    }
    return true;
}

bool dronecan_publish_flow(const flow_sample_t *sample)
{
    dronecan_payload_t payload;
    return dronecan_encode_flow(sample, &payload) &&
           broadcast(COM_HEX_EQUIPMENT_FLOW_MEASUREMENT_SIGNATURE,
                     COM_HEX_EQUIPMENT_FLOW_MEASUREMENT_ID,
                     &tid_flow,
                     CANARD_TRANSFER_PRIORITY_LOW,
                     &payload);
}

bool dronecan_publish_range(const range_sample_t *sample)
{
    dronecan_payload_t payload;
    return dronecan_encode_range(sample, &payload) &&
           broadcast(UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SIGNATURE,
                     UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_ID,
                     &tid_range,
                     CANARD_TRANSFER_PRIORITY_LOW,
                     &payload);
}

bool dronecan_take_tx_overflow(void)
{
    const bool pending = tx_overflow_pending;
    tx_overflow_pending = false;
    return pending;
}

void dronecan_poll(uint32_t now_ms, health_flags_t health)
{
    current_health = health;
    const uint64_t now_us = monotonic_us();
    if (now_us >= next_cleanup_us) {
        canardCleanupStaleTransfers(&canard, now_us);
        next_cleanup_us = now_us + CANARD_CLEANUP_INTERVAL_US;
    }

    for (uint32_t count = 0U; count < DRONECAN_RX_BUDGET_FRAMES; count++) {
        CanardCANFrame frame;
        uint64_t timestamp;
        if (!platform_can_rx_pop(&frame, &timestamp)) {
            break;
        }
        (void)canardHandleRxFrame(&canard, &frame, extend_rx_timestamp(timestamp, now_us));
    }

    if (platform_can_bus_off()) {
        while (canardPeekTxQueue(&canard) != NULL) {
            canardPopTxQueue(&canard);
        }
        return;
    }

    if (time_reached_u32(now_ms, next_status_ms)) {
        dronecan_payload_t payload;
        if (!dronecan_encode_node_status(now_ms / 1000U, health, &payload) ||
            !broadcast(UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
                       UAVCAN_PROTOCOL_NODESTATUS_ID,
                       &tid_status,
                       CANARD_TRANSFER_PRIORITY_HIGH,
                       &payload)) {
            tx_overflow_pending = true;
        }
        next_status_ms = now_ms + 1000U / DRONECAN_NODE_STATUS_HZ;
    }

    for (uint32_t count = 0U; count < DRONECAN_TX_BUDGET_FRAMES; count++) {
        CanardCANFrame *frame = canardPeekTxQueue(&canard);
        if (frame == NULL || !platform_can_tx(frame)) {
            break;
        }
        canardPopTxQueue(&canard);
    }
}

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
static uint8_t tid_status,tid_flow,tid_range;
static health_flags_t current_health;
static bool tx_overflow_pending;
static uint32_t clock_last_us,next_status_ms;
static uint64_t clock_extended_us,next_cleanup_us;

static uint64_t monotonic_us(void){const uint32_t now=platform_micros();clock_extended_us+=(uint32_t)(now-clock_last_us);clock_last_us=now;return clock_extended_us;}
static uint64_t extend_rx_timestamp(uint64_t raw,uint64_t now){const uint32_t age=(uint32_t)(clock_last_us-(uint32_t)raw);return age<=now?now-age:0U;}
static uint8_t node_health(health_flags_t h){if((h&(HEALTH_CAN_BUS_OFF|HEALTH_RTOS_FATAL))!=0U)return UAVCAN_PROTOCOL_NODESTATUS_HEALTH_CRITICAL;if((h&(HEALTH_PMW_FAULT|HEALTH_TOF_FAULT|HEALTH_I2C_FAULT|HEALTH_RTOS_STACK_LOW))!=0U)return UAVCAN_PROTOCOL_NODESTATUS_HEALTH_ERROR;return h!=0U?UAVCAN_PROTOCOL_NODESTATUS_HEALTH_WARNING:UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;}

uint8_t dronecan_range_reading_type(range_status_t s){if(s==RANGE_STATUS_VALID)return UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_VALID_RANGE;if(s==RANGE_STATUS_TOO_CLOSE)return UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_TOO_CLOSE;if(s==RANGE_STATUS_TOO_FAR)return UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_TOO_FAR;return UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_UNDEFINED;}

bool dronecan_encode_flow(const flow_sample_t *s,dronecan_payload_t *p)
{
    struct com_hex_equipment_flow_Measurement m={0};m.integration_interval=(float)s->integration_us/1000000.0f;m.rate_gyro_integral[0]=NAN;m.rate_gyro_integral[1]=NAN;m.flow_integral[0]=s->integral_x_rad;m.flow_integral[1]=s->integral_y_rad;m.quality=s->valid?s->quality:0U;p->length=(uint16_t)com_hex_equipment_flow_Measurement_encode(&m,p->data);return p->length<=sizeof(p->data);
}

bool dronecan_encode_range(const range_sample_t *s,dronecan_payload_t *p)
{
    struct uavcan_equipment_range_sensor_Measurement m={0};m.timestamp.usec=0U;m.sensor_id=0U;m.beam_orientation_in_body_frame.orientation_defined=DRONECAN_RANGE_ORIENTATION_DEFINED!=0;m.beam_orientation_in_body_frame.fixed_axis_roll_pitch_yaw[0]=DRONECAN_RANGE_ROLL_COARSE;m.beam_orientation_in_body_frame.fixed_axis_roll_pitch_yaw[1]=DRONECAN_RANGE_PITCH_COARSE;m.beam_orientation_in_body_frame.fixed_axis_roll_pitch_yaw[2]=DRONECAN_RANGE_YAW_COARSE;m.field_of_view=0.4712f;m.sensor_type=UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SENSOR_TYPE_LIDAR;m.reading_type=dronecan_range_reading_type(s->status);
    if(s->status==RANGE_STATUS_VALID)m.range=(float)s->distance_mm/1000.0f;else if(s->status==RANGE_STATUS_TOO_CLOSE)m.range=(float)RANGE_MIN_MM/1000.0f;else if(s->status==RANGE_STATUS_TOO_FAR)m.range=(float)RANGE_MAX_MM/1000.0f;else m.range=0.0f;
    p->length=(uint16_t)uavcan_equipment_range_sensor_Measurement_encode(&m,p->data);return p->length<=sizeof(p->data);
}

bool dronecan_encode_node_status(uint32_t uptime,health_flags_t h,dronecan_payload_t *p){struct uavcan_protocol_NodeStatus m={.uptime_sec=uptime,.health=node_health(h),.mode=UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL,.sub_mode=0U,.vendor_specific_status_code=h};p->length=(uint16_t)uavcan_protocol_NodeStatus_encode(&m,p->data);return p->length<=sizeof(p->data);}

static bool should_accept(const CanardInstance *ins,uint64_t *sig,uint16_t id,CanardTransferType type,uint8_t source){(void)ins;(void)source;if(type==CanardTransferTypeRequest&&id==UAVCAN_PROTOCOL_GETNODEINFO_ID){*sig=UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE;return true;}return false;}
static void fill_status(struct uavcan_protocol_NodeStatus *s){s->uptime_sec=platform_millis()/1000U;s->health=node_health(current_health);s->mode=UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;s->sub_mode=0U;s->vendor_specific_status_code=current_health;}

static void on_reception(CanardInstance *ins,CanardRxTransfer *tr)
{
    if(tr->transfer_type!=CanardTransferTypeRequest||tr->data_type_id!=UAVCAN_PROTOCOL_GETNODEINFO_ID)return;
    struct uavcan_protocol_GetNodeInfoResponse m={0};fill_status(&m.status);m.software_version.major=FW_VERSION_MAJOR;m.software_version.minor=FW_VERSION_MINOR;m.hardware_version.major=1U;m.hardware_version.minor=0U;const uint8_t *uid=(const uint8_t *)0x1FFFF7E8UL;memcpy(m.hardware_version.unique_id,uid,12U);const char name[]=DRONECAN_NODE_NAME;m.name.len=(uint8_t)(sizeof(name)-1U);if(m.name.len>sizeof(m.name.data))m.name.len=sizeof(m.name.data);memcpy(m.name.data,name,m.name.len);
    uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];const uint16_t len=(uint16_t)uavcan_protocol_GetNodeInfoResponse_encode(&m,buffer);const uint8_t source=tr->source_node_id;uint8_t transfer_id=tr->transfer_id;canardReleaseRxTransferPayload(ins,tr);const int16_t result=canardRequestOrRespond(ins,source,UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,UAVCAN_PROTOCOL_GETNODEINFO_ID,&transfer_id,CANARD_TRANSFER_PRIORITY_HIGH,CanardResponse,buffer,len,monotonic_us()+CAN_HIGH_PRIORITY_TX_DEADLINE_US);if(result<0)tx_overflow_pending=true;
}

void dronecan_init(void){memset(arena,0,sizeof(arena));tid_status=0U;tid_flow=0U;tid_range=0U;current_health=0U;tx_overflow_pending=false;clock_last_us=platform_micros();clock_extended_us=0U;next_cleanup_us=0U;next_status_ms=0U;canardInit(&canard,arena,sizeof(arena),on_reception,should_accept,NULL);canardSetLocalNodeID(&canard,CAN_NODE_ID);}

static bool broadcast(uint64_t sig,uint16_t id,uint8_t *tid,uint8_t priority,const dronecan_payload_t *p)
{
    if(priority>=CANARD_TRANSFER_PRIORITY_LOW){const CanardPoolAllocatorStatistics stats=canardGetPoolAllocatorStatistics(&canard);const uint16_t free_blocks=(uint16_t)(stats.capacity_blocks-stats.current_usage_blocks);if(free_blocks<=CANARD_HIGH_PRIORITY_RESERVE_BLOCKS){tx_overflow_pending=true;return false;}}
    const uint64_t timeout=priority<=CANARD_TRANSFER_PRIORITY_HIGH?CAN_HIGH_PRIORITY_TX_DEADLINE_US:CAN_TX_DEADLINE_US;const int16_t result=canardBroadcast(&canard,sig,id,tid,priority,p->data,p->length,monotonic_us()+timeout);if(result<0){tx_overflow_pending=true;return false;}return true;
}

bool dronecan_publish_flow(const flow_sample_t *s){dronecan_payload_t p;return dronecan_encode_flow(s,&p)&&broadcast(COM_HEX_EQUIPMENT_FLOW_MEASUREMENT_SIGNATURE,COM_HEX_EQUIPMENT_FLOW_MEASUREMENT_ID,&tid_flow,CANARD_TRANSFER_PRIORITY_LOW,&p);}
bool dronecan_publish_range(const range_sample_t *s){dronecan_payload_t p;return dronecan_encode_range(s,&p)&&broadcast(UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SIGNATURE,UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_ID,&tid_range,CANARD_TRANSFER_PRIORITY_LOW,&p);}
bool dronecan_take_tx_overflow(void){const bool pending=tx_overflow_pending;tx_overflow_pending=false;return pending;}

void dronecan_poll(uint32_t now_ms,health_flags_t health)
{
    current_health=health;const uint64_t now_us=monotonic_us();if(now_us>=next_cleanup_us){canardCleanupStaleTransfers(&canard,now_us);next_cleanup_us=now_us+CANARD_CLEANUP_INTERVAL_US;}
    for(uint32_t count=0U;count<DRONECAN_RX_BUDGET_FRAMES;count++){CanardCANFrame frame;uint64_t timestamp;if(!platform_can_rx_pop(&frame,&timestamp))break;(void)canardHandleRxFrame(&canard,&frame,extend_rx_timestamp(timestamp,now_us));}
    if(platform_can_bus_off()){while(canardPeekTxQueue(&canard)!=NULL)canardPopTxQueue(&canard);return;}
    if(time_reached_u32(now_ms,next_status_ms)){dronecan_payload_t p;if(!dronecan_encode_node_status(now_ms/1000U,health,&p)||!broadcast(UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,UAVCAN_PROTOCOL_NODESTATUS_ID,&tid_status,CANARD_TRANSFER_PRIORITY_HIGH,&p))tx_overflow_pending=true;next_status_ms=now_ms+1000U/DRONECAN_NODE_STATUS_HZ;}
    for(uint32_t count=0U;count<DRONECAN_TX_BUDGET_FRAMES;count++){CanardCANFrame *frame=canardPeekTxQueue(&canard);if(frame==NULL||!platform_can_tx(frame))break;canardPopTxQueue(&canard);}
}

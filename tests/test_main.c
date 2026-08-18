#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "flowcan/time_utils.h"
#include "flowcan/ring.h"
#include "msp.h"
#include "dronecan.h"
#include "pmw3901.h"
#include "scheduler.h"
#include "supervisor.h"
#include "activity.h"
#include "can_recovery.h"
#include "vl53l1x.h"
#include "VL53L1X_api.h"
#include "com.hex.equipment.flow.Measurement.h"
#include "uavcan.equipment.range_sensor.Measurement.h"
#include "uavcan.protocol.NodeStatus.h"

static int32_t get_i32(const uint8_t *p){return(int32_t)((uint32_t)p[0]|(uint32_t)p[1]<<8U|(uint32_t)p[2]<<16U|(uint32_t)p[3]<<24U);}
static CanardRxTransfer transfer_from(const uint8_t *p,uint16_t n){CanardRxTransfer t={0};t.payload_head=p;t.payload_len=n;return t;}

static void test_msp(void)
{
    flow_sample_t f={.count_x=0x1020304,.count_y=-2,.quality=77,.valid=true};uint8_t b[32];size_t n=msp_encode_flow(&f,b,sizeof(b));assert(n==18U);assert(memcmp(b,"$X<",3U)==0);assert(b[4]==0x02&&b[5]==0x1F&&b[6]==9&&b[7]==0);assert(b[8]==77&&get_i32(&b[9])==f.count_x&&get_i32(&b[13])==-2);
    msp_parser_t p;msp_parser_init(&p);msp_frame_t out;bool done=false;for(size_t i=0;i<n;i++)done=msp_parser_consume(&p,b[i],&out)||done;assert(done&&out.command==MSP2_SENSOR_OPTIC_FLOW&&out.payload_length==9U);b[n-1]^=1U;msp_parser_init(&p);done=false;for(size_t i=0;i<n;i++)done=msp_parser_consume(&p,b[i],&out)||done;assert(!done);
    msp_parser_init(&p);done=false;for(size_t i=0;i<n/2U;i++)done=msp_parser_consume(&p,b[i],&out)||done;assert(!done);
    range_sample_t r={.distance_mm=1234,.quality=88,.status=RANGE_STATUS_VALID};n=msp_encode_range(&r,b,sizeof(b));assert(n==14U&&get_i32(&b[9])==1234);r.status=RANGE_STATUS_FAULT;n=msp_encode_range(&r,b,sizeof(b));assert(b[8]==0U&&get_i32(&b[9])==-1);
}

static void test_dronecan(void)
{
    flow_sample_t f={.integration_us=25000,.integral_x_rad=0.1f,.integral_y_rad=-0.2f,.quality=200,.valid=true};dronecan_payload_t p;assert(dronecan_encode_flow(&f,&p));CanardRxTransfer t=transfer_from(p.data,p.length);struct com_hex_equipment_flow_Measurement fm;assert(!com_hex_equipment_flow_Measurement_decode(&t,&fm));assert(fabsf(fm.integration_interval-0.025f)<0.0001f&&isnan(fm.rate_gyro_integral[0])&&isnan(fm.rate_gyro_integral[1])&&fm.quality==200U);
    range_sample_t r={.distance_mm=1500,.status=RANGE_STATUS_VALID};assert(dronecan_encode_range(&r,&p));t=transfer_from(p.data,p.length);struct uavcan_equipment_range_sensor_Measurement rm;assert(!uavcan_equipment_range_sensor_Measurement_decode(&t,&rm));assert(rm.reading_type==1U&&fabsf(rm.range-1.5f)<0.01f&&rm.timestamp.usec==0U);
    assert(dronecan_range_reading_type(RANGE_STATUS_TOO_CLOSE)==2U&&dronecan_range_reading_type(RANGE_STATUS_TOO_FAR)==3U&&dronecan_range_reading_type(RANGE_STATUS_FAULT)==0U);
    assert(dronecan_encode_node_status(7U,HEALTH_CAN_BUS_OFF,&p));t=transfer_from(p.data,p.length);struct uavcan_protocol_NodeStatus ns;assert(!uavcan_protocol_NodeStatus_decode(&t,&ns));assert(ns.uptime_sec==7U&&ns.health==3U&&ns.vendor_specific_status_code==HEALTH_CAN_BUS_OFF);
    assert(dronecan_encode_node_status(8U,HEALTH_RTOS_STACK_LOW,&p));t=transfer_from(p.data,p.length);assert(!uavcan_protocol_NodeStatus_decode(&t,&ns));assert(ns.health==2U&&ns.vendor_specific_status_code==HEALTH_RTOS_STACK_LOW);
    dronecan_init();bool saturated=false;for(unsigned i=0;i<100U;i++)if(!dronecan_publish_flow(&f)){saturated=true;break;}assert(saturated);dronecan_poll(1000U,HEALTH_CAN_TX_OVERFLOW);assert(dronecan_publish_flow(&f));dronecan_poll(1001U,0U);
}

static void test_math_and_time(void)
{
    int32_t x,y;pmw3901_transform(32767,-32768,&x,&y);assert(x==32767*FLOW_SIGN_X&&y==-32768*FLOW_SIGN_Y);
    pmw3901_t p;pmw3901_reset_state(&p,0xFFFFFFF0U);p.initialized=true;pmw_motion_t m={.dx=100,.dy=-50,.quality=99,.motion=true};pmw3901_accumulate(&p,&m);flow_sample_t s;pmw3901_publish(&p,0x10U,&s);assert(s.integration_us==32U&&s.count_x==100&&s.count_y==-50&&p.accumulated_x==0);
    p.accumulated_x=INT32_MAX-1;p.accumulated_y=INT32_MIN+1;m.dx=100;m.dy=-100;pmw3901_accumulate(&p,&m);assert(p.accumulated_x==INT32_MAX&&p.accumulated_y==INT32_MIN);
    periodic_task_t task;periodic_task_init(&task,0xFFFFFFF0U,20U);assert(!periodic_task_due(&task,0U));assert(periodic_task_due(&task,4U));assert(!periodic_task_due(&task,5U));assert(elapsed_u32(1U,0xFFFFFFFFU)==2U);
    assert(ring_can_write(256U,0U,0U,255U));assert(!ring_can_write(256U,0U,0U,256U));assert(ring_used(256U,2U,250U)==8U);assert(ring_can_write(256U,2U,250U,247U));assert(!ring_can_write(256U,2U,250U,248U));
    const uint32_t good_watermarks[]={64U,48U,32U};const uint32_t low_watermarks[]={64U,31U,80U};assert(supervisor_should_feed(0x07U,0x07U,false));assert(!supervisor_should_feed(0x03U,0x07U,false));assert(!supervisor_should_feed(0x07U,0x07U,true));assert(!supervisor_stack_low(good_watermarks,3U,32U));assert(supervisor_stack_low(low_watermarks,3U,32U));
}

static bool led_state[4];static bool tof_enabled;static int tof_init_error;static VL53L1X_Result_t tof_result;
static void test_state_machines(void)
{
    activity_init();activity_pulse(LED_PMW,100U);assert(led_state[LED_PMW]);activity_step(129U);assert(led_state[LED_PMW]);activity_step(130U);assert(!led_state[LED_PMW]);
    can_recovery_t c;can_recovery_init(&c);assert(!can_recovery_step(&c,true,0xFFFFFF00U,1000U));assert(!can_recovery_step(&c,true,500U,1000U));assert(can_recovery_step(&c,true,744U,1000U));assert(!can_recovery_step(&c,false,745U,1000U));
    tof_t tof;range_sample_t sample;bool fresh;tof_init_error=-1;tof_begin(&tof,100U);assert(!tof_enabled);assert(!tof_step(&tof,102U,&sample,&fresh)&&tof_enabled);assert(!tof_step(&tof,105U,&sample,&fresh)&&fresh&&sample.status==RANGE_STATUS_FAULT);assert(!tof_step(&tof,1104U,&sample,&fresh));assert(!tof_step(&tof,1105U,&sample,&fresh)&&tof_enabled);tof_init_error=0;for(unsigned i=0;i<91U;i++)assert(!tof_step(&tof,1108U,&sample,&fresh));assert(tof_step(&tof,1109U,&sample,&fresh));tof_result.Status=0U;tof_result.Distance=1000U;tof_irq_notify(&tof);assert(tof_step(&tof,1110U,&sample,&fresh)&&fresh&&sample.status==RANGE_STATUS_VALID&&sample.distance_mm==1000);
}
int main(void){test_msp();test_dronecan();test_math_and_time();test_state_machines();puts("FlowCAN host tests: PASS");return 0;}

/* Hardware symbols referenced by drivers/dronecan are not executed in host tests. */
uint32_t platform_millis(void){return 0U;}uint32_t platform_micros(void){return 0U;}void platform_delay_us(uint32_t us){(void)us;}void platform_pmw_cs(bool a){(void)a;}void platform_pmw_reset(bool a){(void)a;}uint8_t platform_spi_transfer(uint8_t v){return v;}
bool platform_can_tx(const CanardCANFrame *f){(void)f;return true;}bool platform_can_rx_pop(CanardCANFrame *f,uint64_t *t){(void)f;(void)t;return false;}bool platform_can_bus_off(void){return false;}
void platform_activity_led(activity_led_t led,bool on){led_state[(unsigned)led]=on;}void platform_tof_xshut(bool enabled){tof_enabled=enabled;}
const uint8_t VL51L1X_DEFAULT_CONFIGURATION[91]={0};
int8_t VL53L1_WrByte(uint16_t d,uint16_t r,uint8_t v){(void)d;(void)r;(void)v;return(int8_t)tof_init_error;}VL53L1X_ERROR VL53L1X_SetDistanceMode(uint16_t d,uint16_t v){(void)d;(void)v;return 0;}VL53L1X_ERROR VL53L1X_SetTimingBudgetInMs(uint16_t d,uint16_t v){(void)d;(void)v;return 0;}VL53L1X_ERROR VL53L1X_SetInterMeasurementInMs(uint16_t d,uint32_t v){(void)d;(void)v;return 0;}VL53L1X_ERROR VL53L1X_StartRanging(uint16_t d){(void)d;return 0;}VL53L1X_ERROR VL53L1X_StopRanging(uint16_t d){(void)d;return 0;}VL53L1X_ERROR VL53L1X_CheckForDataReady(uint16_t d,uint8_t *ready){(void)d;*ready=1U;return 0;}VL53L1X_ERROR VL53L1X_GetResult(uint16_t d,VL53L1X_Result_t *r){(void)d;*r=tof_result;return 0;}VL53L1X_ERROR VL53L1X_ClearInterrupt(uint16_t d){(void)d;return 0;}

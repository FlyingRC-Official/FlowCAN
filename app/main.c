#include "activity.h"
#include "can_recovery.h"
#include "scheduler.h"
#include "status_led.h"
#include "config.h"
#include "flowcan/time_utils.h"
#include "flowcan/types.h"
#include "platform.h"
#include "pmw3901.h"
#include "vl53l1x.h"
#include "dronecan.h"
#include "msp.h"

int main(void)
{
    platform_init();activity_init();dronecan_init();
    const uint32_t start=platform_millis();pmw3901_t pmw;pmw3901_reset_state(&pmw,platform_micros());tof_t tof;tof_begin(&tof,start);
    health_flags_t health=0U;pmw3901_start_init(&pmw,platform_micros());
    flow_sample_t flow={0};range_sample_t range={.distance_mm=-1,.status=RANGE_STATUS_INVALID};msp_parser_t parser;msp_parser_init(&parser);
    periodic_task_t flow_pub,range_pub;periodic_task_init(&flow_pub,start,1000U/MSP_FLOW_HZ);periodic_task_init(&range_pub,start,1000U/MSP_RANGE_HZ);
    uint32_t pmw_retry=start+SENSOR_RETRY_MS;can_recovery_t can_recovery;can_recovery_init(&can_recovery);uint8_t frame[32];
    for(;;){uint32_t now=platform_millis();bool core_ok=true;
        if(platform_take_pmw_irq()&&pmw.initialized){pmw_motion_t motion;if(pmw3901_read_motion(&motion)){pmw3901_accumulate(&pmw,&motion);activity_pulse(LED_PMW,now);}else{health|=HEALTH_PMW_FAULT;pmw.initialized=false;}}
        if(!pmw.initialized&&time_reached_u32(now,pmw_retry)){pmw3901_start_init(&pmw,platform_micros());pmw_retry=now+SENSOR_RETRY_MS;}
        if(!pmw.initialized&&pmw.init_state!=0U){pmw_init_result_t init_result=pmw3901_init_step(&pmw,platform_micros());if(init_result==PMW_INIT_READY)health&=(health_flags_t)~HEALTH_PMW_FAULT;else if(init_result==PMW_INIT_FAULT){health|=HEALTH_PMW_FAULT;pmw.init_state=0U;pmw_retry=now+SENSOR_RETRY_MS;}}
        if(platform_take_tof_irq()){tof_irq_notify(&tof);activity_pulse(LED_TOF,now);}bool fresh=false;if(tof_step(&tof,now,&range,&fresh))health&=(health_flags_t)~(HEALTH_TOF_FAULT|HEALTH_I2C_FAULT);else health|=(HEALTH_TOF_FAULT|HEALTH_I2C_FAULT);
        const bool bus_off=platform_can_bus_off();
        if(periodic_task_due(&flow_pub,now)){pmw3901_publish(&pmw,platform_micros(),&flow);size_t n=msp_encode_flow(&flow,frame,sizeof(frame));if(n==0U||!platform_uart_tx(frame,n))core_ok=false;if(!bus_off&&!dronecan_publish_flow(&flow))health|=HEALTH_CAN_TX_OVERFLOW;}
        if(periodic_task_due(&range_pub,now)){size_t n=msp_encode_range(&range,frame,sizeof(frame));if(n==0U||!platform_uart_tx(frame,n))core_ok=false;if(!bus_off&&!dronecan_publish_range(&range))health|=HEALTH_CAN_TX_OVERFLOW;}
        uint8_t b;msp_frame_t received;while(platform_uart_rx_pop(&b)){activity_pulse(LED_MSP_RX,now);(void)msp_parser_consume(&parser,b,&received);}
        if(platform_take_can_rx_activity())activity_pulse(LED_CAN_RX,now);
        if(platform_take_can_rx_overflow())health|=HEALTH_CAN_RX_OVERFLOW;
        if(bus_off)health|=HEALTH_CAN_BUS_OFF;else health&=(health_flags_t)~HEALTH_CAN_BUS_OFF;
        if(can_recovery_step(&can_recovery,bus_off,now,CAN_RECOVERY_MS))platform_can_reinit();
        dronecan_poll(now,health);activity_step(now);status_led_step(now,health,(tof.state!=TOF_RUNNING)||!pmw.initialized);
        if(core_ok)platform_watchdog_feed();else health|=HEALTH_WATCHDOG_MISSED;
    }
}

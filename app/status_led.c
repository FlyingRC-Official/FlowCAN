#include "status_led.h"
#include "platform.h"
#include "flowcan/time_utils.h"

typedef struct{uint8_t r,g,b;uint16_t on_ms,period_ms;}pattern_t;
static pattern_t select_pattern(health_flags_t h,bool init)
{
    if(init)return(pattern_t){0,0,48,1000,1000};
    if(h&HEALTH_CAN_BUS_OFF)return(pattern_t){48,0,48,100,500};
    if((h&(HEALTH_PMW_FAULT|HEALTH_TOF_FAULT))==(HEALTH_PMW_FAULT|HEALTH_TOF_FAULT))return(pattern_t){64,0,0,100,250};
    if(h&HEALTH_PMW_FAULT)return(pattern_t){64,0,0,200,1000};
    if(h&HEALTH_TOF_FAULT)return(pattern_t){48,32,0,200,1000};
    return(pattern_t){0,48,0,1000,1000};
}
void status_led_step(uint32_t now,health_flags_t h,bool init)
{
    static uint32_t next;static uint8_t last_r=255,last_g=255,last_b=255;pattern_t p=select_pattern(h,init);bool on=(now%p.period_ms)<p.on_ms;uint8_t r=on?p.r:0,g=on?p.g:0,b=on?p.b:0;
    if((r!=last_r||g!=last_g||b!=last_b)&&time_reached_u32(now,next)&&!platform_ws2812_busy()){if(platform_ws2812_start(r,g,b)){last_r=r;last_g=g;last_b=b;next=now+2U;}}
}

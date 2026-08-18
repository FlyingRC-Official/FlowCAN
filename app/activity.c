#include "activity.h"
#include "config.h"
#include "flowcan/time_utils.h"
static uint32_t off_at[4];
void activity_init(void){for(unsigned i=0;i<4U;i++){off_at[i]=0U;platform_activity_led((activity_led_t)i,false);}}
void activity_pulse(activity_led_t led,uint32_t now){platform_activity_led(led,true);off_at[(unsigned)led]=now+ACTIVITY_LED_HOLD_MS;}
void activity_step(uint32_t now){for(unsigned i=0;i<4U;i++)if(off_at[i]!=0U&&time_reached_u32(now,off_at[i])){platform_activity_led((activity_led_t)i,false);off_at[i]=0U;}}

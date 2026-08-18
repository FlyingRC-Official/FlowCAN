#include "can_recovery.h"
#include "flowcan/time_utils.h"
void can_recovery_init(can_recovery_t *s){s->active=false;s->retry_at_ms=0U;}
bool can_recovery_step(can_recovery_t *s,bool bus_off,uint32_t now,uint32_t retry)
{
    if(!bus_off){s->active=false;s->retry_at_ms=0U;return false;}
    if(!s->active){s->active=true;s->retry_at_ms=now+retry;return false;}
    if(time_reached_u32(now,s->retry_at_ms)){s->retry_at_ms=now+retry;return true;}
    return false;
}

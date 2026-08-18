#include "scheduler.h"
#include "flowcan/time_utils.h"
void periodic_task_init(periodic_task_t *t,uint32_t now,uint32_t period){t->period=period;t->deadline=now+period;}
bool periodic_task_due(periodic_task_t *t,uint32_t now){if(!time_reached_u32(now,t->deadline))return false;do{t->deadline+=t->period;}while(time_reached_u32(now,t->deadline));return true;}

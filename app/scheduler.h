#ifndef FLOWCAN_SCHEDULER_H
#define FLOWCAN_SCHEDULER_H
#include <stdbool.h>
#include <stdint.h>
typedef struct{uint32_t deadline;uint32_t period;}periodic_task_t;
void periodic_task_init(periodic_task_t *task,uint32_t now,uint32_t period);
bool periodic_task_due(periodic_task_t *task,uint32_t now);
#endif

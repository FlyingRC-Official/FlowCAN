#ifndef FLOWCAN_FREERTOS_CONFIG_H
#define FLOWCAN_FREERTOS_CONFIG_H

#include <stdint.h>

extern void flowcan_rtos_assert_failed(const char *file, int line);

#define configUSE_PREEMPTION                    1
#define configUSE_TIME_SLICING                  1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#define configUSE_TICKLESS_IDLE                 0
#define configCPU_CLOCK_HZ                      144000000UL
#define configTICK_RATE_HZ                      1000U
#define configMAX_PRIORITIES                    5U
#define configMINIMAL_STACK_SIZE                128U
#define configMAX_TASK_NAME_LEN                 16U
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_16_BIT_TICKS                  0

#define configSUPPORT_STATIC_ALLOCATION         1
#define configSUPPORT_DYNAMIC_ALLOCATION        0
#define configAPPLICATION_ALLOCATED_HEAP        0
#define configUSE_NEWLIB_REENTRANT              0

#define configUSE_TASK_NOTIFICATIONS            1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES   1U
#define configUSE_MUTEXES                       0
#define configUSE_RECURSIVE_MUTEXES             0
#define configUSE_COUNTING_SEMAPHORES           0
#define configQUEUE_REGISTRY_SIZE               0U
#define configUSE_QUEUE_SETS                    0
#define configUSE_TIMERS                        0
#define configUSE_CO_ROUTINES                   0
#define configMAX_CO_ROUTINE_PRIORITIES          1U

#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configUSE_MALLOC_FAILED_HOOK            0
#define configCHECK_FOR_STACK_OVERFLOW          2
#define configUSE_TRACE_FACILITY                0
#define configUSE_STATS_FORMATTING_FUNCTIONS    0
#define configGENERATE_RUN_TIME_STATS           0
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS 0

#define INCLUDE_vTaskDelay                      1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_uxTaskGetStackHighWaterMark     1
#define INCLUDE_vTaskDelete                     0
#define INCLUDE_vTaskSuspend                    0

#define configPRIO_BITS                         4U
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 15U
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5U
#define configKERNEL_INTERRUPT_PRIORITY         (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8U - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8U - configPRIO_BITS))

#define configASSERT(condition) do { if ((condition) == 0) { flowcan_rtos_assert_failed(__FILE__, __LINE__); } } while (0)

#endif

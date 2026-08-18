#include "rtos_app.h"

#include "activity.h"
#include "can_recovery.h"
#include "config.h"
#include "dronecan.h"
#include "flowcan/time_utils.h"
#include "flowcan/types.h"
#include "msp.h"
#include "platform.h"
#include "pmw3901.h"
#include "scheduler.h"
#include "status_led.h"
#include "supervisor.h"
#include "vl53l1x.h"
#include "FreeRTOS.h"
#include "event_groups.h"
#include "queue.h"
#include "task.h"

#include <limits.h>
#include <stddef.h>
#include <stdint.h>

#define HEARTBEAT_COMMUNICATION (1UL << 0)
#define HEARTBEAT_FLOW          (1UL << 1)
#define HEARTBEAT_RANGE         (1UL << 2)
#define HEARTBEAT_REQUIRED      (HEARTBEAT_COMMUNICATION | HEARTBEAT_FLOW | HEARTBEAT_RANGE)

#define APP_READY_FLOW          (1UL << 16)
#define APP_READY_RANGE         (1UL << 17)
#define APP_HEALTH_MASK         0xFFFFUL

static StaticTask_t communication_tcb;
static StaticTask_t flow_tcb;
static StaticTask_t range_tcb;
static StaticTask_t system_tcb;
static StaticTask_t idle_tcb;

static StackType_t communication_stack[RTOS_COMM_STACK_WORDS];
static StackType_t flow_stack[RTOS_FLOW_STACK_WORDS];
static StackType_t range_stack[RTOS_RANGE_STACK_WORDS];
static StackType_t system_stack[RTOS_SYSTEM_STACK_WORDS];
static StackType_t idle_stack[RTOS_IDLE_STACK_WORDS];

static TaskHandle_t communication_handle;
static TaskHandle_t flow_handle;
static TaskHandle_t range_handle;
static TaskHandle_t system_handle;

static StaticQueue_t flow_queue_control;
static StaticQueue_t range_queue_control;
static uint8_t flow_queue_storage[sizeof(flow_sample_t)];
static uint8_t range_queue_storage[sizeof(range_sample_t)];
static QueueHandle_t flow_queue;
static QueueHandle_t range_queue;

static StaticEventGroup_t app_events_control;
static StaticEventGroup_t heartbeat_events_control;
static EventGroupHandle_t app_events;
static EventGroupHandle_t heartbeat_events;
static volatile bool fatal_error;
static volatile bool scheduler_started;

static void health_update(health_flags_t set_bits, health_flags_t clear_bits)
{
    if (clear_bits != 0U) {
        (void)xEventGroupClearBits(app_events, (EventBits_t)clear_bits);
    }
    if (set_bits != 0U) {
        (void)xEventGroupSetBits(app_events, (EventBits_t)set_bits);
    }
}

static health_flags_t health_snapshot(void)
{
    health_flags_t health = (health_flags_t)(xEventGroupGetBits(app_events) & APP_HEALTH_MASK);
    if (fatal_error) {
        health |= HEALTH_RTOS_FATAL;
    }
    return health;
}

static void set_ready(EventBits_t bit, bool ready)
{
    if (ready) {
        (void)xEventGroupSetBits(app_events, bit);
    } else {
        (void)xEventGroupClearBits(app_events, bit);
    }
}

static void signal_activity(activity_led_t led)
{
    if (system_handle != NULL) {
        (void)xTaskNotify(system_handle, 1UL << (uint32_t)led, eSetBits);
    }
}

static void notify_task_from_isr(TaskHandle_t handle)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    if (scheduler_started && handle != NULL) {
        vTaskNotifyGiveFromISR(handle, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

void rtos_notify_flow_from_isr(void)
{
    notify_task_from_isr(flow_handle);
}

void rtos_notify_range_from_isr(void)
{
    notify_task_from_isr(range_handle);
}

void rtos_notify_communication_from_isr(void)
{
    notify_task_from_isr(communication_handle);
}

static void publish_flow_to_protocols(const flow_sample_t *sample, bool bus_off, uint8_t *frame)
{
#if FEATURE_MSP
    const size_t length = msp_encode_flow(sample, frame, 32U);
    if (length == 0U || !platform_uart_tx(frame, length)) {
        health_update(HEALTH_UART_TX_OVERFLOW, 0U);
    }
#else
    (void)frame;
#endif
#if FEATURE_DRONECAN
    if (!bus_off && !dronecan_publish_flow(sample)) {
        health_update(HEALTH_CAN_TX_OVERFLOW, 0U);
    }
#else
    (void)bus_off;
#endif
}

static void publish_range_to_protocols(const range_sample_t *sample, bool bus_off, uint8_t *frame)
{
#if FEATURE_MSP
    const size_t length = msp_encode_range(sample, frame, 32U);
    if (length == 0U || !platform_uart_tx(frame, length)) {
        health_update(HEALTH_UART_TX_OVERFLOW, 0U);
    }
#else
    (void)frame;
#endif
#if FEATURE_DRONECAN
    if (!bus_off && !dronecan_publish_range(sample)) {
        health_update(HEALTH_CAN_TX_OVERFLOW, 0U);
    }
#else
    (void)bus_off;
#endif
}

static void communication_task(void *argument)
{
    (void)argument;
    scheduler_started = true;
    msp_parser_t parser;
    msp_parser_init(&parser);
    dronecan_init();
    can_recovery_t recovery;
    can_recovery_init(&recovery);
    uint8_t frame[32];

    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1U));
        const uint32_t now = platform_millis();
        const bool bus_off = platform_can_bus_off();

        if (bus_off) {
            health_update(HEALTH_CAN_BUS_OFF, 0U);
        } else {
            health_update(0U, HEALTH_CAN_BUS_OFF);
        }
        if (can_recovery_step(&recovery, bus_off, now, CAN_RECOVERY_MS)) {
            platform_can_reinit();
        }

        flow_sample_t flow_sample;
        while (xQueueReceive(flow_queue, &flow_sample, 0U) == pdPASS) {
            publish_flow_to_protocols(&flow_sample, bus_off, frame);
        }
        range_sample_t range_sample;
        while (xQueueReceive(range_queue, &range_sample, 0U) == pdPASS) {
            publish_range_to_protocols(&range_sample, bus_off, frame);
        }

        uint8_t byte;
        msp_frame_t received;
        while (platform_uart_rx_pop(&byte)) {
            signal_activity(LED_MSP_RX);
            (void)msp_parser_consume(&parser, byte, &received);
        }
        if (platform_take_can_rx_activity()) {
            signal_activity(LED_CAN_RX);
        }
        if (platform_take_can_rx_overflow()) {
            health_update(HEALTH_CAN_RX_OVERFLOW, 0U);
        }
        dronecan_poll(now, health_snapshot());
        (void)xEventGroupSetBits(heartbeat_events, HEARTBEAT_COMMUNICATION);
    }
}

static void flow_task(void *argument)
{
    (void)argument;
    const uint32_t start_ms = platform_millis();
    pmw3901_t pmw;
    pmw3901_reset_state(&pmw, platform_micros());
    pmw3901_start_init(&pmw, platform_micros());
    uint32_t retry_at_ms = start_ms + SENSOR_RETRY_MS;
    periodic_task_t publisher;
    periodic_task_init(&publisher, start_ms, 1000U / MSP_FLOW_HZ);

    for (;;) {
        uint32_t irq_count = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1U));
        const uint32_t now_ms = platform_millis();
        const uint32_t now_us = platform_micros();

        if (irq_count > 8U) {
            irq_count = 8U;
        }
        while (irq_count > 0U && pmw.initialized) {
            pmw_motion_t motion;
            if (pmw3901_read_motion(&motion)) {
                pmw3901_accumulate(&pmw, &motion);
                signal_activity(LED_PMW);
            } else {
                health_update(HEALTH_PMW_FAULT, 0U);
                set_ready(APP_READY_FLOW, false);
                pmw3901_reset_state(&pmw, now_us);
                retry_at_ms = now_ms + SENSOR_RETRY_MS;
                break;
            }
            irq_count--;
        }

        if (!pmw.initialized && pmw.init_state == 0U && time_reached_u32(now_ms, retry_at_ms)) {
            pmw3901_start_init(&pmw, now_us);
            retry_at_ms = now_ms + SENSOR_RETRY_MS;
        }
        if (!pmw.initialized && pmw.init_state != 0U) {
            const pmw_init_result_t result = pmw3901_init_step(&pmw, now_us);
            if (result == PMW_INIT_READY) {
                health_update(0U, HEALTH_PMW_FAULT);
                set_ready(APP_READY_FLOW, true);
            } else if (result == PMW_INIT_FAULT) {
                health_update(HEALTH_PMW_FAULT, 0U);
                set_ready(APP_READY_FLOW, false);
                pmw.init_state = 0U;
                retry_at_ms = now_ms + SENSOR_RETRY_MS;
            }
        }

        if (periodic_task_due(&publisher, now_ms)) {
            flow_sample_t sample;
            pmw3901_publish(&pmw, platform_micros(), &sample);
            (void)xQueueOverwrite(flow_queue, &sample);
            if (communication_handle != NULL) {
                xTaskNotifyGive(communication_handle);
            }
        }
        (void)xEventGroupSetBits(heartbeat_events, HEARTBEAT_FLOW);
    }
}

static void range_task(void *argument)
{
    (void)argument;
    const uint32_t start_ms = platform_millis();
    tof_t tof;
    tof_begin(&tof, start_ms);
    range_sample_t sample = {.distance_mm = -1, .quality = 0U, .status = RANGE_STATUS_INVALID};
    periodic_task_t publisher;
    periodic_task_init(&publisher, start_ms, 1000U / MSP_RANGE_HZ);

    for (;;) {
        const uint32_t irq_count = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1U));
        const uint32_t now_ms = platform_millis();
        if (irq_count != 0U) {
            tof_irq_notify(&tof);
            signal_activity(LED_TOF);
        }

        bool fresh = false;
        if (tof_step(&tof, now_ms, &sample, &fresh)) {
            health_update(0U, HEALTH_TOF_FAULT | HEALTH_I2C_FAULT);
            set_ready(APP_READY_RANGE, true);
        } else {
            health_update(HEALTH_TOF_FAULT | HEALTH_I2C_FAULT, 0U);
            set_ready(APP_READY_RANGE, false);
        }
        (void)fresh;

        if (periodic_task_due(&publisher, now_ms)) {
            (void)xQueueOverwrite(range_queue, &sample);
            if (communication_handle != NULL) {
                xTaskNotifyGive(communication_handle);
            }
        }
        (void)xEventGroupSetBits(heartbeat_events, HEARTBEAT_RANGE);
    }
}

static void system_task(void *argument)
{
    (void)argument;
    activity_init();
    uint32_t heartbeat_deadline = platform_millis() + RTOS_HEARTBEAT_WINDOW_MS;
    uint32_t stack_check_deadline = platform_millis() + 1000U;
    bool heartbeat_complete = false;

    for (;;) {
        uint32_t activity_bits = 0U;
        (void)xTaskNotifyWait(0U, UINT32_MAX, &activity_bits, pdMS_TO_TICKS(10U));
        const uint32_t now = platform_millis();
        for (uint32_t i = 0U; i < 4U; i++) {
            if ((activity_bits & (1UL << i)) != 0U) {
                activity_pulse((activity_led_t)i, now);
            }
        }
        activity_step(now);

        const EventBits_t app_state = xEventGroupGetBits(app_events);
        health_flags_t health = (health_flags_t)(app_state & APP_HEALTH_MASK);
        if (fatal_error) {
            health |= HEALTH_RTOS_FATAL;
        }
        const bool initializing = (app_state & (APP_READY_FLOW | APP_READY_RANGE)) !=
                                  (APP_READY_FLOW | APP_READY_RANGE);
        status_led_step(now, health, initializing);

        const EventBits_t heartbeat = xEventGroupWaitBits(heartbeat_events,
                                                          HEARTBEAT_REQUIRED,
                                                          pdTRUE,
                                                          pdTRUE,
                                                          0U);
        if ((heartbeat & HEARTBEAT_REQUIRED) == HEARTBEAT_REQUIRED) {
            heartbeat_complete = true;
        }
        if (time_reached_u32(now, heartbeat_deadline)) {
            const uint32_t observed = heartbeat_complete ? HEARTBEAT_REQUIRED : 0U;
            if (supervisor_should_feed(observed, HEARTBEAT_REQUIRED, fatal_error)) {
                platform_watchdog_feed();
                health_update(0U, HEALTH_WATCHDOG_MISSED);
            } else {
                health_update(HEALTH_WATCHDOG_MISSED, 0U);
            }
            if (!heartbeat_complete) {
                (void)xEventGroupClearBits(heartbeat_events, HEARTBEAT_REQUIRED);
            }
            heartbeat_complete = false;
            heartbeat_deadline += RTOS_HEARTBEAT_WINDOW_MS;
        }

        if (time_reached_u32(now, stack_check_deadline)) {
            const uint32_t watermarks[] = {
                (uint32_t)uxTaskGetStackHighWaterMark(communication_handle),
                (uint32_t)uxTaskGetStackHighWaterMark(flow_handle),
                (uint32_t)uxTaskGetStackHighWaterMark(range_handle),
                (uint32_t)uxTaskGetStackHighWaterMark(system_handle)
            };
            if (supervisor_stack_low(watermarks,
                                     sizeof(watermarks) / sizeof(watermarks[0]),
                                     RTOS_STACK_LOW_WATER_WORDS)) {
                health_update(HEALTH_RTOS_STACK_LOW, 0U);
            } else {
                health_update(0U, HEALTH_RTOS_STACK_LOW);
            }
            stack_check_deadline += 1000U;
        }
    }
}

bool rtos_app_init(void)
{
    app_events = xEventGroupCreateStatic(&app_events_control);
    heartbeat_events = xEventGroupCreateStatic(&heartbeat_events_control);
    flow_queue = xQueueCreateStatic(1U,
                                    sizeof(flow_sample_t),
                                    flow_queue_storage,
                                    &flow_queue_control);
    range_queue = xQueueCreateStatic(1U,
                                     sizeof(range_sample_t),
                                     range_queue_storage,
                                     &range_queue_control);
    if (app_events == NULL || heartbeat_events == NULL ||
        flow_queue == NULL || range_queue == NULL) {
        return false;
    }

    system_handle = xTaskCreateStatic(system_task,
                                      "system",
                                      RTOS_SYSTEM_STACK_WORDS,
                                      NULL,
                                      RTOS_SYSTEM_PRIORITY,
                                      system_stack,
                                      &system_tcb);
    communication_handle = xTaskCreateStatic(communication_task,
                                             "communication",
                                             RTOS_COMM_STACK_WORDS,
                                             NULL,
                                             RTOS_COMM_PRIORITY,
                                             communication_stack,
                                             &communication_tcb);
    flow_handle = xTaskCreateStatic(flow_task,
                                    "flow",
                                    RTOS_FLOW_STACK_WORDS,
                                    NULL,
                                    RTOS_FLOW_PRIORITY,
                                    flow_stack,
                                    &flow_tcb);
    range_handle = xTaskCreateStatic(range_task,
                                     "range",
                                     RTOS_RANGE_STACK_WORDS,
                                     NULL,
                                     RTOS_RANGE_PRIORITY,
                                     range_stack,
                                     &range_tcb);
    return system_handle != NULL && communication_handle != NULL &&
           flow_handle != NULL && range_handle != NULL;
}

void vApplicationGetIdleTaskMemory(StaticTask_t **task_buffer,
                                   StackType_t **stack_buffer,
                                   configSTACK_DEPTH_TYPE *stack_size)
{
    *task_buffer = &idle_tcb;
    *stack_buffer = idle_stack;
    *stack_size = RTOS_IDLE_STACK_WORDS;
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
    (void)task;
    (void)task_name;
    fatal_error = true;
    for (;;) {
    }
}

void flowcan_rtos_assert_failed(const char *file, int line)
{
    (void)file;
    (void)line;
    fatal_error = true;
    for (;;) {
    }
}

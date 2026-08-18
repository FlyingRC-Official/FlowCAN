#include "platform.h"
#include "rtos_app.h"
#include "FreeRTOS.h"
#include "task.h"

int main(void)
{
    platform_init();
    if (!rtos_app_init()) {
        flowcan_rtos_assert_failed(__FILE__, __LINE__);
    }
    vTaskStartScheduler();
    for (;;) {
    }
}

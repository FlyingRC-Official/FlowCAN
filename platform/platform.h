#ifndef FLOWCAN_PLATFORM_H
#define FLOWCAN_PLATFORM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "canard.h"

void platform_init(void);
uint32_t platform_millis(void);
uint32_t platform_micros(void);
void platform_delay_us(uint32_t us);
void platform_watchdog_feed(void);

bool platform_spi_transfer(uint8_t value, uint8_t *received);
void platform_pmw_cs(bool asserted);
void platform_pmw_reset(bool asserted);

void platform_tof_xshut(bool enabled);
bool platform_i2c_write(uint8_t address, const uint8_t *data, size_t length, uint32_t timeout_us);
bool platform_i2c_read(uint8_t address, uint8_t *data, size_t length, uint32_t timeout_us);

bool platform_uart_tx(const uint8_t *data, size_t length);
bool platform_uart_rx_pop(uint8_t *byte);

bool platform_can_tx(const CanardCANFrame *frame);
bool platform_can_rx_pop(CanardCANFrame *frame, uint64_t *timestamp_us);
bool platform_can_bus_off(void);
void platform_can_reinit(void);
bool platform_take_can_rx_overflow(void);
bool platform_take_can_rx_activity(void);

typedef enum { LED_PMW = 0, LED_TOF, LED_MSP_RX, LED_CAN_RX } activity_led_t;
void platform_activity_led(activity_led_t led, bool on);
bool platform_ws2812_start(uint8_t red, uint8_t green, uint8_t blue);
bool platform_ws2812_busy(void);

#endif

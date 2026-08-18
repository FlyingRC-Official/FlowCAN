/* FlowCAN platform binding for the ST VL53L1X ULD core. */
#ifndef VL53L1_PLATFORM_H
#define VL53L1_PLATFORM_H

#include <stdint.h>

int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t reg, uint8_t *data, uint32_t count);
int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t reg, uint8_t *data, uint32_t count);
int8_t VL53L1_WrByte(uint16_t dev, uint16_t reg, uint8_t data);
int8_t VL53L1_WrWord(uint16_t dev, uint16_t reg, uint16_t data);
int8_t VL53L1_WrDWord(uint16_t dev, uint16_t reg, uint32_t data);
int8_t VL53L1_RdByte(uint16_t dev, uint16_t reg, uint8_t *data);
int8_t VL53L1_RdWord(uint16_t dev, uint16_t reg, uint16_t *data);
int8_t VL53L1_RdDWord(uint16_t dev, uint16_t reg, uint32_t *data);
int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms);

#endif

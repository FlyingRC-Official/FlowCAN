#ifndef FLOWCAN_TIME_UTILS_H
#define FLOWCAN_TIME_UTILS_H

#include <stdbool.h>
#include <stdint.h>

static inline bool time_reached_u32(uint32_t now, uint32_t deadline)
{
    return (int32_t)(now - deadline) >= 0;
}

static inline uint32_t elapsed_u32(uint32_t now, uint32_t then)
{
    return now - then;
}

#endif

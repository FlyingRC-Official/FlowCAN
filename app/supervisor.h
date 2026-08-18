#ifndef FLOWCAN_SUPERVISOR_H
#define FLOWCAN_SUPERVISOR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

bool supervisor_should_feed(uint32_t observed, uint32_t required, bool fatal);
bool supervisor_stack_low(const uint32_t *watermarks, size_t count, uint32_t minimum_words);

#endif

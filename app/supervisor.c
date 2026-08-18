#include "supervisor.h"

bool supervisor_should_feed(uint32_t observed, uint32_t required, bool fatal)
{
    return !fatal && (observed & required) == required;
}

bool supervisor_stack_low(const uint32_t *watermarks, size_t count, uint32_t minimum_words)
{
    for (size_t i = 0U; i < count; i++) {
        if (watermarks[i] < minimum_words) {
            return true;
        }
    }
    return false;
}

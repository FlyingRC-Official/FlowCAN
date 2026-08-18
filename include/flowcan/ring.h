#ifndef FLOWCAN_RING_H
#define FLOWCAN_RING_H

#include <stdbool.h>
#include <stddef.h>

static inline size_t ring_used(size_t capacity,size_t head,size_t tail)
{
    return head>=tail?head-tail:capacity-(tail-head);
}

static inline bool ring_can_write(size_t capacity,size_t head,size_t tail,size_t length)
{
    return capacity>1U&&head<capacity&&tail<capacity&&
           length<=capacity-1U-ring_used(capacity,head,tail);
}

#endif

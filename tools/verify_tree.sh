#!/bin/sh
set -eu
test -f LICENSE
test -f THIRD_PARTY.md
grep -q 'ORIGIN = 0x08000000' linker/at32f415kb_flash.ld
grep -q 'PROVISIONAL' config/config.h
grep -q 'NAN' protocol/dronecan.c
grep -q '#define configSUPPORT_DYNAMIC_ALLOCATION        0' config/FreeRTOSConfig.h
grep -q '#define configSUPPORT_STATIC_ALLOCATION         1' config/FreeRTOSConfig.h
grep -q 'vPortSVCHandler' startup/startup_at32f415.s
grep -q 'xPortPendSVHandler' startup/startup_at32f415.s
grep -q 'xPortSysTickHandler' startup/startup_at32f415.s
grep -q 'CANARD_ENABLE_DEADLINE=1' Makefile
grep -q 'canardCleanupStaleTransfers' protocol/dronecan.c
grep -q 'r.dlc>CANARD_CAN_FRAME_MAX_DATA_LEN' platform/platform.c
grep -q 'I2C_TDC_FLAG' platform/platform.c
grep -q 'ring_can_write' platform/platform.c
grep -q 'PMW3901_SPI_TIMEOUT_US' platform/platform.c
test -f third_party/freertos/LICENSE.md
if find third_party/freertos -type f -name 'heap_*.c' | grep -q .; then echo 'FreeRTOS heap implementation found'; exit 1; fi
if rg -n --glob '*.c' --glob '*.h' '\b(malloc|calloc|realloc|free)\s*\(' app drivers platform protocol include config; then echo 'Dynamic allocation call found'; exit 1; fi
echo 'Repository policy checks: PASS'

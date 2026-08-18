#!/bin/sh
set -eu
test -f LICENSE
test -f THIRD_PARTY.md
grep -q 'ORIGIN = 0x08000000' linker/at32f415kb_flash.ld
grep -q 'PROVISIONAL' config/config.h
grep -q 'NAN' protocol/dronecan.c
grep -q 'CANARD_ENABLE_DEADLINE=1' Makefile
grep -q 'canardCleanupStaleTransfers' protocol/dronecan.c
grep -q 'r.dlc>CANARD_CAN_FRAME_MAX_DATA_LEN' platform/platform.c
grep -q 'I2C_TDC_FLAG' platform/platform.c
grep -q 'ring_can_write' platform/platform.c
grep -q 'PMW3901_SPI_TIMEOUT_US' platform/platform.c
if rg -n --glob '*.c' --glob '*.h' '\b(malloc|calloc|realloc|free)\s*\(' app drivers platform protocol include config; then echo 'Dynamic allocation call found'; exit 1; fi
echo 'Repository policy checks: PASS'

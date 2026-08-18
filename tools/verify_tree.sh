#!/bin/sh
set -eu
test -f LICENSE
test -f THIRD_PARTY.md
test -f build/release/SHA256SUMS.txt || true
grep -q 'ORIGIN = 0x08000000' linker/at32f415kb_flash.ld
grep -q 'PROVISIONAL' config/config.h
grep -q 'NAN' protocol/dronecan.c
grep -q '#define configSUPPORT_DYNAMIC_ALLOCATION        0' config/FreeRTOSConfig.h
grep -q '#define configSUPPORT_STATIC_ALLOCATION         1' config/FreeRTOSConfig.h
grep -q 'vPortSVCHandler' startup/startup_at32f415.s
grep -q 'xPortPendSVHandler' startup/startup_at32f415.s
grep -q 'xPortSysTickHandler' startup/startup_at32f415.s
test -f third_party/freertos/LICENSE.md
if find third_party/freertos -type f -name 'heap_*.c' | grep -q .; then echo 'FreeRTOS heap implementation found'; exit 1; fi
if rg -n --glob '*.c' --glob '*.h' '\b(malloc|calloc|realloc|free)\s*\(' app drivers platform protocol include config; then echo 'Dynamic allocation call found'; exit 1; fi
echo 'Repository policy checks: PASS'

#!/bin/sh
set -eu
test -f LICENSE
test -f THIRD_PARTY.md
test -f build/release/SHA256SUMS.txt || true
grep -q 'ORIGIN = 0x08000000' linker/at32f415kb_flash.ld
grep -q 'PROVISIONAL' config/config.h
grep -q 'NAN' protocol/dronecan.c
if rg -n --glob '*.c' --glob '*.h' '\b(malloc|calloc|realloc|free)\s*\(' app drivers platform protocol include config; then echo 'Dynamic allocation call found'; exit 1; fi
echo 'Repository policy checks: PASS'

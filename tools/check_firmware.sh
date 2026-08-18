#!/bin/sh
set -eu
elf=$1
map=$2
size_line=$(${SIZE:-arm-none-eabi-size} "$elf" | tail -n 1)
text=$(printf '%s\n' "$size_line" | awk '{print $1}')
data=$(printf '%s\n' "$size_line" | awk '{print $2}')
bss=$(printf '%s\n' "$size_line" | awk '{print $3}')
flash=$((text + data))
ram=$((data + bss))
test "$flash" -le 131072 || { echo "Flash overflow: $flash"; exit 1; }
test "$ram" -le 32768 || { echo "RAM overflow: $ram"; exit 1; }
entry=$(${READELF:-arm-none-eabi-readelf} -h "$elf" | awk '/Entry point address/{print $4}')
case "$entry" in 0x0800*|0x800*) ;; *) echo "Unexpected entry: $entry"; exit 1;; esac
vector=$(${OBJDUMP:-arm-none-eabi-objdump} -h "$elf" | awk '$2==".isr_vector"{print $4}')
test "$vector" = "08000000" || { echo "Vector table is $vector"; exit 1; }
if ${NM:-arm-none-eabi-nm} -u "$elf" | grep -Eq '(^| )(_?malloc|_?calloc|_?realloc|_?free)$'; then echo "Dynamic allocation symbol found"; exit 1; fi
if grep -Eq 'USART1.*(printf|puts)|printf.*USART1' "$map"; then echo "Debug strings routed to MSP UART"; exit 1; fi
printf 'Limits OK: flash=%s/131072 bytes, ram=%s/32768 bytes, vector=%s\n' "$flash" "$ram" "$vector"

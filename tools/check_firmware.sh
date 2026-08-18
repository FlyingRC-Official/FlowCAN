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
for symbol in vTaskStartScheduler vPortSVCHandler xPortPendSVHandler xPortSysTickHandler; do
    ${NM:-arm-none-eabi-nm} "$elf" | grep -Eq " [Tt] ${symbol}$" || { echo "Missing RTOS symbol: $symbol"; exit 1; }
done
if ${NM:-arm-none-eabi-nm} "$elf" | grep -Eq ' [TtU] (pvPortMalloc|vPortFree|malloc|calloc|realloc|free)$'; then echo "Dynamic RTOS allocation symbol found"; exit 1; fi

vector_file="${TMPDIR:-/tmp}/flowcan-vector-$$.bin"
trap 'rm -f "$vector_file"' EXIT HUP INT TERM
${OBJCOPY:-arm-none-eabi-objcopy} -O binary --only-section=.isr_vector "$elf" "$vector_file"
check_vector_handler() {
    offset=$1
    symbol=$2
    vector_value=$(od -An -tu4 -j "$offset" -N 4 "$vector_file" | tr -d ' ')
    symbol_hex=$(${NM:-arm-none-eabi-nm} "$elf" | awk -v wanted="$symbol" '$3==wanted{print $1}')
    symbol_value=$((0x$symbol_hex + 1))
    test "$vector_value" -eq "$symbol_value" || { echo "Vector offset $offset does not reference $symbol"; exit 1; }
}
check_vector_handler 44 vPortSVCHandler
check_vector_handler 56 xPortPendSVHandler
check_vector_handler 60 xPortSysTickHandler

stack_total=0
for stack in communication_stack flow_stack range_stack system_stack idle_stack; do
    stack_hex=$(${NM:-arm-none-eabi-nm} -S "$elf" | awk -v wanted="$stack" '$4==wanted{print $2}')
    test -n "$stack_hex" || { echo "Missing static stack: $stack"; exit 1; }
    stack_total=$((stack_total + 0x$stack_hex))
done
test "$stack_total" -eq 6400 || { echo "Unexpected static task stack budget: $stack_total"; exit 1; }
if grep -Eq 'USART1.*(printf|puts)|printf.*USART1' "$map"; then echo "Debug strings routed to MSP UART"; exit 1; fi
printf 'Limits OK: flash=%s/131072 bytes, ram=%s/32768 bytes, vector=%s, static_task_stacks=%s bytes\n' "$flash" "$ram" "$vector" "$stack_total"

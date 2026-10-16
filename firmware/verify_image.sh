#!/usr/bin/env bash
# Check that the linked image is well formed.
#
# Linking is a weak claim on its own: an image can link and still have the
# vector table at the wrong address, a stack pointer outside RAM, an interrupt
# entry pointing at the default handler, or a DMA buffer somewhere the DMA
# controllers cannot reach. None of that can be found without hardware. All of
# it can be found in the ELF and this is where.
#
# Run after build_firmware.sh. Needs arm-none-eabi-readelf, nm, objdump and
# size.
#
# Every tool is run into a variable rather than into a pipe. awk exiting early
# on a large symbol table sends the producer a broken pipe and under
# pipefail that reads as a failure of the check rather than of the image.
set -eu

cd "$(dirname "$0")"

ELF=build-arm/pa_firmware.elf
BIN=build-arm/pa_firmware.bin

test -f "$ELF" || { echo "no image at $ELF; run ./build_firmware.sh first"; exit 1; }
test -s "$BIN" || { echo "no binary at $BIN"; exit 1; }

failures=0
fail() { echo "FAIL: $*"; failures=$((failures + 1)); }
pass() { echo "ok:   $*"; }

sections=$(arm-none-eabi-readelf -S -W "$ELF")
symbols=$(arm-none-eabi-nm "$ELF")

# Found by locating the section name and taking the field two along, which is
# the address. Reading a fixed column number instead breaks the moment the
# section index reaches double figures, because readelf prints "[ 9]" as two
# fields and "[10]" as one.
section_address() {
  echo "$sections" | awk -v s="$1" '{ for (i = 1; i <= NF; i++) if ($i == s) { print $(i + 2); exit } }'
}
symbol_address() {
  echo "$symbols" | awk -v n="$1" '$3 == n {print $1; exit}'
}

# --- the vector table is first and at the reset address --------------------
vector_addr=$(section_address .isr_vector)
if [ "0x$vector_addr" = "0x08000000" ]; then
  pass "vector table at 0x08000000"
else
  fail "vector table at 0x$vector_addr, expected 0x08000000"
fi

# --- the first two words: initial stack pointer and reset handler -----------
dump=$(arm-none-eabi-objdump -s -j .isr_vector "$ELF")
words=$(echo "$dump" | awk 'NR==5 {print $2, $3}')
swap() { echo "${1:6:2}${1:4:2}${1:2:2}${1:0:2}"; }
sp=$(swap "$(echo "$words" | cut -d' ' -f1)")
pc=$(swap "$(echo "$words" | cut -d' ' -f2)")

sp_dec=$((16#$sp))
if [ "$sp_dec" -gt $((0x20000000)) ] && [ "$sp_dec" -le $((0x20020000)) ]; then
  pass "initial stack pointer 0x$sp is at the top of DTCM"
else
  fail "initial stack pointer 0x$sp is outside DTCM"
fi

# Thumb entry points have bit zero set. A reset vector with an even address
# faults on its first instruction, before any handler that could report it.
if [ $((16#$pc & 1)) -eq 1 ]; then
  pass "reset vector 0x$pc is a thumb address"
else
  fail "reset vector 0x$pc is not a thumb address"
fi

reset_symbol=$(symbol_address Reset_Handler)
if [ -n "$reset_symbol" ] && [ $((16#$pc & 0xFFFFFFFE)) -eq $((16#$reset_symbol)) ]; then
  pass "reset vector points at Reset_Handler"
else
  fail "reset vector 0x$pc does not point at Reset_Handler at 0x${reset_symbol:-?}"
fi

# --- the three interrupts this firmware actually uses -----------------------
# Entry 16 + n is external interrupt n. Getting this wrong produces a board
# that runs and quietly never services its DMA.
for entry in "27:DMA1_Stream0_IRQHandler" "28:DMA1_Stream1_IRQHandler" "53:USART1_IRQHandler"; do
  index=${entry%%:*}
  name=${entry##*:}
  symbol=$(symbol_address "$name")
  if [ -z "$symbol" ]; then
    fail "$name is not in the image"
    continue
  fi
  value=$(od -An -tx4 -j $((index * 4)) -N 4 "$BIN" | tr -d ' \n')
  if [ $((16#$value & 0xFFFFFFFE)) -eq $((16#$symbol)) ]; then
    pass "vector $index points at $name"
  else
    fail "vector $index is 0x$value, expected $name at 0x$symbol"
  fi
done

# --- sections are where the linker script says ------------------------------
check_section() {
  local name=$1 expected=$2
  local addr
  addr=$(section_address "$name")
  if [ -z "$addr" ]; then
    fail "$name is missing"
    return
  fi
  case "0x$addr" in
    ${expected}*) pass "$name at 0x$addr" ;;
    *) fail "$name at 0x$addr, expected ${expected}..." ;;
  esac
}
check_section .text 0x0800
check_section .data 0x2000
check_section .bss 0x2000

# The DMA buffers have to be in AXI SRAM. In DTCM the controllers cannot reach
# them and every transfer silently does nothing, which is the single failure
# this check exists for.
check_section .dma_buffer 0x24

# --- nothing has overflowed -------------------------------------------------
size_line=$(arm-none-eabi-size "$ELF" | tail -1)
text=$(echo "$size_line" | awk '{print $1}')
data=$(echo "$size_line" | awk '{print $2}')
bss=$(echo "$size_line" | awk '{print $3}')
flash_used=$((text + data))
ram_used=$((data + bss))

if [ "$flash_used" -lt $((2 * 1024 * 1024)) ]; then
  pass "$flash_used B of flash used of 2 MB"
else
  fail "the image does not fit in flash"
fi
if [ "$ram_used" -lt $((128 * 1024 - 8 * 1024)) ]; then
  pass "$ram_used B of DTCM used, $((128 * 1024 - ram_used)) B left for the 8 KB stack"
else
  fail "data and bss leave no room for the stack"
fi

echo
if [ "$failures" -eq 0 ]; then
  echo "The image is well formed. It has never been flashed; firmware/renode runs it on an"
  echo "emulated STM32H743."
  exit 0
fi
echo "$failures check(s) failed."
exit 1

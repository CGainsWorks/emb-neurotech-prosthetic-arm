#!/usr/bin/env bash
# Cross-compile the STM32H743 firmware image.
#
#   ./build_firmware.sh          configure and build
#   ./build_firmware.sh clean    wipe the build directory first
#
# Requires arm-none-eabi-gcc, g++, objcopy and size.
#
# The image links and its sections land where the linker script says. It has
# never been flashed. See firmware/docs/BRING_UP.md.
set -euo pipefail

cd "$(dirname "$0")"

if [ "${1:-}" = "clean" ]; then
  rm -rf build-arm
fi

jobs="$( (command -v nproc >/dev/null && nproc) || echo 4)"

cmake -S . -B build-arm \
  -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Release

cmake --build build-arm -j"${jobs}"

echo
echo "Artifacts:"
ls -l build-arm/pa_firmware.elf build-arm/pa_firmware.bin build-arm/pa_firmware.hex 2>/dev/null || true

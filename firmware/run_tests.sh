#!/usr/bin/env bash
# Build and run the host-side firmware unit tests. No board required.
#
#   ./run_tests.sh          configure, build and run everything
#   ./run_tests.sh clean    wipe the build directory first
set -euo pipefail

cd "$(dirname "$0")"

if [ "${1:-}" = "clean" ]; then
  rm -rf build
fi

jobs="$( (command -v nproc >/dev/null && nproc) || echo 4)"

cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j"${jobs}"

cd build
ctest --output-on-failure

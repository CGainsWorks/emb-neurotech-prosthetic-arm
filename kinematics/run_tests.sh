#!/usr/bin/env bash
# Build and run the kinematics unit tests. No ROS, no hardware, no network.
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

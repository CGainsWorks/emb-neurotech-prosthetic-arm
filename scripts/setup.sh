#!/usr/bin/env bash
# What this repository needs and where to get it.
#
#   ./scripts/setup.sh          report what is present and what is missing
#   ./scripts/setup.sh install  apt-get the missing pieces (Debian or Ubuntu)
#
# Nothing here needs hardware. There is none.
set -eu

cd "$(dirname "$0")/.."

missing=0
have() {
  if command -v "$1" >/dev/null 2>&1; then
    printf 'ok      %-24s %s\n' "$1" "$2"
  else
    printf 'MISSING %-24s %s\n' "$1" "$2"
    missing=$((missing + 1))
  fi
}

echo "Required for the kinematics library and the firmware's portable core:"
have cmake "any version from 3.16"
have g++ "any compiler with C++17"
have python3 "3.8 or later"

echo
echo "Required for the firmware image:"
have arm-none-eabi-gcc "Arm GNU Toolchain; scripts/install_arm_toolchain.sh fetches it"
have arm-none-eabi-objcopy "produces the .bin and .hex"

echo
echo "Required for the ROS 2 workspace and the end-to-end simulation:"
if [ -d /opt/ros ]; then
  printf 'ok      %-24s %s\n' "ROS 2" "$(ls /opt/ros | tr '\n' ' ')"
else
  printf 'MISSING %-24s %s\n' "ROS 2" "Humble; or use docker compose"
  missing=$((missing + 1))
fi
have colcon "builds the workspace"

echo
if [ "${1:-}" = "install" ]; then
  echo "Installing the pieces that do not need a ROS apt source."
  sudo apt-get update
  sudo apt-get install -y --no-install-recommends \
    build-essential cmake python3 python3-pytest python3-venv curl xz-utils
  echo
  echo "Installing the pinned Arm GNU Toolchain. Put this on PATH:"
  "$(dirname "$0")/install_arm_toolchain.sh"
  echo
  echo "ROS 2 Humble is not installed from here: it needs its own apt source,"
  echo "and the instructions change. See docs.ros.org, or run everything in"
  echo "Docker with 'make docker-up', which needs none of the above."
  exit 0
fi

if [ "$missing" -eq 0 ]; then
  echo "Everything is present. Run 'make demo'."
else
  echo "$missing item(s) missing. Run './scripts/setup.sh install', or run"
  echo "everything in a container with 'make docker-up', which needs only Docker."
fi

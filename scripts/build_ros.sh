#!/usr/bin/env bash
# Build the ROS 2 workspace, the kinematics library and the firmware core.
#
#   ./scripts/build_ros.sh              build
#   ./scripts/build_ros.sh clean        wipe build, install and log first
#   ./scripts/build_ros.sh test         build, then run every test
#
# The kinematics library and the firmware core are plain CMake packages that
# live outside ros2_ws/src, because neither of them depends on ROS and putting
# them under it would suggest otherwise. colcon takes several base paths, so
# they are built and installed ahead of the packages that consume them without
# either being duplicated or symlinked.
set -eu

cd "$(dirname "$0")/.."

if [ "${1:-}" = "clean" ]; then
  rm -rf build install log
  shift || true
fi

# AMENT_PREFIX_PATH rather than ROS_DISTRO. The official ros images set
# ROS_DISTRO as a plain environment variable without having sourced anything,
# so testing that one skips the sourcing and every package then fails to find
# ament_cmake.
if [ -z "${AMENT_PREFIX_PATH:-}" ]; then
  for candidate in /opt/ros/*/setup.bash; do
    if [ -f "$candidate" ]; then
      # ROS setup scripts read variables they have not set, so nounset has to
      # come off for the duration. It goes straight back on afterwards.
      set +u
      # shellcheck disable=SC1090
      . "$candidate"
      set -u
      break
    fi
  done
fi

if [ -z "${AMENT_PREFIX_PATH:-}" ]; then
  echo "no ROS 2 installation found; source one first, or use docker compose" >&2
  exit 1
fi

jobs="$( (command -v nproc >/dev/null && nproc) || echo 4)"

colcon build \
  --base-paths kinematics firmware ros2_ws/src \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  --parallel-workers "${jobs}"

if [ "${1:-}" = "test" ]; then
  colcon test --base-paths kinematics firmware ros2_ws/src --return-code-on-test-failure
  colcon test-result --verbose
fi

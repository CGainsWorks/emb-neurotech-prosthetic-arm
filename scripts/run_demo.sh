#!/usr/bin/env bash
# Everything, from a clean checkout, in about four minutes.
#
#   ./scripts/run_demo.sh
#
# Builds and tests the kinematics library, the firmware's portable core and
# the ROS 2 workspace, regenerates the description and the hardware analyses
# from the models that produce them, runs the end-to-end simulation and
# checks that every documented number still matches the result it came from.
#
# Nothing here needs hardware. Every figure it prints is either calculated or
# simulated and the outputs say which.
set -eu

cd "$(dirname "$0")/.."

say() { printf '\n\033[1m== %s\033[0m\n' "$1"; }

say "Kinematics library"
./kinematics/run_tests.sh

say "Firmware, host side"
./firmware/run_tests.sh

if command -v arm-none-eabi-gcc >/dev/null 2>&1; then
  say "Firmware image for the STM32H743"
  ./firmware/build_firmware.sh
  ./firmware/verify_image.sh
else
  say "Firmware image: skipped, no arm-none-eabi-gcc"
fi

say "The description, regenerated from the model"
./kinematics/build/dump_model > ros2_ws/src/arm_description/config/a1_arm_model.json
python3 scripts/generate_urdf.py
python3 -m pytest ros2_ws/src/arm_description/test -q

say "Workspace and solver timing"
mkdir -p analysis
./kinematics/build/workspace_report --samples 200000 > analysis/workspace.json
./kinematics/build/ik_benchmark --iterations 20000 > analysis/kinematics_latency.json

say "The analogue front end and the power tree"
python3 hardware/semg-servo-board/analysis/report.py
python3 -m pytest hardware/semg-servo-board/analysis -q

say "ROS 2 workspace"
./scripts/build_ros.sh test

say "Acquisition through to actuation"
mkdir -p analysis/sim
set +u
# shellcheck disable=SC1091
. install/setup.bash
set -u
ros2 run arm_bringup sim_demo --output analysis/sim/control_loop_sim.json
python3 scripts/check_sim_result.py analysis/sim/control_loop_sim.json

say "Every documented number still matches its source"
python3 scripts/generate_urdf.py --check
python3 scripts/check_docs.py

say "Done"
echo "Results are in analysis/ and hardware/semg-servo-board/analysis/."
echo "None of them was measured on hardware, because there is none."

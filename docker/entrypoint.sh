#!/usr/bin/env bash
# Source the ROS environment and the workspace, then run whatever was asked
# for. nounset comes off for the sourcing because the ROS setup scripts read
# variables they have not set.
set -e
set +u
. /opt/ros/humble/setup.bash
if [ -f /arm/install/setup.bash ]; then
  . /arm/install/setup.bash
fi
set -u
exec "$@"

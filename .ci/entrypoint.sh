#!/bin/bash
set -e
source "/opt/ros/$ROS_DISTRO/setup.bash"
# Source workspace if it exists (built in dep, or manual build in dev)
[ -f "/ros2_ws/install/setup.bash" ] && source "/ros2_ws/install/setup.bash"
exec "$@"
#!/usr/bin/env bash
set -e

echo "=== apriltag_pose entrypoint ==="

source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash

unset RMW_IMPLEMENTATION

cd /ros2_ws

echo "=== Building apriltag_pose ==="
colcon build --packages-select apriltag_pose
source /ros2_ws/install/setup.bash

echo "=== Launching apriltag_pose ==="
exec ros2 launch apriltag_pose apriltag_pose.launch.py \ filter_type:=median \ filter_window_size:=5
    publishing_frame:=pelvis \
    verbose:=true

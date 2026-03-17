#!/usr/bin/env bash
set -e

echo "=== apriltag_pose entrypoint ==="

source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash

cd /ros2_ws

echo "=== Building apriltag_pose ==="
colcon build --packages-select apriltag_pose
source /ros2_ws/install/setup.bash

echo "=== Launching apriltag_pose ==="
LAUNCH_ARGS="verbose:=${APRILTAG_VERBOSE:-true}"
LAUNCH_ARGS="${LAUNCH_ARGS} debug:=${APRILTAG_DEBUG:-false}"
LAUNCH_ARGS="${LAUNCH_ARGS} enable_apriltag_to_box:=${ENABLE_APRILTAG_TO_BOX:-false}"
LAUNCH_ARGS="${LAUNCH_ARGS} apriltag_to_box_tag_id:=${APRILTAG_TO_BOX_TAG_ID:-23}"
LAUNCH_ARGS="${LAUNCH_ARGS} box_pose_topic:=${BOX_POSE_TOPIC:-/g1pilot/box_pose}"
if [ -n "${APRILTAG_PUBLISHING_FRAME}" ]; then
    LAUNCH_ARGS="${LAUNCH_ARGS} publishing_frame:=${APRILTAG_PUBLISHING_FRAME}"
fi
if [ -n "${BOX_PUBLISHING_FRAME}" ]; then
    LAUNCH_ARGS="${LAUNCH_ARGS} box_publishing_frame:=${BOX_PUBLISHING_FRAME}"
fi
LAUNCH_ARGS="${LAUNCH_ARGS} box_extents:=${BOX_EXTENTS:-0.02,0.0,1.09,0.30,0.25,0.56}"
exec ros2 launch apriltag_pose apriltag_pose_rgbd.launch.py ${LAUNCH_ARGS}

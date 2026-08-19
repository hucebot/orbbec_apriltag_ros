#!/usr/bin/env bash
set -e

echo "=== apriltag_pose entrypoint ==="

source /opt/ros/${ROS_DISTRO:-humble}/setup.bash

cd /ros2_ws

echo "=== Starting RouDi ==="
iox-roudi &
ROUDI_PID=$!

echo "=== Building apriltag_pose ==="
colcon build --packages-select apriltag_pose
source /ros2_ws/install/setup.bash

echo "=== Launching apriltag_pose ==="
LAUNCH_ARGS="verbose:=${APRILTAG_VERBOSE:-true}"
LAUNCH_ARGS="${LAUNCH_ARGS} debug:=${APRILTAG_DEBUG:-false}"
LAUNCH_ARGS="${LAUNCH_ARGS} enable_apriltag_to_box:=${ENABLE_APRILTAG_TO_BOX:-false}"
LAUNCH_ARGS="${LAUNCH_ARGS} apriltag_to_box_tag_id:=${APRILTAG_TO_BOX_TAG_ID:-23}"
LAUNCH_ARGS="${LAUNCH_ARGS} box_pose_topic:=${BOX_POSE_TOPIC:-/g1pilot/box_pose}"

# Add these lines to catch the camera topics:
if [ -n "${IMAGE_TOPIC}" ]; then
    LAUNCH_ARGS="${LAUNCH_ARGS} image_topic:=${IMAGE_TOPIC}"
fi
if [ -n "${DEPTH_TOPIC}" ]; then
    LAUNCH_ARGS="${LAUNCH_ARGS} depth_topic:=${DEPTH_TOPIC}"
fi
if [ -n "${CAMERA_INFO_TOPIC}" ]; then
    LAUNCH_ARGS="${LAUNCH_ARGS} camera_info_topic:=${CAMERA_INFO_TOPIC}"
fi

if [ -n "${APRILTAG_PUBLISHING_FRAME}" ]; then
    LAUNCH_ARGS="${LAUNCH_ARGS} publishing_frame:=${APRILTAG_PUBLISHING_FRAME}"
fi
if [ -n "${BOX_PUBLISHING_FRAME}" ]; then
    LAUNCH_ARGS="${LAUNCH_ARGS} box_publishing_frame:=${BOX_PUBLISHING_FRAME}"
fi
LAUNCH_ARGS="${LAUNCH_ARGS} box_extents:=${BOX_EXTENTS:-0.02,0.0,1.09,0.30,0.25,0.56}"
if [ -n "${APRILTAG_FIX_AXIS}" ]; then
    LAUNCH_ARGS="${LAUNCH_ARGS} fix_axis:=${APRILTAG_FIX_AXIS}"
fi
if [ -n "${APRILTAG_POSE_MODE}" ]; then
    LAUNCH_ARGS="${LAUNCH_ARGS} pose_mode:=${APRILTAG_POSE_MODE}"
fi
if [ -n "${APRILTAG_TAG_SIZE}" ]; then
    LAUNCH_ARGS="${LAUNCH_ARGS} tag_size:=${APRILTAG_TAG_SIZE}"
fi

if [ -n "${APRILTAG_DISPLAY}" ]; then
    LAUNCH_ARGS="${LAUNCH_ARGS} display:=${APRILTAG_DISPLAY}"
fi

exec "$@" ${LAUNCH_ARGS}
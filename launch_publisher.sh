#!/bin/bash
# Usage: ./launch_publisher.sh [display] [verbose]
#   display: true/false (default: false)
#   verbose: true/false (default: false)

DISPLAY_ARG=${1:-false}
VERBOSE_ARG=${2:-false}

IsRunning=`docker ps -f name=apriltag_pose | grep -c "apriltag_pose"`;
if [ $IsRunning -eq "0" ]; then
    xhost +local:docker
    docker run --rm \
        --gpus all \
        -e DISPLAY=$DISPLAY \
        -e XAUTHORITY=$XAUTHORITY \
        -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -e 'QT_X11_NO_MITSHM=1' \
        -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-1} \
        -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v "$(dirname "$(readlink -f "$0")")/apriltag_pose":/ros2_ws/src/apriltag_pose:ro \
        --ipc host \
        --device /dev/dri \
        --net host \
        --name apriltag_pose \
        -ti inria_docker:orbbec \
        bash -c "source /ros2_ws/install/setup.bash && ros2 launch apriltag_pose apriltag_pose.launch.py display:=${DISPLAY_ARG} verbose:=${VERBOSE_ARG}"
else
    echo "Docker image is already running. Aborting.";
fi

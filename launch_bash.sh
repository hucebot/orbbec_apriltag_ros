#!/bin/bash
IsRunning=`docker ps -f name=apriltag_pose | grep -c "apriltag_pose"`;
if [ $IsRunning -eq "0" ]; then
    echo "Docker image is not running. Starting it...";
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
        -ti inria_docker:orbbec
else
    echo "Docker image is already running. Opening new terminal...";
    docker exec -ti apriltag_pose /bin/bash
fi

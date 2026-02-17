#!/bin/bash
xhost +local:docker
docker run --rm \
    --gpus all \
    -e DISPLAY=$DISPLAY \
    -e XAUTHORITY=$XAUTHORITY \
    -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e 'QT_X11_NO_MITSHM=1' \
    -e ROS_DOMAIN_ID=1 \
    -e RMW_IMPLEMENTATION= \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$(dirname "$(readlink -f "$0")")/apriltag_pose":/ros2_ws/src/apriltag_pose:ro \
    -v /dev/shm:/dev/shm \
    --ipc host \
    --device /dev/dri \
    --net host \
    --name apriltag_pose \
    -ti inria_docker:orbbec \
    bash -c "source /opt/ros/humble/setup.bash && cd /ros2_ws && colcon build --packages-select apriltag_pose && source install/setup.bash && bash"
#!/bin/bash
IsRunning=`docker ps -f name=apriltag_pose | grep -c "apriltag_pose"`;
if [ $IsRunning -eq "0" ]; then
    echo "Docker image is not running. Starting it...";
    xhost +local:docker
    docker rm apriltag_pose
    docker run  \
        --name apriltag_pose  \
        --gpus all \
        -e DISPLAY=$DISPLAY \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        --env QT_X11_NO_MITSHM=1 \
        --net host \
        --ipc host \
        --pid host \
        --privileged \
        -it \
        -v $(pwd):/host_ws \
        -v /dev:/dev \
        -v /run/udev:/run/udev \
        --device /dev/dri \
        --device /dev/snd \
        --device /dev/input \
        --device /dev/bus/usb \
        -e ROS_DOMAIN_ID=1\
        -v $(pwd)/configs/:/xml_configs \
        -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\
        -e CYCLONEDDS_URI=/xml_configs/cyclonedds.xml\
        -w /ros2_ws \
        inria_docker:apriltag_pose
else
    echo "Docker image is already running. Opening new terminal...";
    docker exec -ti apriltag_pose /bin/bash
fi

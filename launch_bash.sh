#!/bin/bash
IsRunning=`docker ps -f name=inria_orbbec | grep -c "orbbec"`;
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
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v `pwd`/inria_orbbec_tags:/root/catkin_ws/src/inria_orbbec_tags \
        --ipc host \
        --device /dev/dri \
        --device /dev/snd \
        --device /dev/input \
        --device /dev/bus/usb \
        --privileged \
        --ulimit rtprio=99 \
        --net host \
        --name inria_orbbec \
        --entrypoint /bin/bash \
        -ti inria_docker:orbbec
else
    echo "Docker image is already running. Opening new terminal...";
    docker exec -ti inria_orbbec /bin/bash
fi


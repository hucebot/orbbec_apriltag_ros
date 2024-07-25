#!/bin/bash
IsRunning=`docker ps -f name=inria_orbbec | grep -c "orbbec"`;
if [ $IsRunning -eq "0" ]; then
    xhost +local:docker
    docker run --rm \
        --gpus all \
        -e DISPLAY=$DISPLAY \
        -e XAUTHORITY=$XAUTHORITY \
        -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -e 'QT_X11_NO_MITSHM=1' \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
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
        -ti inria_docker:orbbec \
        -c "source ./install/setup.bash ; ./inria_orbbec_tags $1 $2 $3 $4"
else
    echo "Docker image is already running. Aborting.";
fi


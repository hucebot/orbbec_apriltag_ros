FROM nvidia/opengl:1.2-glvnd-devel-ubuntu20.04

#System full upgrade
RUN apt-get update && apt-get --with-new-pkgs upgrade -y

#Essential packages
RUN apt-get update && apt-get install -y apt-utils
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Paris
RUN apt-get update && apt-get install -y --fix-missing \
    git vim curl build-essential zlib1g-dev libx11-dev libusb-1.0-0-dev freeglut3-dev liblapacke-dev \
    unzip libopenblas-dev libatlas-base-dev cmake make lsb-release tree \
    sudo ca-certificates gnupg-agent libssl-dev apt-transport-https \
    software-properties-common usbutils mesa-utils mesa-va-drivers vainfo \
    python3-pip python3-numpy libeigen3-dev \
    libv4l-dev v4l-utils wget curl libnuma-dev libnuma1 libgles2-mesa 

#Install ROS
RUN \
    echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update
RUN apt-get update && apt-get -y -o Dpkg::Options::="--force-overwrite" dist-upgrade
RUN apt-get update && apt-get install -y \
    ros-noetic-catkin python3-catkin-tools ros-noetic-ros-base ros-noetic-geometry-msgs \
    ros-noetic-common-msgs ros-noetic-roscpp ros-noetic-realtime-tools

#Install OpenCV
RUN apt-get update && apt-get install -y libopencv-dev libopencv-core-dev opencv-data

#Install Orbbec SDK
RUN \
    cd /opt && \
    wget https://dl.orbbec3d.com/dist/orbbecsdk/1.8.3/OrbbecSDK_1.8.3_Linux.zip && \
    unzip OrbbecSDK_1.8.3_Linux.zip && \
    unzip OrbbecSDK_1.8.3_Linux/OrbbecSDK_C_C++_v1.8.3_20231124_6c51dc1_linux_x64_release.zip && \
    rm -rf OrbbecSDK_1.8.3_Linux/ OrbbecSDK_1.8.3_Linux.zip && \
    wget https://dl.orbbec3d.com/dist/orbbecsdk/1.8.3/OrbbecViewer_1.8.3_Linux.zip && \
    unzip OrbbecViewer_1.8.3_Linux.zip && \
    unzip OrbbecViewer_1.8.3_Linux/OrbbecViewer_v1.8.3_202311241128_linux_x64_release.zip && \
    rm -rf OrbbecViewer_1.8.3_Linux.zip OrbbecViewer_1.8.3_Linux && \
    mv OrbbecViewer_v1.8.3_202311241128_linux_x64_release/ OrbbecViewer_v1.8.3

#Install AprilTags
RUN \
    cd /opt && \
    git clone --depth 1 https://github.com/AprilRobotics/apriltag.git && \
    cd apriltag && mkdir -p build && cd build && \
    cmake .. && make

#Setup catkin workspace
RUN mkdir -p /root/catkin_ws/src && \
    /bin/bash -c \
    "source /opt/ros/*/setup.bash && \
    cd /root/catkin_ws && catkin init && \
    catkin config --extend /opt/ros/noetic --install -DCMAKE_BUILD_TYPE=Release"
WORKDIR /root/catkin_ws
RUN echo "source /root/catkin_ws/install/setup.bash" >> /root/.bashrc
#Precompilation
COPY inria_orbbec_tags /root/catkin_ws/src/inria_orbbec_tags
RUN /bin/bash -c "cd /root/catkin_ws && catkin build"
RUN ln -s ./devel/lib/inria_orbbec_tags/inria_orbbec_tags ./inria_orbbec_tags


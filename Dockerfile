FROM nvidia/cuda:12.6.1-cudnn-devel-ubuntu24.04
ENV ROS_DISTRO=jazzy

ENV DISPLAY=:0
ENV LIBGL_ALWAYS_INDIRECT=0

SHELL ["/bin/bash", "-c"]

ENV QT_X11_NO_MITSHM=1
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Paris

#System full upgrade
RUN apt-get update && apt-get upgrade -y

#Essential packages
RUN apt-get update && apt-get install -y --fix-missing \
    git vim curl build-essential zlib1g-dev libx11-dev freeglut3-dev liblapacke-dev \
    unzip libopenblas-dev cmake make lsb-release tree \
    sudo ca-certificates gpg-agent libssl-dev \
    software-properties-common mesa-utils mesa-va-drivers vainfo \
    python3-pip python3-numpy libeigen3-dev \
    libv4l-dev v4l-utils wget curl libnuma-dev libnuma1 libgles-dev \
    libglvnd-dev libgl1-mesa-dev libglu1-mesa-dev

#Install ROS 2 Jazzy
RUN apt-get update && apt-get install -y --no-install-recommends curl ca-certificates gnupg && \
    rm -rf /var/lib/apt/lists/*
RUN ROS_APT_SOURCE_VERSION="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
      | grep -F "tag_name" | awk -F\" '{print $4}')" && \
    UBUNTU_CODENAME="$(. /etc/os-release && echo ${UBUNTU_CODENAME:-$VERSION_CODENAME})" && \
    curl -fsSL -o /tmp/ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${UBUNTU_CODENAME}_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    rm -f /tmp/ros2-apt-source.deb
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-desktop-full \
    ros-dev-tools \
    && \
    rm -rf /var/lib/apt/lists/*

#Install CycloneDDS
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

#Install OpenCV
RUN apt-get update && apt-get install -y libopencv-dev libopencv-core-dev opencv-data

#Install AprilTags
RUN cd /opt && \
    git clone --depth 1 https://github.com/AprilRobotics/apriltag.git && \
    cd apriltag && mkdir -p build && cd build && \
    cmake .. && make && make install && ldconfig

#Setup colcon workspace and build
RUN mkdir -p /ros2_ws/src
COPY apriltag_pose /ros2_ws/src/apriltag_pose
WORKDIR /ros2_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --packages-select apriltag_pose
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]

# --- STAGE 1: Base ---
FROM nvidia/opengl:1.2-glvnd-devel-ubuntu22.04 AS base
ENV ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# 1. Install prerequisites for adding repositories
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg2 lsb-release ca-certificates

# 2. Add ROS 2 apt repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3. Install dependencies and ROS 2
RUN apt-get update && apt-get install -y --no-install-recommends \
    git vim build-essential zlib1g-dev libx11-dev freeglut3-dev \
    cmake make tree sudo python3-pip python3-numpy libeigen3-dev \
    libopencv-dev libv4l-dev \
    ros-${ROS_DISTRO}-desktop-full ros-dev-tools ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

# 4. Install AprilTags Library
RUN cd /opt && git clone --depth 1 https://github.com/AprilRobotics/apriltag.git && \
    cd apriltag && mkdir build && cd build && cmake .. && make install && ldconfig

WORKDIR /ros2_ws
COPY .ci/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# --- STAGE 2: Dev ---
FROM base AS dev
CMD ["/bin/bash"]

# --- STAGE 3: Dep ---
FROM base AS dep
# Copy source and build inside the image
COPY apriltag_pose /ros2_ws/src/apriltag_pose
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --packages-select apriltag_pose

# Embed configs
COPY configs/cyclonedds.xml /etc/ros2/cyclonedds.xml
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=/etc/ros2/cyclonedds.xml
ENV ROS_DOMAIN_ID=1

# Default launch command
CMD ros2 launch apriltag_pose apriltag_pose.launch.py image_topic:=/tiago_head_camera_down/color/image_raw cloud_topic:=/tiago_head_camera_down/depth_registered/points display:=true verbose:=true
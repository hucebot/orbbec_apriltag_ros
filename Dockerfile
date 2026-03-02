FROM registry.gitlab.com/bleurobotics/code/bleu_integration/bleu-base:jazzy-cuda-26-02

# OpenCV
RUN apt-get update && apt-get install -y \
    libopencv-dev libopencv-core-dev opencv-data \
    freeglut3-dev \
    && rm -rf /var/lib/apt/lists/*

# AprilTags
RUN cd /opt && \
    git clone --depth 1 https://github.com/AprilRobotics/apriltag.git && \
    cd apriltag && mkdir -p build && cd build && \
    cmake .. && make && make install && ldconfig

# Setup colcon workspace and build
RUN mkdir -p /ros2_ws/src
COPY apriltag_pose /ros2_ws/src/apriltag_pose
WORKDIR /ros2_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --packages-select apriltag_pose

CMD ["bash"]

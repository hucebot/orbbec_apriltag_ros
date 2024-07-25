cmake_minimum_required(VERSION 2.8.12)
project(inria_orbbec_tags)

find_package(catkin REQUIRED
    roscpp
    geometry_msgs
)
catkin_package()

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
    src
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    /opt/OrbbecSDK_v1.8.3/SDK/include
    /opt/apriltag
)
link_directories(
    /opt/OrbbecSDK_v1.8.3/SDK/lib
    /opt/apriltag/build
)

add_compile_options(-W -Wall -std=c++17)

add_executable(inria_orbbec_tags src/tags.cpp)
target_link_libraries(inria_orbbec_tags
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    pthread
    OrbbecSDK
    apriltag
)


cmake_minimum_required(VERSION 3.8)
project(apriltag_pose)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-W -Wall -std=c++17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
    /opt/apriltag
)
link_directories(
    /opt/apriltag/build
)

add_executable(apriltag_pose src/tags.cpp)
target_include_directories(apriltag_pose PRIVATE
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(apriltag_pose
    rclcpp
    geometry_msgs
    sensor_msgs
    tf2_ros
)
target_link_libraries(apriltag_pose
    ${OpenCV_LIBS}
    apriltag
)

install(TARGETS apriltag_pose
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

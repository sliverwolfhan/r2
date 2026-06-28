cmake_minimum_required(VERSION 3.10)

project(usb_apriltag_tf)

# ---- C++17 ----
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- OpenCV ----
find_package(OpenCV REQUIRED)

# ---- AprilTag ----
find_package(PkgConfig REQUIRED)
pkg_check_modules(APRILTAG REQUIRED apriltag)

# ---- ROS2 Humble ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(tag tag.cpp)

target_include_directories(tag PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${APRILTAG_INCLUDE_DIRS}
)

target_link_libraries(tag
    ${OpenCV_LIBS}
    ${APRILTAG_LIBRARIES}
    rclcpp::rclcpp
    tf2_ros::tf2_ros
    tf2::tf2
)

ament_target_dependencies(tag
    rclcpp
    tf2_ros
    tf2
    geometry_msgs
)

# install
install(TARGETS tag
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()

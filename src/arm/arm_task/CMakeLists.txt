cmake_minimum_required(VERSION 3.8)
project(arm_task)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)

# Include directories
include_directories(include)

# Create executable
add_executable(arm_task
  src/main.cpp
  src/robot.cpp
  src/task/idel.cpp
  src/task/catch_kfs.cpp
  src/task/move_kfs.cpp
  src/task/place_kfs.cpp
)

add_executable(catch_kfs_test
  src/catch_kfs_test.cpp
)

add_executable(move_kfs_test
  src/move_kfs_test.cpp
)

add_executable(place_kfs_test
  src/place_kfs_test.cpp
)

ament_target_dependencies(arm_task
  rclcpp
  rclcpp_action
  rcl_interfaces
  geometry_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
  robot_interfaces
  visualization_msgs
)

ament_target_dependencies(catch_kfs_test
  rclcpp
  rclcpp_action
  geometry_msgs
  tf2_ros
  robot_interfaces
)

ament_target_dependencies(move_kfs_test
  rclcpp
  rclcpp_action
  robot_interfaces
)

ament_target_dependencies(place_kfs_test
  rclcpp
  rclcpp_action
  geometry_msgs
  tf2_ros
  robot_interfaces
)

target_link_libraries(arm_task
  yaml-cpp
)

# Install executable
install(TARGETS arm_task
  catch_kfs_test
  move_kfs_test
  place_kfs_test
  DESTINATION lib/${PROJECT_NAME}
)

# Install config directory
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

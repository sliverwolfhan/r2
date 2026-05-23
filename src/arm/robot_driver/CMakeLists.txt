cmake_minimum_required(VERSION 3.8)
project(robot_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)


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

add_executable(robot_driver
  src/main.cpp
  src/serialnode.cpp
  src/cdc_trans.cpp
)


target_include_directories(robot_driver
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/robot_driver>
  $<INSTALL_INTERFACE:include>
)

# Find and link libusb (libusb-1.0) via pkg-config to resolve undefined references
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

target_include_directories(robot_driver
  PUBLIC
  ${LIBUSB_INCLUDE_DIRS}
)

target_link_directories(robot_driver
  PUBLIC
  ${LIBUSB_LIBRARY_DIRS}
)

target_link_libraries(robot_driver
  ${LIBUSB_LIBRARIES}
)



ament_target_dependencies(robot_driver
  rclcpp
  robot_interfaces
  sensor_msgs
)

install(TARGETS
robot_driver
DESTINATION lib/${PROJECT_NAME})

ament_package()

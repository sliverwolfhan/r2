cmake_minimum_required(VERSION 3.8)
project(vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(vision_node vision.cpp)

target_include_directories(vision_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(vision_node ${OpenCV_LIBS})

ament_target_dependencies(
  vision_node
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
)

install(TARGETS
  vision_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

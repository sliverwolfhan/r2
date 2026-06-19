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

# 强制使用本地编译的 OpenCV 4.14.0（系统默认 4.5.4 无法解析 YOLOv8/11 ONNX）
set(OpenCV_DIR "/home/zk/opencv-4.14.0/build" CACHE PATH "" FORCE)
find_package(OpenCV 4.14 REQUIRED)
message(STATUS "vision_node using OpenCV ${OpenCV_VERSION} from ${OpenCV_DIR}")

add_executable(vision_node vision.cpp)

target_include_directories(vision_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(vision_node ${OpenCV_LIBS})

# 运行时优先加载 4.14 的动态库，而非 /usr/lib 里的 4.5.4
set_target_properties(vision_node PROPERTIES
  BUILD_RPATH "/home/zk/opencv-4.10.0/build/lib"
  INSTALL_RPATH "/home/zk/opencv-4.10.0/build/lib"
  BUILD_WITH_INSTALL_RPATH FALSE) 

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

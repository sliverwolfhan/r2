cmake_minimum_required(VERSION 3.8)
project(at_r2_serial_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

# 头文件目录
include_directories(
  include
  ${LIBUSB_INCLUDE_DIRS}
)

# CDC传输库
add_library(at_r2_serial_bridge_cdc_trans SHARED
  src/cdc_trans.cpp
)
target_link_libraries(at_r2_serial_bridge_cdc_trans ${LIBUSB_LIBRARIES})
ament_target_dependencies(at_r2_serial_bridge_cdc_trans rclcpp)

# AT R2 串口桥接节点
add_executable(at_r2_serial_bridge_node
  src/at_r2_serial_bridge_node.cpp
)
target_link_libraries(at_r2_serial_bridge_node at_r2_serial_bridge_cdc_trans)
ament_target_dependencies(at_r2_serial_bridge_node rclcpp std_msgs geometry_msgs tf2_ros tf2 tf2_msgs)
set_target_properties(at_r2_serial_bridge_node PROPERTIES
  INSTALL_RPATH "$ORIGIN/.."
)

# 安装
install(TARGETS
  at_r2_serial_bridge_cdc_trans
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  at_r2_serial_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

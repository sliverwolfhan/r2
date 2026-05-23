cmake_minimum_required(VERSION 3.8)
project(r2_meilin_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)

set(_planner_core_srcs
  src/planner.cpp
  src/forest_defaults.cpp
  src/block_table.cpp
)

add_executable(r2_planner_node
  src/r2_planner_node.cpp
  ${_planner_core_srcs}
)
target_include_directories(r2_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(r2_planner_node yaml-cpp)
ament_target_dependencies(r2_planner_node rclcpp std_msgs geometry_msgs nav_msgs visualization_msgs robot_interfaces)

add_executable(evaluator
  src/evaluator.cpp
  ${_planner_core_srcs}
)
target_link_libraries(evaluator yaml-cpp)
ament_target_dependencies(evaluator robot_interfaces)

add_executable(r2_qt_planner_node
  src/r2_qt_planner_node.cpp
  src/r2_qt_planner_window.cpp
  include/r2_meilin_planner/r2_qt_planner_window.hpp
  ${_planner_core_srcs}
)
target_include_directories(r2_qt_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(r2_qt_planner_node Qt5::Widgets yaml-cpp)
ament_target_dependencies(r2_qt_planner_node rclcpp robot_interfaces ament_index_cpp)
target_include_directories(evaluator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

add_executable(plan_yaml_publisher_node
  src/plan_yaml_publisher_node.cpp
)
target_include_directories(plan_yaml_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(plan_yaml_publisher_node yaml-cpp)
ament_target_dependencies(plan_yaml_publisher_node rclcpp robot_interfaces ament_index_cpp)

install(TARGETS
  r2_planner_node
  evaluator
  r2_qt_planner_node
  plan_yaml_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
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

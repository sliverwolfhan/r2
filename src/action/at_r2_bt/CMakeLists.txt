cmake_minimum_required(VERSION 3.8)
project(at_r2_bt)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
get_filename_component(_workspace_root "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(_workspace_robot_interfaces_prefix "${_workspace_root}/install/robot_interfaces")
set(_workspace_robot_interfaces_config "${_workspace_robot_interfaces_prefix}/share/robot_interfaces/cmake/robot_interfacesConfig.cmake")
if(EXISTS "${_workspace_robot_interfaces_config}")
  list(PREPEND CMAKE_PREFIX_PATH "${_workspace_robot_interfaces_prefix}")
endif()
find_package(nav_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(include)

# Create library
add_library(${PROJECT_NAME} SHARED
  src/publish_goal_action.cpp
  src/arm_task_action.cpp
  src/arm_move_joint_action.cpp
  src/arm_move_named_action.cpp
  src/climb_stair_action.cpp
  src/descend_stair_action.cpp
  src/pre_steer_align_action.cpp
  src/set_costmap_inflation_action.cpp
  src/set_mppi_params_action.cpp
  src/get_plan_action.cpp
  src/pop_next_step_action.cpp
  src/peek_next_step_action.cpp
  src/select_arm_prep_name_action.cpp
  src/is_prep_skippable_condition.cpp
  src/grasp_ready_by_pose_distance_condition.cpp
  src/dock_to_wall_action.cpp
  src/publish_head_cmd_action.cpp
  src/set_zone_mode_action.cpp
  src/wait_docking_release_action.cpp
)

# Set dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  behaviortree_cpp
  geometry_msgs
  tf2_geometry_msgs
  pluginlib
  nav2_msgs
  nav_msgs
  std_msgs
  tf2_ros
  robot_interfaces
)
target_link_libraries(${PROJECT_NAME} yaml-cpp)

# Install library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Create simple test executable
add_executable(simple_bt_runner 
  src/simple_bt_runner.cpp
  src/publish_goal_action.cpp
  src/arm_task_action.cpp
  src/arm_move_joint_action.cpp
  src/arm_move_named_action.cpp
  src/climb_stair_action.cpp
  src/descend_stair_action.cpp
  src/pre_steer_align_action.cpp
  src/set_costmap_inflation_action.cpp
  src/set_mppi_params_action.cpp
  src/get_plan_action.cpp
  src/pop_next_step_action.cpp
  src/peek_next_step_action.cpp
  src/select_arm_prep_name_action.cpp
  src/is_prep_skippable_condition.cpp
  src/grasp_ready_by_pose_distance_condition.cpp
  src/dock_to_wall_action.cpp
  src/publish_head_cmd_action.cpp
  src/set_zone_mode_action.cpp
  src/wait_docking_release_action.cpp
  src/delay_decorator.cpp
)
ament_target_dependencies(simple_bt_runner
  rclcpp
  rclcpp_action
  behaviortree_cpp
  ament_index_cpp
  geometry_msgs
  tf2_geometry_msgs
  nav2_msgs
  nav_msgs
  std_msgs
  tf2_ros
  robot_interfaces
)
target_link_libraries(simple_bt_runner yaml-cpp)

install(TARGETS simple_bt_runner
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install plugin description
install(FILES plugins/plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install behavior tree examples
install(DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

# Install yaml configs (block_*.yaml, arm_ready_position.yaml, ...)
install(DIRECTORY yaml/
  DESTINATION share/${PROJECT_NAME}/yaml
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Export plugin
pluginlib_export_plugin_description_file(behaviortree_cpp plugins/plugin_description.xml)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_action
  behaviortree_cpp
  geometry_msgs
  tf2_geometry_msgs
  pluginlib
  nav2_msgs
  nav_msgs
  std_msgs
  tf2_ros
  robot_interfaces
)

ament_package()

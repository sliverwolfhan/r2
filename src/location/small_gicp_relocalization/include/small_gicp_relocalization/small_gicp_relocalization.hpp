// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_
#define SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "pcl/io/pcd_io.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float64.hpp"
#include "small_gicp/ann/kdtree_omp.hpp"
#include "small_gicp/factors/gicp_factor.hpp"
#include "small_gicp/pcl/pcl_point.hpp"
#include "small_gicp/registration/reduction_omp.hpp"
#include "small_gicp/registration/registration.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace small_gicp_relocalization
{

class SmallGicpRelocalizationNode : public rclcpp::Node
{
public:
  explicit SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options);

private:
  void registeredPcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void loadGlobalMap(const std::string & file_name);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filterScanByMapBounds(
    const pcl::PointCloud<pcl::PointXYZ> & scan, const Eigen::Isometry3d & map_to_odom) const;
  void performRegistration();
  void publishTransform();
  Eigen::Isometry3d filterRegistrationResult(const Eigen::Isometry3d & raw_result);
  Eigen::Isometry3d averageTransformWindow() const;
  void resetResultFilter();
  bool shouldResetResultFilter(const Eigen::Isometry3d & raw_result) const;
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void laserDistanceXCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void laserDistanceYCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void tryInitializeLaserPose();
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr laser_x_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr laser_y_sub_;

  int num_threads_;
  int num_neighbors_;
  float global_leaf_size_;
  float registered_leaf_size_;
  float max_dist_sq_;
  bool relocalization_enabled_;
  bool auto_disable_relocalization_;
  bool map_bounds_filter_enabled_;
  bool sliding_window_filter_enabled_;
  bool transform_global_map_;
  int stable_required_count_;
  int map_bounds_filter_min_points_;
  int sliding_window_filter_size_;
  double stable_translation_threshold_;
  double stable_rotation_threshold_;
  double map_bounds_filter_margin_;
  double sliding_window_filter_reset_translation_threshold_;
  double sliding_window_filter_reset_rotation_threshold_;
  bool map_bounds_valid_;
  int stable_count_;
  std::vector<double> init_pose_;
  Eigen::Vector3d map_bounds_min_;
  Eigen::Vector3d map_bounds_max_;

  // Laser-ranging relocalization mode: derive map->odom from two wall-distance
  // lasers once at startup, assuming the robot is already perpendicular to the wall.
  bool laser_localization_enabled_;
  std::string laser_x_topic_;
  std::string laser_y_topic_;
  double laser_distance_scale_;
  double laser_x_offset_;
  double laser_y_offset_;
  bool laser_x_received_;
  bool laser_y_received_;
  bool laser_pose_initialized_;
  double laser_x_raw_;
  double laser_y_raw_;

  std::string map_frame_;
  std::string odom_frame_;
  std::string prior_pcd_file_;
  std::string base_frame_;
  std::string robot_base_frame_;
  std::string lidar_frame_;
  std::string current_scan_frame_id_;
  rclcpp::Time last_scan_time_;
  Eigen::Isometry3d result_t_;
  Eigen::Isometry3d previous_result_t_;
  std::vector<Eigen::Isometry3d> result_window_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr registered_scan_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr target_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr source_;

  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> target_tree_;
  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> source_tree_;
  std::shared_ptr<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>
    register_;

  rclcpp::TimerBase::SharedPtr transform_timer_;
  rclcpp::TimerBase::SharedPtr register_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace small_gicp_relocalization

#endif  // SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_

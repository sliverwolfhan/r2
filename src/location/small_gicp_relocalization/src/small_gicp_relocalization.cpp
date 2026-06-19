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

#include "small_gicp_relocalization/small_gicp_relocalization.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "pcl/common/transforms.h"
#include "pcl_conversions/pcl_conversions.h"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace small_gicp_relocalization
{

SmallGicpRelocalizationNode::SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options)
: Node("small_gicp_relocalization", options),
  map_bounds_valid_(false),
  stable_count_(0),
  map_bounds_min_(Eigen::Vector3d::Zero()),
  map_bounds_max_(Eigen::Vector3d::Zero()),
  result_t_(Eigen::Isometry3d::Identity()),
  previous_result_t_(Eigen::Isometry3d::Identity())
{
  this->declare_parameter("num_threads", 4);
  this->declare_parameter("num_neighbors", 20);
  this->declare_parameter("global_leaf_size", 0.25);
  this->declare_parameter("registered_leaf_size", 0.25);
  this->declare_parameter("max_dist_sq", 1.0);
  this->declare_parameter("relocalization_enabled", true);
  this->declare_parameter("auto_disable_relocalization", true);
  this->declare_parameter("map_bounds_filter_enabled", true);
  this->declare_parameter("sliding_window_filter_enabled", true);
  this->declare_parameter("transform_global_map", false);
  this->declare_parameter("stable_required_count", 5);
  this->declare_parameter("map_bounds_filter_min_points", 1);
  this->declare_parameter("sliding_window_filter_size", 3);
  this->declare_parameter("stable_translation_threshold", 0.02);
  this->declare_parameter("stable_rotation_threshold", 0.02);
  this->declare_parameter("map_bounds_filter_margin", 1.0);
  this->declare_parameter("sliding_window_filter_reset_translation_threshold", 0.3);
  this->declare_parameter("sliding_window_filter_reset_rotation_threshold", 0.3);
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("base_frame", "");
  this->declare_parameter("robot_base_frame", "");
  this->declare_parameter("lidar_frame", "");
  this->declare_parameter("prior_pcd_file", "");
  this->declare_parameter("init_pose", std::vector<double>{0., 0., 0., 0., 0., 0.});

  this->get_parameter("num_threads", num_threads_);
  this->get_parameter("num_neighbors", num_neighbors_);
  this->get_parameter("global_leaf_size", global_leaf_size_);
  this->get_parameter("registered_leaf_size", registered_leaf_size_);
  this->get_parameter("max_dist_sq", max_dist_sq_);
  this->get_parameter("relocalization_enabled", relocalization_enabled_);
  this->get_parameter("auto_disable_relocalization", auto_disable_relocalization_);
  this->get_parameter("map_bounds_filter_enabled", map_bounds_filter_enabled_);
  this->get_parameter("sliding_window_filter_enabled", sliding_window_filter_enabled_);
  this->get_parameter("transform_global_map", transform_global_map_);
  this->get_parameter("stable_required_count", stable_required_count_);
  this->get_parameter("map_bounds_filter_min_points", map_bounds_filter_min_points_);
  this->get_parameter("sliding_window_filter_size", sliding_window_filter_size_);
  this->get_parameter("stable_translation_threshold", stable_translation_threshold_);
  this->get_parameter("stable_rotation_threshold", stable_rotation_threshold_);
  this->get_parameter("map_bounds_filter_margin", map_bounds_filter_margin_);
  this->get_parameter(
    "sliding_window_filter_reset_translation_threshold",
    sliding_window_filter_reset_translation_threshold_);
  this->get_parameter(
    "sliding_window_filter_reset_rotation_threshold", sliding_window_filter_reset_rotation_threshold_);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("prior_pcd_file", prior_pcd_file_);
  this->get_parameter("init_pose", init_pose_);

  if (sliding_window_filter_size_ < 1) {
    RCLCPP_WARN(
      this->get_logger(), "sliding_window_filter_size must be greater than 0, using 1.");
    sliding_window_filter_size_ = 1;
  }
  if (sliding_window_filter_reset_translation_threshold_ < 0.0) {
    RCLCPP_WARN(
      this->get_logger(),
      "sliding_window_filter_reset_translation_threshold must not be negative, using 0.0.");
    sliding_window_filter_reset_translation_threshold_ = 0.0;
  }
  if (sliding_window_filter_reset_rotation_threshold_ < 0.0) {
    RCLCPP_WARN(
      this->get_logger(),
      "sliding_window_filter_reset_rotation_threshold must not be negative, using 0.0.");
    sliding_window_filter_reset_rotation_threshold_ = 0.0;
  }

  // [x, y, z, roll, pitch, yaw] - init_pose parameters
  if (!init_pose_.empty() && init_pose_.size() >= 6) {
    result_t_.translation() << init_pose_[0], init_pose_[1], init_pose_[2];
    result_t_.linear() =
      Eigen::AngleAxisd(init_pose_[5], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(init_pose_[4], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(init_pose_[3], Eigen::Vector3d::UnitX()).toRotationMatrix();
  }
  previous_result_t_ = result_t_;

  accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  register_ = std::make_shared<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&SmallGicpRelocalizationNode::parametersCallback, this, std::placeholders::_1));

  loadGlobalMap(prior_pcd_file_);

  // Downsample points and convert them into pcl::PointCloud<pcl::PointCovariance>
  target_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *global_map_, global_leaf_size_);

  // Estimate covariances of points
  small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);

  // Create KdTree for target
  target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    target_, small_gicp::KdTreeBuilderOMP(num_threads_));

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "registered_scan", 10,
    std::bind(&SmallGicpRelocalizationNode::registeredPcdCallback, this, std::placeholders::_1));

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10,
    std::bind(&SmallGicpRelocalizationNode::initialPoseCallback, this, std::placeholders::_1));

  register_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),  // 2 Hz
    std::bind(&SmallGicpRelocalizationNode::performRegistration, this));

  transform_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),  // 20 Hz
    std::bind(&SmallGicpRelocalizationNode::publishTransform, this));
}

rcl_interfaces::msg::SetParametersResult SmallGicpRelocalizationNode::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & parameter : parameters) {
    if (parameter.get_name() == "relocalization_enabled") {
      relocalization_enabled_ = parameter.as_bool();
      stable_count_ = 0;
      resetResultFilter();
      if (!relocalization_enabled_) {
        accumulated_cloud_->clear();
      }
      RCLCPP_INFO(
        this->get_logger(), "Relocalization %s.", relocalization_enabled_ ? "enabled" : "disabled");
    } else if (parameter.get_name() == "auto_disable_relocalization") {
      auto_disable_relocalization_ = parameter.as_bool();
    } else if (parameter.get_name() == "map_bounds_filter_enabled") {
      map_bounds_filter_enabled_ = parameter.as_bool();
    } else if (parameter.get_name() == "sliding_window_filter_enabled") {
      sliding_window_filter_enabled_ = parameter.as_bool();
      resetResultFilter();
      RCLCPP_INFO(
        this->get_logger(), "Sliding window filter %s.",
        sliding_window_filter_enabled_ ? "enabled" : "disabled");
    } else if (parameter.get_name() == "stable_required_count") {
      const auto stable_required_count = parameter.as_int();
      if (stable_required_count < 1) {
        result.successful = false;
        result.reason = "stable_required_count must be greater than 0";
        return result;
      }
      stable_required_count_ = stable_required_count;
    } else if (parameter.get_name() == "map_bounds_filter_min_points") {
      const auto map_bounds_filter_min_points = parameter.as_int();
      if (map_bounds_filter_min_points < 0) {
        result.successful = false;
        result.reason = "map_bounds_filter_min_points must not be negative";
        return result;
      }
      map_bounds_filter_min_points_ = map_bounds_filter_min_points;
    } else if (parameter.get_name() == "sliding_window_filter_size") {
      const auto sliding_window_filter_size = parameter.as_int();
      if (sliding_window_filter_size < 1) {
        result.successful = false;
        result.reason = "sliding_window_filter_size must be greater than 0";
        return result;
      }
      sliding_window_filter_size_ = sliding_window_filter_size;
      while (result_window_.size() > static_cast<std::size_t>(sliding_window_filter_size_)) {
        result_window_.erase(result_window_.begin());
      }
    } else if (parameter.get_name() == "stable_translation_threshold") {
      const auto stable_translation_threshold = parameter.as_double();
      if (stable_translation_threshold < 0.0) {
        result.successful = false;
        result.reason = "stable_translation_threshold must not be negative";
        return result;
      }
      stable_translation_threshold_ = stable_translation_threshold;
    } else if (parameter.get_name() == "stable_rotation_threshold") {
      const auto stable_rotation_threshold = parameter.as_double();
      if (stable_rotation_threshold < 0.0) {
        result.successful = false;
        result.reason = "stable_rotation_threshold must not be negative";
        return result;
      }
      stable_rotation_threshold_ = stable_rotation_threshold;
    } else if (parameter.get_name() == "map_bounds_filter_margin") {
      const auto map_bounds_filter_margin = parameter.as_double();
      if (map_bounds_filter_margin < 0.0) {
        result.successful = false;
        result.reason = "map_bounds_filter_margin must not be negative";
        return result;
      }
      map_bounds_filter_margin_ = map_bounds_filter_margin;
    } else if (parameter.get_name() == "sliding_window_filter_reset_translation_threshold") {
      const auto reset_translation_threshold = parameter.as_double();
      if (reset_translation_threshold < 0.0) {
        result.successful = false;
        result.reason = "sliding_window_filter_reset_translation_threshold must not be negative";
        return result;
      }
      sliding_window_filter_reset_translation_threshold_ = reset_translation_threshold;
    } else if (parameter.get_name() == "sliding_window_filter_reset_rotation_threshold") {
      const auto reset_rotation_threshold = parameter.as_double();
      if (reset_rotation_threshold < 0.0) {
        result.successful = false;
        result.reason = "sliding_window_filter_reset_rotation_threshold must not be negative";
        return result;
      }
      sliding_window_filter_reset_rotation_threshold_ = reset_rotation_threshold;
    }
  }

  return result;
}

void SmallGicpRelocalizationNode::loadGlobalMap(const std::string & file_name)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", file_name.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Loaded global map with %zu points", global_map_->points.size());

  // NOTE: Transform global pcd_map (based on `lidar_odom` frame) to the `odom` frame
  if (transform_global_map_) {
    Eigen::Affine3d odom_to_lidar_odom;
    while (true) {
      try {
        auto tf_stamped = tf_buffer_->lookupTransform(
          base_frame_, lidar_frame_, this->now(), rclcpp::Duration::from_seconds(1.0));
        odom_to_lidar_odom = tf2::transformToEigen(tf_stamped.transform);
        RCLCPP_INFO_STREAM(
          this->get_logger(), "odom_to_lidar_odom: translation = "
                                << odom_to_lidar_odom.translation().transpose() << ", rpy = "
                                << odom_to_lidar_odom.rotation().eulerAngles(0, 1, 2).transpose());
        break;
      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s Retrying...", ex.what());
        rclcpp::sleep_for(std::chrono::seconds(1));
      }
    }
    pcl::transformPointCloud(*global_map_, *global_map_, odom_to_lidar_odom);
  } else {
    RCLCPP_INFO(
      this->get_logger(),
      "transform_global_map is disabled, using prior PCD as-is without TF transform.");
  }

  if (global_map_->empty()) {
    map_bounds_valid_ = false;
    RCLCPP_WARN(this->get_logger(), "Global map is empty, map bounds filter is disabled.");
    return;
  }

  map_bounds_min_ = Eigen::Vector3d(
    std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
    std::numeric_limits<double>::max());
  map_bounds_max_ = Eigen::Vector3d(
    std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(),
    std::numeric_limits<double>::lowest());

  for (const auto & point : global_map_->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }

    map_bounds_min_.x() = std::min(map_bounds_min_.x(), static_cast<double>(point.x));
    map_bounds_min_.y() = std::min(map_bounds_min_.y(), static_cast<double>(point.y));
    map_bounds_min_.z() = std::min(map_bounds_min_.z(), static_cast<double>(point.z));
    map_bounds_max_.x() = std::max(map_bounds_max_.x(), static_cast<double>(point.x));
    map_bounds_max_.y() = std::max(map_bounds_max_.y(), static_cast<double>(point.y));
    map_bounds_max_.z() = std::max(map_bounds_max_.z(), static_cast<double>(point.z));
  }

  map_bounds_valid_ =
    map_bounds_min_.x() <= map_bounds_max_.x() && map_bounds_min_.y() <= map_bounds_max_.y() &&
    map_bounds_min_.z() <= map_bounds_max_.z();

  if (!map_bounds_valid_) {
    RCLCPP_WARN(this->get_logger(), "Global map has no finite points, map bounds filter is disabled.");
    return;
  }

  RCLCPP_INFO_STREAM(
    this->get_logger(), "Global map bounds: min = " << map_bounds_min_.transpose()
                                                     << ", max = " << map_bounds_max_.transpose());
}

pcl::PointCloud<pcl::PointXYZ>::Ptr SmallGicpRelocalizationNode::filterScanByMapBounds(
  const pcl::PointCloud<pcl::PointXYZ> & scan, const Eigen::Isometry3d & map_to_odom) const
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
  filtered->points.reserve(scan.points.size());
  filtered->header = scan.header;

  const Eigen::Vector3d min_bound =
    map_bounds_min_ - Eigen::Vector3d::Constant(map_bounds_filter_margin_);
  const Eigen::Vector3d max_bound =
    map_bounds_max_ + Eigen::Vector3d::Constant(map_bounds_filter_margin_);

  for (const auto & point : scan.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }

    const Eigen::Vector3d point_map =
      map_to_odom * Eigen::Vector3d(point.x, point.y, point.z);
    if ((point_map.array() >= min_bound.array()).all() &&
      (point_map.array() <= max_bound.array()).all())
    {
      filtered->points.push_back(point);
    }
  }

  filtered->width = filtered->points.size();
  filtered->height = 1;
  filtered->is_dense = false;
  return filtered;
}

Eigen::Isometry3d SmallGicpRelocalizationNode::filterRegistrationResult(
  const Eigen::Isometry3d & raw_result)
{
  if (shouldResetResultFilter(raw_result)) {
    resetResultFilter();
  }

  result_window_.push_back(raw_result);
  while (result_window_.size() > static_cast<std::size_t>(sliding_window_filter_size_)) {
    result_window_.erase(result_window_.begin());
  }

  return averageTransformWindow();
}

Eigen::Isometry3d SmallGicpRelocalizationNode::averageTransformWindow() const
{
  if (result_window_.empty()) {
    return result_t_;
  }

  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  Eigen::Vector4d rotation_coefficients = Eigen::Vector4d::Zero();
  const Eigen::Quaterniond reference(result_window_.front().rotation());

  for (const auto & transform : result_window_) {
    translation += transform.translation();

    Eigen::Quaterniond rotation(transform.rotation());
    if (rotation.dot(reference) < 0.0) {
      rotation.coeffs() *= -1.0;
    }
    rotation_coefficients += rotation.coeffs();
  }

  translation /= static_cast<double>(result_window_.size());

  Eigen::Quaterniond rotation;
  if (rotation_coefficients.norm() > std::numeric_limits<double>::epsilon()) {
    rotation.coeffs() = rotation_coefficients;
    rotation.normalize();
  } else {
    rotation = reference.normalized();
  }

  Eigen::Isometry3d average = Eigen::Isometry3d::Identity();
  average.translation() = translation;
  average.linear() = rotation.toRotationMatrix();
  return average;
}

void SmallGicpRelocalizationNode::resetResultFilter()
{
  result_window_.clear();
}

bool SmallGicpRelocalizationNode::shouldResetResultFilter(
  const Eigen::Isometry3d & raw_result) const
{
  if (result_window_.empty()) {
    return false;
  }

  const auto & last_result = result_window_.back();
  const auto translation_delta = (raw_result.translation() - last_result.translation()).norm();
  const auto rotation_delta = std::abs(
    Eigen::AngleAxisd(last_result.rotation().transpose() * raw_result.rotation()).angle());

  return translation_delta > sliding_window_filter_reset_translation_threshold_ ||
         rotation_delta > sliding_window_filter_reset_rotation_threshold_;
}

void SmallGicpRelocalizationNode::registeredPcdCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  last_scan_time_ = msg->header.stamp;
  current_scan_frame_id_ = msg->header.frame_id;

  if (!relocalization_enabled_) {
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *scan);

  if (!map_bounds_filter_enabled_ || !map_bounds_valid_) {
    *accumulated_cloud_ += *scan;
    return;
  }

  const auto filtered = filterScanByMapBounds(*scan, previous_result_t_);
  if (static_cast<int>(filtered->points.size()) < map_bounds_filter_min_points_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Map bounds filter kept %zu / %zu points, using unfiltered scan instead.",
      filtered->points.size(), scan->points.size());
    *accumulated_cloud_ += *scan;
    return;
  }

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 2000, "Map bounds filter kept %zu / %zu points.",
    filtered->points.size(), scan->points.size());
  *accumulated_cloud_ += *filtered;
}

void SmallGicpRelocalizationNode::performRegistration()
{
  if (!relocalization_enabled_) {
    accumulated_cloud_->clear();
    return;
  }

  if (accumulated_cloud_->empty()) {
    RCLCPP_WARN(this->get_logger(), "No accumulated points to process.");
    return;
  }

  source_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *accumulated_cloud_, registered_leaf_size_);

  small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);

  source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    source_, small_gicp::KdTreeBuilderOMP(num_threads_));

  if (!source_ || !source_tree_) {
    return;
  }

  register_->reduction.num_threads = num_threads_;
  register_->rejector.max_dist_sq = max_dist_sq_;
  register_->optimizer.max_iterations = 20;

  auto result = register_->align(*target_, *source_, *target_tree_, previous_result_t_);

  if (result.converged) {
    const auto last_result = previous_result_t_;
    const auto new_result = result.T_target_source;
    const auto translation_delta = (new_result.translation() - last_result.translation()).norm();
    const auto rotation_delta = std::abs(
      Eigen::AngleAxisd(last_result.rotation().transpose() * new_result.rotation()).angle());

    if (translation_delta < stable_translation_threshold_ &&
      rotation_delta < stable_rotation_threshold_)
    {
      ++stable_count_;
    } else {
      stable_count_ = 0;
    }

    previous_result_t_ = new_result;
    if (sliding_window_filter_enabled_) {
      result_t_ = filterRegistrationResult(new_result);
    } else {
      result_t_ = new_result;
      resetResultFilter();
    }

    if (auto_disable_relocalization_ && stable_count_ >= stable_required_count_) {
      const auto stable_count = stable_count_;
      RCLCPP_INFO(
        this->get_logger(),
        "Relocalization is stable for %d iterations, disabling registration and keeping TF output.",
        stable_count);
      relocalization_enabled_ = false;
      accumulated_cloud_->clear();
      this->set_parameter(rclcpp::Parameter("relocalization_enabled", false));
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "GICP did not converge.");
  }

  accumulated_cloud_->clear();
}

void SmallGicpRelocalizationNode::publishTransform()
{
  if (result_t_.matrix().isZero()) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  // `+ 0.1` means transform into future. according to https://robotics.stackexchange.com/a/96615
  transform_stamped.header.stamp = this->now() + rclcpp::Duration::from_seconds(0.5);
  transform_stamped.header.frame_id = map_frame_;
  transform_stamped.child_frame_id = odom_frame_;

  const Eigen::Vector3d translation = result_t_.translation();
  const Eigen::Quaterniond rotation(result_t_.rotation());

  transform_stamped.transform.translation.x = translation.x();
  transform_stamped.transform.translation.y = translation.y();
  transform_stamped.transform.translation.z = translation.z();
  transform_stamped.transform.rotation.x = rotation.x();
  transform_stamped.transform.rotation.y = rotation.y();
  transform_stamped.transform.rotation.z = rotation.z();
  transform_stamped.transform.rotation.w = rotation.w();

  tf_broadcaster_->sendTransform(transform_stamped);
}

void SmallGicpRelocalizationNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(), "Received initial pose: [x: %f, y: %f, z: %f]", msg->pose.pose.position.x,
    msg->pose.pose.position.y, msg->pose.pose.position.z);

  Eigen::Isometry3d map_to_robot_base = Eigen::Isometry3d::Identity();
  map_to_robot_base.translation() << msg->pose.pose.position.x, msg->pose.pose.position.y,
    msg->pose.pose.position.z;
  map_to_robot_base.linear() = Eigen::Quaterniond(
                                 msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                 msg->pose.pose.orientation.y, msg->pose.pose.orientation.z)
                                 .toRotationMatrix();

  try {
    auto transform =
      tf_buffer_->lookupTransform(robot_base_frame_, current_scan_frame_id_, tf2::TimePointZero);
    Eigen::Isometry3d robot_base_to_odom = tf2::transformToEigen(transform.transform);
    Eigen::Isometry3d map_to_odom = map_to_robot_base * robot_base_to_odom;

    previous_result_t_ = result_t_ = map_to_odom;
    stable_count_ = 0;
    resetResultFilter();
    if (!relocalization_enabled_) {
      relocalization_enabled_ = true;
      this->set_parameter(rclcpp::Parameter("relocalization_enabled", true));
      RCLCPP_INFO(this->get_logger(), "Relocalization enabled by initial pose reset.");
    }
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Could not transform initial pose from %s to %s: %s",
      robot_base_frame_.c_str(), current_scan_frame_id_.c_str(), ex.what());
  }
}

}  // namespace small_gicp_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_relocalization::SmallGicpRelocalizationNode)
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

#include "small_gicp_relocalization/transformed_pcd_publisher.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>

#include "pcl/PCLPointField.h"
#include "pcl/io/pcd_io.h"
#include "pcl_conversions/pcl_conversions.h"

namespace small_gicp_relocalization
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
}  // namespace

TransformedPcdPublisherNode::TransformedPcdPublisherNode(const rclcpp::NodeOptions & options)
: Node("transformed_pcd_publisher", options),
  cloud_loaded_(false),
  transformed_cloud_ready_(false)
{
  this->declare_parameter("pcd_file", "");
  this->declare_parameter("frame_id", "map");
  this->declare_parameter("topic_name", "prior_pcd_transformed");
  this->declare_parameter("offset_x", 0.0);
  this->declare_parameter("offset_y", 0.0);
  this->declare_parameter("offset_z", 0.0);
  this->declare_parameter("roll", 0.0);
  this->declare_parameter("pitch", 0.0);
  this->declare_parameter("yaw", 0.0);
  this->declare_parameter("angles_in_degrees", false);
  this->declare_parameter("publish_period_sec", 0.0);
  this->declare_parameter("output_pcd_file", "");

  this->get_parameter("pcd_file", pcd_file_);
  this->get_parameter("frame_id", frame_id_);
  this->get_parameter("topic_name", topic_name_);
  this->get_parameter("offset_x", offset_x_);
  this->get_parameter("offset_y", offset_y_);
  this->get_parameter("offset_z", offset_z_);
  this->get_parameter("roll", roll_);
  this->get_parameter("pitch", pitch_);
  this->get_parameter("yaw", yaw_);
  this->get_parameter("angles_in_degrees", angles_in_degrees_);
  this->get_parameter("publish_period_sec", publish_period_sec_);
  this->get_parameter("output_pcd_file", output_pcd_file_);

  rclcpp::QoS qos(1);
  qos.reliable();
  qos.transient_local();
  cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name_, qos);
  save_service_ = this->create_service<std_srvs::srv::Trigger>(
    "save_transformed_pcd",
    std::bind(
      &TransformedPcdPublisherNode::saveTransformedPcd, this, std::placeholders::_1,
      std::placeholders::_2));

  if (!pcd_file_.empty() && loadPcdFile(pcd_file_, original_cloud_)) {
    cloud_loaded_ = true;
    updateTransformedCloud();
    publishCloud();
  } else {
    RCLCPP_WARN(this->get_logger(), "No valid pcd_file was provided, waiting for parameter update.");
  }

  configureTimer();
  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&TransformedPcdPublisherNode::parametersCallback, this, std::placeholders::_1));
}

bool TransformedPcdPublisherNode::loadPcdFile(
  const std::string & pcd_file, pcl::PCLPointCloud2 & cloud) const
{
  if (pcd_file.empty()) {
    RCLCPP_ERROR(this->get_logger(), "pcd_file is empty.");
    return false;
  }

  if (pcl::io::loadPCDFile(pcd_file, cloud) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", pcd_file.c_str());
    return false;
  }

  RCLCPP_INFO(
    this->get_logger(), "Loaded PCD file: %s, width: %u, height: %u", pcd_file.c_str(),
    cloud.width, cloud.height);
  return true;
}

Eigen::Affine3f TransformedPcdPublisherNode::createTransform() const
{
  double roll = roll_;
  double pitch = pitch_;
  double yaw = yaw_;
  if (angles_in_degrees_) {
    roll = roll * kPi / 180.0;
    pitch = pitch * kPi / 180.0;
    yaw = yaw * kPi / 180.0;
  }

  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << static_cast<float>(offset_x_), static_cast<float>(offset_y_),
    static_cast<float>(offset_z_);
  transform.linear() =
    (Eigen::AngleAxisf(static_cast<float>(yaw), Eigen::Vector3f::UnitZ()) *
    Eigen::AngleAxisf(static_cast<float>(pitch), Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(static_cast<float>(roll), Eigen::Vector3f::UnitX()))
      .toRotationMatrix();
  return transform;
}

void TransformedPcdPublisherNode::updateTransformedCloud()
{
  if (!cloud_loaded_) {
    transformed_cloud_ready_ = false;
    return;
  }

  int x_offset = -1;
  int y_offset = -1;
  int z_offset = -1;
  for (const auto & field : original_cloud_.fields) {
    if (field.datatype != pcl::PCLPointField::FLOAT32 || field.count != 1) {
      continue;
    }

    if (field.name == "x") {
      x_offset = static_cast<int>(field.offset);
    } else if (field.name == "y") {
      y_offset = static_cast<int>(field.offset);
    } else if (field.name == "z") {
      z_offset = static_cast<int>(field.offset);
    }
  }

  if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
    RCLCPP_ERROR(this->get_logger(), "PCD must contain float32 x, y and z fields.");
    transformed_cloud_ready_ = false;
    return;
  }

  transformed_cloud_ = original_cloud_;
  const auto transform = createTransform();
  for (std::size_t point_idx = 0; point_idx < transformed_cloud_.width * transformed_cloud_.height;
    ++point_idx)
  {
    auto * point_data = transformed_cloud_.data.data() + point_idx * transformed_cloud_.point_step;
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
    std::memcpy(&x, point_data + x_offset, sizeof(float));
    std::memcpy(&y, point_data + y_offset, sizeof(float));
    std::memcpy(&z, point_data + z_offset, sizeof(float));

    const Eigen::Vector3f transformed_point = transform * Eigen::Vector3f(x, y, z);
    x = transformed_point.x();
    y = transformed_point.y();
    z = transformed_point.z();
    std::memcpy(point_data + x_offset, &x, sizeof(float));
    std::memcpy(point_data + y_offset, &y, sizeof(float));
    std::memcpy(point_data + z_offset, &z, sizeof(float));
  }

  transformed_cloud_ready_ = true;
}

void TransformedPcdPublisherNode::publishCloud()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!transformed_cloud_ready_) {
    return;
  }

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl_conversions::fromPCL(transformed_cloud_, cloud_msg);
  cloud_msg.header.stamp = this->now();
  cloud_msg.header.frame_id = frame_id_;
  cloud_pub_->publish(cloud_msg);
}

void TransformedPcdPublisherNode::configureTimer()
{
  publish_timer_.reset();
  if (publish_period_sec_ <= 0.0) {
    return;
  }

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(publish_period_sec_));
  publish_timer_ = this->create_wall_timer(period, std::bind(&TransformedPcdPublisherNode::publishCloud, this));
}

rcl_interfaces::msg::SetParametersResult TransformedPcdPublisherNode::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  std::string next_pcd_file = pcd_file_;
  std::string next_frame_id = frame_id_;
  std::string next_topic_name = topic_name_;
  std::string next_output_pcd_file = output_pcd_file_;
  double next_offset_x = offset_x_;
  double next_offset_y = offset_y_;
  double next_offset_z = offset_z_;
  double next_roll = roll_;
  double next_pitch = pitch_;
  double next_yaw = yaw_;
  bool next_angles_in_degrees = angles_in_degrees_;
  double next_publish_period_sec = publish_period_sec_;

  for (const auto & parameter : parameters) {
    const auto & name = parameter.get_name();
    if (name == "pcd_file") {
      next_pcd_file = parameter.as_string();
    } else if (name == "frame_id") {
      next_frame_id = parameter.as_string();
    } else if (name == "topic_name") {
      next_topic_name = parameter.as_string();
    } else if (name == "offset_x") {
      next_offset_x = parameter.as_double();
    } else if (name == "offset_y") {
      next_offset_y = parameter.as_double();
    } else if (name == "offset_z") {
      next_offset_z = parameter.as_double();
    } else if (name == "roll") {
      next_roll = parameter.as_double();
    } else if (name == "pitch") {
      next_pitch = parameter.as_double();
    } else if (name == "yaw") {
      next_yaw = parameter.as_double();
    } else if (name == "angles_in_degrees") {
      next_angles_in_degrees = parameter.as_bool();
    } else if (name == "publish_period_sec") {
      next_publish_period_sec = parameter.as_double();
    } else if (name == "output_pcd_file") {
      next_output_pcd_file = parameter.as_string();
    }
  }

  if (next_topic_name != topic_name_) {
    result.successful = false;
    result.reason = "topic_name only takes effect at startup.";
    return result;
  }

  if (next_publish_period_sec < 0.0) {
    result.successful = false;
    result.reason = "publish_period_sec must be greater than or equal to 0.0.";
    return result;
  }

  pcl::PCLPointCloud2 next_cloud;
  bool pcd_file_changed = next_pcd_file != pcd_file_;
  if (pcd_file_changed && !loadPcdFile(next_pcd_file, next_cloud)) {
    result.successful = false;
    result.reason = "Failed to load pcd_file.";
    return result;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    pcd_file_ = next_pcd_file;
    frame_id_ = next_frame_id;
    output_pcd_file_ = next_output_pcd_file;
    offset_x_ = next_offset_x;
    offset_y_ = next_offset_y;
    offset_z_ = next_offset_z;
    roll_ = next_roll;
    pitch_ = next_pitch;
    yaw_ = next_yaw;
    angles_in_degrees_ = next_angles_in_degrees;
    publish_period_sec_ = next_publish_period_sec;

    if (pcd_file_changed) {
      original_cloud_ = next_cloud;
      cloud_loaded_ = true;
    }
    updateTransformedCloud();
  }

  configureTimer();
  publishCloud();
  return result;
}

void TransformedPcdPublisherNode::saveTransformedPcd(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  const std_srvs::srv::Trigger::Response::SharedPtr response)
{
  (void)request;

  std::lock_guard<std::mutex> lock(mutex_);
  if (!transformed_cloud_ready_) {
    response->success = false;
    response->message = "No transformed cloud is ready.";
    return;
  }

  if (output_pcd_file_.empty()) {
    response->success = false;
    response->message = "output_pcd_file is empty.";
    return;
  }

  if (output_pcd_file_ == pcd_file_) {
    response->success = false;
    response->message = "output_pcd_file must be different from pcd_file.";
    return;
  }

  pcl::PCDWriter writer;
  if (writer.write(
      output_pcd_file_, transformed_cloud_, Eigen::Vector4f::Zero(),
      Eigen::Quaternionf::Identity(), true) < 0)
  {
    response->success = false;
    response->message = "Failed to save transformed PCD: " + output_pcd_file_;
    return;
  }

  response->success = true;
  response->message = "Saved transformed PCD: " + output_pcd_file_;
}

}  // namespace small_gicp_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_relocalization::TransformedPcdPublisherNode)

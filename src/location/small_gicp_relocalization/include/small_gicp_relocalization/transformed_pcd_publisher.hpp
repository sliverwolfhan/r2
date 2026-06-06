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

#ifndef SMALL_GICP_RELOCALIZATION__TRANSFORMED_PCD_PUBLISHER_HPP_
#define SMALL_GICP_RELOCALIZATION__TRANSFORMED_PCD_PUBLISHER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "Eigen/Geometry"
#include "pcl/PCLPointCloud2.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace small_gicp_relocalization
{

class TransformedPcdPublisherNode : public rclcpp::Node
{
public:
  explicit TransformedPcdPublisherNode(const rclcpp::NodeOptions & options);

private:
  bool loadPcdFile(const std::string & pcd_file, pcl::PCLPointCloud2 & cloud) const;
  void updateTransformedCloud();
  void publishCloud();
  void configureTimer();
  Eigen::Affine3f createTransform() const;
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);
  void saveTransformedPcd(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    const std_srvs::srv::Trigger::Response::SharedPtr response);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  std::mutex mutex_;
  pcl::PCLPointCloud2 original_cloud_;
  pcl::PCLPointCloud2 transformed_cloud_;
  bool cloud_loaded_;
  bool transformed_cloud_ready_;

  std::string pcd_file_;
  std::string frame_id_;
  std::string topic_name_;
  std::string output_pcd_file_;
  double offset_x_;
  double offset_y_;
  double offset_z_;
  double roll_;
  double pitch_;
  double yaw_;
  bool angles_in_degrees_;
  double publish_period_sec_;
};

}  // namespace small_gicp_relocalization

#endif  // SMALL_GICP_RELOCALIZATION__TRANSFORMED_PCD_PUBLISHER_HPP_

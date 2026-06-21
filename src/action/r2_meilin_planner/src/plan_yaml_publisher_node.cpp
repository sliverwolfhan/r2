#include <algorithm>
#include <cctype>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/plan.hpp"
#include "robot_interfaces/msg/plan_step.hpp"
#include "yaml-cpp/yaml.h"

namespace {
std::string to_upper(std::string s)
{
  std::transform(
    s.begin(), s.end(), s.begin(),
    [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
  return s;
}

uint8_t parse_type(const YAML::Node & step)
{
  using PS = robot_interfaces::msg::PlanStep;
  if (!step["type"]) {
    return PS::TYPE_MOVE;
  }
  if (step["type"].IsScalar()) {
    const std::string t = to_upper(step["type"].as<std::string>());
    if (t == "MOVE") {
      return PS::TYPE_MOVE;
    }
    if (t == "PICK") {
      return PS::TYPE_PICK;
    }
    if (t == "PUSH") {
      return PS::TYPE_PUSH;
    }
  }
  return step["type"].as<uint8_t>(PS::TYPE_MOVE);
}

uint8_t parse_stair_dir(const YAML::Node & step)
{
  using PS = robot_interfaces::msg::PlanStep;
  if (!step["stair_dir"]) {
    return PS::STAIR_NONE;
  }
  if (step["stair_dir"].IsScalar()) {
    const std::string d = to_upper(step["stair_dir"].as<std::string>());
    if (d == "UP") {
      return PS::STAIR_UP;
    }
    if (d == "DOWN") {
      return PS::STAIR_DOWN;
    }
    if (d == "NONE") {
      return PS::STAIR_NONE;
    }
  }
  return step["stair_dir"].as<uint8_t>(PS::STAIR_NONE);
}
}  // namespace

class PlanYamlPublisherNode : public rclcpp::Node
{
public:
  PlanYamlPublisherNode()
  : Node("plan_yaml_publisher_node")
  {
    const std::string yaml_path =
      ament_index_cpp::get_package_share_directory("r2_meilin_planner") +
      "/config/plan_meilin.yaml";

    plan_msg_ = load_plan(yaml_path);
    pub_ = this->create_publisher<robot_interfaces::msg::Plan>(
      "/r2_planner/plan", rclcpp::QoS(1).transient_local().reliable());

    plan_msg_.header.stamp = this->get_clock()->now();
    pub_->publish(plan_msg_);

    RCLCPP_INFO(
      get_logger(), "Loaded %zu steps from %s, published once to /r2_planner/plan",
      plan_msg_.steps.size(), yaml_path.c_str());
  }

private:
  robot_interfaces::msg::Plan load_plan(const std::string & yaml_path)
  {
    robot_interfaces::msg::Plan out;
    YAML::Node root = YAML::LoadFile(yaml_path);

    out.notes = root["notes"] ? root["notes"].as<std::string>() : std::string();
    if (!root["steps"] || !root["steps"].IsSequence()) {
      throw std::runtime_error("plan_meilin1.yaml missing key 'steps' sequence");
    }

    out.steps.reserve(root["steps"].size());
    for (const auto & s : root["steps"]) {
      robot_interfaces::msg::PlanStep step;
      step.type = parse_type(s);
      step.target_id = s["target_id"].as<int32_t>(0);
      step.from_id = s["from_id"].as<int32_t>(0);

      if (s["prep_pose"]) {
        step.prep_pose.x = s["prep_pose"]["x"].as<double>(0.0);
        step.prep_pose.y = s["prep_pose"]["y"].as<double>(0.0);
        step.prep_pose.theta = s["prep_pose"]["theta"].as<double>(0.0);
      }

      step.abs_dh = s["abs_dh"].as<double>(0.0);
      step.stair_dir = parse_stair_dir(s);
      step.cube_x = s["cube_x"].as<double>(0.0);
      step.cube_y = s["cube_y"].as<double>(0.0);
      step.block_height = s["block_height"].as<double>(0.0);
      step.grasp_yaw = s["grasp_yaw"].as<double>(0.0);

      out.steps.push_back(step);
    }
    return out;
  }

  robot_interfaces::msg::Plan plan_msg_;
  rclcpp::Publisher<robot_interfaces::msg::Plan>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanYamlPublisherNode>());
  rclcpp::shutdown();
  return 0;
}

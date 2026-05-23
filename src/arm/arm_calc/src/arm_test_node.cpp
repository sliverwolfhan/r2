#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <array>
#include <string>
#include <vector>

namespace arm_calc {

namespace {

constexpr std::size_t kJointCount = 6;
constexpr char kJointSpaceTargetTopic[] = "joint_space_target";
constexpr char kVisualTargetTopic[] = "visual_target_pose";
constexpr std::array<const char*, kJointCount> kJointParameterNames = {
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
constexpr std::array<const char*, 3> kPosePositionParameterNames = {"pose_x", "pose_y", "pose_z"};
constexpr std::array<const char*, 3> kPoseEulerParameterNames = {"pose_roll", "pose_pitch", "pose_yaw"};

class ArmTestNode : public rclcpp::Node {
public:
    ArmTestNode()
        : rclcpp::Node("arm_test_node") {
        declare_parameters();
        joint_target_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(kJointSpaceTargetTopic, 10);
        pose_target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(kVisualTargetTopic, 10);

        param_callback_ = this->add_on_set_parameters_callback(
            std::bind(&ArmTestNode::on_parameters_changed, this, std::placeholders::_1));
    }

private:
    void declare_parameters() {
        for (const auto* name : kJointParameterNames) {
            this->declare_parameter<double>(name, 0.0);
        }
        this->declare_parameter<double>("pose_x", 0.7);
        this->declare_parameter<double>("pose_y", 0.0);
        this->declare_parameter<double>("pose_z", 0.15);
        this->declare_parameter<double>("pose_roll", 0.0);
        this->declare_parameter<double>("pose_pitch", 0.0);
        this->declare_parameter<double>("pose_yaw", 0.0);
        this->declare_parameter<std::string>("pose_frame_id", "world");
        this->declare_parameter<bool>("publish_joint_target", false);
        this->declare_parameter<bool>("publish_pose_target", false);
    }

    rcl_interfaces::msg::SetParametersResult on_parameters_changed(const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto& param : params) {
            if (param.get_name() == "publish_joint_target" &&  param.as_bool()) {
                publish_joint_target();
            } else if (param.get_name() == "publish_pose_target" &&  param.as_bool()) {
                publish_pose_target();
            }
        }

        return result;
    }

    void publish_joint_target() {
        std_msgs::msg::Float64MultiArray msg;
        msg.data.reserve(kJointCount);
        for (const auto* name : kJointParameterNames) {
            msg.data.push_back(this->get_parameter(name).as_double());
        }
        joint_target_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published joint target with %zu joints", msg.data.size());
    }

    void publish_pose_target() {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = this->get_parameter("pose_frame_id").as_string();
        msg.pose.position.x = this->get_parameter("pose_x").as_double();
        msg.pose.position.y = this->get_parameter("pose_y").as_double();
        msg.pose.position.z = this->get_parameter("pose_z").as_double();

        tf2::Quaternion quaternion;
        quaternion.setRPY(
            this->get_parameter("pose_roll").as_double(),
            this->get_parameter("pose_pitch").as_double(),
            this->get_parameter("pose_yaw").as_double());
        msg.pose.orientation.w = quaternion.w();
        msg.pose.orientation.x = quaternion.x();
        msg.pose.orientation.y = quaternion.y();
        msg.pose.orientation.z = quaternion.z();
        pose_target_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published pose target in frame %s", msg.header.frame_id.c_str());
    }


    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_target_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_target_pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
};

}  // namespace

}  // namespace arm_calc

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<arm_calc::ArmTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

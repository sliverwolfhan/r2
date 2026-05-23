/**
 * @file publish_odom_map.cpp
 * @brief 发布机器人在map坐标系下的位姿，并记录与仿真真值的对比
 */

#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <iomanip>
#include <string>
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_msgs/msg/tf_message.hpp>


class Publish_map_odom : public rclcpp::Node
{
public:
    Publish_map_odom() : Node("publish_odom_map")
    {
        // 声明参数
        this->declare_parameter<std::string>("robot_name", "AT_R2");
        this->declare_parameter<std::string>("robot_base_frame", "base_footprint");
        this->declare_parameter<std::string>("global_frame", "map");
        this->declare_parameter<std::string>("csv_file_path", "/home/zk/at_rc2026/rcu_ws/pose_comparison.csv");
        
        robot_name_ = this->get_parameter("robot_name").as_string();
        robot_base_frame_ = this->get_parameter("robot_base_frame").as_string();
        global_frame_ = this->get_parameter("global_frame").as_string();
        csv_file_path_ = this->get_parameter("csv_file_path").as_string();

        if (robot_name_.empty()) {
            robot_name_ = "AT_R2";
        }

        // 初始化位姿数据
        odom_x_ = odom_y_ = odom_z_ = odom_yaw_ = 0.0;
        gt_x_ = gt_y_ = gt_z_ = gt_yaw_ = 0.0;
        gt_received_ = false;

        // 创建发布者
        std::string odom_map_topic = "/" + robot_name_ + "/odom_map";
        std::string gt_topic = "/" + robot_name_ + "/ground_truth";
        
        odom_map_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_map_topic, 10);
        ground_truth_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(gt_topic, 10);

        // TF Buffer
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        
        // 订阅带命名空间的 TF 话题（机器人本体坐标系）
        std::string tf_topic = "/" + robot_name_ + "/tf";
        std::string tf_static_topic = "/" + robot_name_ + "/tf_static";
        
        // tf_static 需要 transient_local QoS 才能接收历史消息
        rclcpp::QoS tf_static_qos(100);
        tf_static_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
        
        tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            tf_topic, 10, [this](tf2_msgs::msg::TFMessage::SharedPtr msg) {
                for (const auto & transform : msg->transforms) {
                    tf_buffer_->setTransform(transform, "default_authority", false);
                }
            });
        tf_static_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            tf_static_topic, tf_static_qos, [this](tf2_msgs::msg::TFMessage::SharedPtr msg) {
                for (const auto & transform : msg->transforms) {
                    tf_buffer_->setTransform(transform, "default_authority", true);
                }
            });

        // 订阅 Ignition 世界坐标真值（从 ros_gz_bridge 桥接）
        std::string world_poses_topic = "/" + robot_name_ + "/ground_truth/pose";
        world_poses_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            world_poses_topic, 10, 
            std::bind(&Publish_map_odom::worldPosesCallback, this, std::placeholders::_1));

        // 定时器：50ms 发布 odom_map
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&Publish_map_odom::publishOdomMap, this));
        
        // 定时器：1秒记录一次 CSV
        csv_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Publish_map_odom::recordToCSV, this));

        // 初始化 CSV 文件
        initCSV();

        RCLCPP_INFO(this->get_logger(), "节点启动");
        RCLCPP_INFO(this->get_logger(), "  odom_map 话题: %s", odom_map_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  ground_truth 话题: %s", gt_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  CSV 文件: %s", csv_file_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "  订阅世界位姿: %s", world_poses_topic.c_str());
    }

    ~Publish_map_odom()
    {
        if (csv_file_.is_open()) {
            csv_file_.close();
            RCLCPP_INFO(this->get_logger(), "CSV 文件已保存: %s", csv_file_path_.c_str());
        }
    }

private:
    void initCSV()
    {
        csv_file_.open(csv_file_path_);
        if (csv_file_.is_open()) {
            csv_file_ << "timestamp,odom_x,odom_y,odom_z,odom_yaw,gt_x,gt_y,gt_z,gt_yaw,error_x,error_y,error_z,error_yaw\n";
            RCLCPP_INFO(this->get_logger(), "CSV 文件已创建");
        } else {
            RCLCPP_ERROR(this->get_logger(), "无法创建 CSV 文件: %s", csv_file_path_.c_str());
        }
    }

    void worldPosesCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        for (const auto & transform : msg->transforms) {
            // 找到 AT_R2 模型的位姿
            if (transform.child_frame_id == robot_name_) {
                gt_x_ = transform.transform.translation.x;
                gt_y_ = transform.transform.translation.y;
                gt_z_ = transform.transform.translation.z;
                
                tf2::Quaternion q;
                tf2::fromMsg(transform.transform.rotation, q);
                double roll, pitch;
                tf2::Matrix3x3(q).getRPY(roll, pitch, gt_yaw_);
                
                gt_received_ = true;
                
                // 发布真值话题
                geometry_msgs::msg::PoseStamped gt_msg;
                gt_msg.header.stamp = this->get_clock()->now();
                gt_msg.header.frame_id = "world";
                gt_msg.pose.position.x = gt_x_;
                gt_msg.pose.position.y = gt_y_;
                gt_msg.pose.position.z = gt_z_;
                gt_msg.pose.orientation = transform.transform.rotation;
                ground_truth_pub_->publish(gt_msg);
                
                return;
            }
        }
    }


    bool getRobotPose(double & x, double & y, double & z, double & yaw)
    {
        try {
            auto transform = tf_buffer_->lookupTransform(
                global_frame_, robot_base_frame_,
                tf2::TimePointZero);

            x = transform.transform.translation.x;
            y = transform.transform.translation.y;
            z = transform.transform.translation.z;

            tf2::Quaternion q;
            tf2::fromMsg(transform.transform.rotation, q);
            double roll, pitch;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            return true;
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *this->get_clock(), 5000,
                "无法获取机器人位姿: %s", ex.what());
            return false;
        }
    }

    void publishOdomMap()
    {
        if (getRobotPose(odom_x_, odom_y_, odom_z_, odom_yaw_)) {
            nav_msgs::msg::Odometry msg;
            msg.header.stamp = this->get_clock()->now();
            msg.header.frame_id = "map";
            msg.child_frame_id = "base_footprint";

            msg.pose.pose.position.x = odom_x_;
            msg.pose.pose.position.y = odom_y_;
            msg.pose.pose.position.z = odom_z_;
            
            tf2::Quaternion q;
            q.setRPY(0, 0, odom_yaw_);
            msg.pose.pose.orientation = tf2::toMsg(q);

            odom_map_pub_->publish(msg);
        }
    }

    void recordToCSV()
    {
        if (!csv_file_.is_open()) return;
        
        double timestamp = this->get_clock()->now().seconds();
        double error_x = odom_x_ - gt_x_;
        double error_y = odom_y_ - gt_y_;
        double error_z = odom_z_ - gt_z_;
        double error_yaw = odom_yaw_ - gt_yaw_;
        
        // 归一化 yaw 误差到 [-pi, pi]
        while (error_yaw > M_PI) error_yaw -= 2 * M_PI;
        while (error_yaw < -M_PI) error_yaw += 2 * M_PI;
        
        csv_file_ << std::fixed << std::setprecision(6)
                  << timestamp << ","
                  << odom_x_ << "," << odom_y_ << "," << odom_z_ << "," << odom_yaw_ << ","
                  << gt_x_ << "," << gt_y_ << "," << gt_z_ << "," << gt_yaw_ << ","
                  << error_x << "," << error_y << "," << error_z << "," << error_yaw
                  << "\n";
        csv_file_.flush();
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "记录中... odom=(%.2f,%.2f,%.2f) gt=(%.2f,%.2f,%.2f) err=(%.3f,%.3f,%.3f)",
            odom_x_, odom_y_, odom_z_, gt_x_, gt_y_, gt_z_, error_x, error_y, error_z);
    }

    // 发布者
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_map_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ground_truth_pub_;

    // 订阅者
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr world_poses_sub_;

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr csv_timer_;

    // 参数
    std::string robot_name_;
    std::string robot_base_frame_;
    std::string global_frame_;
    std::string csv_file_path_;

    // odom_map 位姿
    double odom_x_, odom_y_, odom_z_, odom_yaw_;
    
    // 仿真真值位姿
    double gt_x_, gt_y_, gt_z_, gt_yaw_;
    bool gt_received_;

    // CSV 文件
    std::ofstream csv_file_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Publish_map_odom>());
    rclcpp::shutdown();
    return 0;
}
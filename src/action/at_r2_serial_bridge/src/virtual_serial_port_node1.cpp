#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "at_r2_serial_bridge/cdc_trans.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <cmath>

// 发送给下位机的速度数据包结构
#pragma pack(push, 1)
struct VelocityPacket {
    uint8_t header;    // 包头 0xAB
    float vx;          // x方向线速度 m/s
    float vy;          // y方向线速度 m/s  
    float omega;       // 角速度 rad/s
    uint8_t mode;      // 模式标志位 (0或1)
    uint8_t tail;      // 包尾 0xBA
};
#pragma pack(pop)

class VirtualSerialPortNode : public rclcpp::Node
{
public:
    VirtualSerialPortNode() : Node("virtual_serial_port_node"), running_(true), current_mode_(0), mode_send_once_(false)
    {
        // 声明参数
        this->declare_parameter<int>("usb_vid", 0x0483);  // STM32 默认VID
        this->declare_parameter<int>("usb_pid", 0x5740);  // CDC默认PID
        this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
        this->declare_parameter<int>("send_interval_ms", 20);  // 发送周期，降低到20ms避免缓冲区溢出
        this->declare_parameter<double>("rotation_threshold", 5.0);  // 旋转阈值，单位度
        
        // 获取参数
        usb_vid_ = static_cast<uint16_t>(this->get_parameter("usb_vid").as_int());
        usb_pid_ = static_cast<uint16_t>(this->get_parameter("usb_pid").as_int());
        std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
        send_interval_ms_ = this->get_parameter("send_interval_ms").as_int();
        rotation_threshold_ = this->get_parameter("rotation_threshold").as_double();
        
        // 初始化速度为0
        current_velocity_ = {0.0f, 0.0f, 0.0f};
        
        // 初始化TF2
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // 订阅自定义TF话题
        tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/AT_R2/tf", 10,
            std::bind(&VirtualSerialPortNode::tf_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "订阅TF话题: /AT_R2/tf");
        
        // 初始化CDC设备
        cdc_trans_ = std::make_unique<CDCTrans>();
        
        // 尝试打开USB设备
        if (cdc_trans_->open(usb_vid_, usb_pid_)) {
            RCLCPP_INFO(this->get_logger(), "USB-CDC设备打开成功 VID:0x%04X PID:0x%04X", usb_vid_, usb_pid_);
        } else {
            RCLCPP_WARN(this->get_logger(), "USB-CDC设备打开失败，将持续尝试重连");
        }
        
        // 注册接收回调
        cdc_trans_->regeiser_recv_cb([this](const uint8_t* data, int size) {
            this->on_data_received(data, size);
        });
        
        // 订阅cmd_vel话题
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/AT_R2/cmd_vel_nav2_result", 10,
            std::bind(&VirtualSerialPortNode::cmd_vel_callback, this, std::placeholders::_1));
        
        // 启动发送线程（独立线程，8ms周期）
        send_thread_ = std::thread(&VirtualSerialPortNode::send_thread_func, this);
        
        // 启动USB事件处理线程
        usb_thread_ = std::thread(&VirtualSerialPortNode::usb_thread_func, this);
        
        // 启动TF监听线程
        tf_thread_ = std::thread(&VirtualSerialPortNode::tf_thread_func, this);
        
        RCLCPP_INFO(this->get_logger(), "虚拟串口节点已启动，订阅话题: %s, 发送周期: %dms, 旋转阈值: %.1f度", 
            cmd_vel_topic.c_str(), send_interval_ms_, rotation_threshold_);
    }
    
    ~VirtualSerialPortNode()
    {
        running_ = false;
        if (send_thread_.joinable()) {
            send_thread_.join();
        }
        if (usb_thread_.joinable()) {
            usb_thread_.join();
        }
        if (tf_thread_.joinable()) {
            tf_thread_.join();
        }
    }

private:
    void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        // 将接收到的TF消息添加到buffer中
        for (const auto& transform : msg->transforms) {
            try {
                tf_buffer_->setTransform(transform, "default_authority", false);
            } catch (tf2::TransformException &ex) {
                RCLCPP_DEBUG(this->get_logger(), "TF设置失败: %s", ex.what());
            }
        }
    }
    
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // 只更新速度值，不在回调中发送
        std::lock_guard<std::mutex> lock(velocity_mutex_);
        current_velocity_.vx = static_cast<float>(msg->linear.x);
        current_velocity_.vy = static_cast<float>(msg->linear.y);
        current_velocity_.omega = static_cast<float>(msg->angular.z);
    }
    
    void send_thread_func()
    {
        RCLCPP_INFO(this->get_logger(), "发送线程已启动");
        
        int consecutive_failures = 0;  // 连续失败计数
        const int max_failures = 5;    // 最大连续失败次数
        
        while (running_) {
            auto now = std::chrono::system_clock::now();
            
            // 构建数据包
            VelocityPacket packet;
            packet.header = 0xAB;  // 包头
            {
                std::lock_guard<std::mutex> lock(velocity_mutex_);
                packet.vx = -(current_velocity_.vy);
                packet.vy = current_velocity_.vx;
                packet.omega = -(current_velocity_.omega);
                
                // 如果需要发送一次mode，则发送实际值，否则发送0
                if (mode_send_once_) {
                    packet.mode = current_mode_;
                    mode_send_once_ = false;  // 发送后清除标志
                    RCLCPP_INFO(this->get_logger(), "发送mode: %d", packet.mode);
                } else {
                    packet.mode = 0;  // 其余时间发送0
                }
            }
            packet.tail = 0xBA;  // 包尾
            // RCLCPP_INFO(this->get_logger(), "packet.vx:%.2f, packet.vx:%.2f, packet.vx:%.2f, packet.vx:%d, ", 
            // packet.vx, packet.vy,packet.omega,packet.mode);
            // 发送数据并检查结果
            int result = cdc_trans_->send_struct(packet);
            
            if (result < 0) {
                consecutive_failures++;
                if (consecutive_failures >= max_failures) {
                    RCLCPP_ERROR(this->get_logger(), 
                        "连续发送失败%d次，USB设备可能断开，等待重连...", consecutive_failures);
                    // 等待重连
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    consecutive_failures = 0;
                }
            } else {
                // 发送成功，重置失败计数
                if (consecutive_failures > 0) {
                    RCLCPP_INFO(this->get_logger(), "USB通信恢复正常");
                    consecutive_failures = 0;
                }
            }
            
            // 使用参数配置的发送周期
            std::this_thread::sleep_until(now + std::chrono::milliseconds(send_interval_ms_));
        }
        
        RCLCPP_INFO(this->get_logger(), "发送线程已退出");
    }
    
    void usb_thread_func()
    {
        RCLCPP_INFO(this->get_logger(), "USB事件处理线程已启动");
        
        while (running_) {
            cdc_trans_->process_once();
        }
        
        RCLCPP_INFO(this->get_logger(), "USB事件处理线程已退出");
    }
    
    void tf_thread_func()
    {
        RCLCPP_INFO(this->get_logger(), "TF监听线程已启动");
        
        auto last_print_time = std::chrono::system_clock::now();
        bool tf_available = false;
        
        while (running_) {
            try {
                // 查询从map到base_footprint的变换
                RCLCPP_INFO(this->get_logger(), "TF监听线程已启动1");
                geometry_msgs::msg::TransformStamped transform_stamped = 
                    tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
                RCLCPP_INFO(this->get_logger(), "TF监听线程已启动2");
                if (!tf_available) {
                    RCLCPP_INFO(this->get_logger(), "TF变换可用: map -> base_footprint");
                    tf_available = true;
                }
                
                // 提取四元数
                auto& q = transform_stamped.transform.rotation;
                tf2::Quaternion quaternion(q.x, q.y, q.z, q.w);
                // 转换为欧拉角
                double roll, pitch, yaw;
                tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
                
                // 将弧度转换为度数
                double pitch_degrees = pitch * 180.0 / M_PI;
                
                // 每隔一秒打印一次
                auto now = std::chrono::system_clock::now();
                if (std::chrono::duration_cast<std::chrono::seconds>(now - last_print_time).count() >= 1) {
                    RCLCPP_INFO(this->get_logger(), "Pitch角度: %.2f度", pitch_degrees);
                    last_print_time = now;
                }
                
                // 检查绕Y轴旋转是否大于阈值
                uint8_t new_mode = (std::abs(pitch_degrees) > rotation_threshold_) ? 2 : 1;
                
                // 更新模式（线程安全）
                {
                    std::lock_guard<std::mutex> lock(velocity_mutex_);
                    if (current_mode_ != new_mode) {
                        current_mode_ = new_mode;
                        mode_send_once_ = true;  // 标记需要发送一次
                        uint8_t mode_value = current_mode_.load();  // 读取atomic值
                        RCLCPP_INFO(this->get_logger(), "模式切换: %d (pitch: %.2f度)", 
                                   mode_value, pitch_degrees);
                    }
                }
                
            } catch (tf2::TransformException &ex) {
                if (tf_available) {
                    RCLCPP_WARN(this->get_logger(), "TF查询失败: %s", ex.what());
                    tf_available = false;
                } else {
                    // 首次启动时每5秒打印一次等待信息
                    auto now = std::chrono::system_clock::now();
                    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_print_time).count() >= 5) {
                        RCLCPP_WARN(this->get_logger(), "等待TF变换: map -> base_footprint (%s)", ex.what());
                        last_print_time = now;
                    }
                }
            }
            
            // 10Hz频率检查TF
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        RCLCPP_INFO(this->get_logger(), "TF监听线程已退出");
    }
    
    void on_data_received(const uint8_t* data, int size)
    {
        RCLCPP_DEBUG(this->get_logger(), "收到下位机数据，长度: %d", size);
        (void)data;
    }
    
    // 当前速度结构
    struct Velocity {
        float vx;
        float vy;
        float omega;
    };
    
    std::unique_ptr<CDCTrans> cdc_trans_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
    
    // TF2相关
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    std::thread send_thread_;
    std::thread usb_thread_;
    std::thread tf_thread_;
    std::atomic<bool> running_;
    
    std::mutex velocity_mutex_;
    Velocity current_velocity_;
    std::atomic<uint8_t> current_mode_;
    bool mode_send_once_;  // 标志位：是否需要发送一次mode
    
    uint16_t usb_vid_;
    uint16_t usb_pid_;
    int send_interval_ms_;
    double rotation_threshold_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VirtualSerialPortNode>());
    rclcpp::shutdown();
    return 0;
}

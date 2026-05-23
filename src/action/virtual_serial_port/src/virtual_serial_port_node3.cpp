#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include "virtual_serial_port/cdc_trans.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <thread>
#include <mutex>
#include <atomic>

// 爬楼梯动作类型
enum class ClimbAction : uint8_t {
    NONE = 0,      // 无动作
    CLIMB = 1,     // 上楼梯
    DESCEND = 2    // 下楼梯
};

// 发送给下位机的速度数据包结构
#pragma pack(push, 1)
struct VelocityPacket {
    uint8_t header;        // 包头 0xAB
    float vx;              // x方向线速度 m/s
    float vy;              // y方向线速度 m/s  
    float omega;           // 角速度 rad/s
    uint8_t mode;          // 模式标志位 (0或1)
    uint8_t climb_action;  // 爬楼梯动作类型 (0=无, 1=上, 2=下)
    uint8_t climb_height;  // 爬楼梯高度 (0=无, 1=200mm, 2=400mm)
    uint8_t tail;          // 包尾 0xBA
};
#pragma pack(pop)

// 从下位机接收的状态数据包结构
#pragma pack(push, 1)
struct StatusPacket {
    uint8_t header;          // 包头 0xAB
    uint8_t climber_status;  // 爬楼梯状态 (0=空闲, 1=执行中, 2=成功, 3=失败)
    uint8_t tail;            // 包尾 0xBA
};
#pragma pack(pop)

class VirtualSerialPortNode : public rclcpp::Node
{
public:
    VirtualSerialPortNode() : Node("virtual_serial_port_node"), running_(true), current_mode_(0), mode_send_once_(false), climb_send_once_(false)
    {
        // 声明参数
        this->declare_parameter<int>("usb_vid", 0x0483);  // STM32 默认VID
        this->declare_parameter<int>("usb_pid", 0x5740);  // CDC默认PID
        this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
        this->declare_parameter<int>("send_interval_ms", 8);  // 发送周期
        this->declare_parameter<double>("zone_x_min", 9.0);
        this->declare_parameter<double>("zone_x_max", 10.5);
        this->declare_parameter<double>("zone_y_min", -0.5);
        this->declare_parameter<double>("zone_y_max", 1.0);
        
        // 获取参数
        usb_vid_ = static_cast<uint16_t>(this->get_parameter("usb_vid").as_int());
        usb_pid_ = static_cast<uint16_t>(this->get_parameter("usb_pid").as_int());
        std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
        send_interval_ms_ = this->get_parameter("send_interval_ms").as_int();
        zone_x_min_ = this->get_parameter("zone_x_min").as_double();
        zone_x_max_ = this->get_parameter("zone_x_max").as_double();
        zone_y_min_ = this->get_parameter("zone_y_min").as_double();
        zone_y_max_ = this->get_parameter("zone_y_max").as_double();
        
        // 初始化速度和爬楼梯状态
        current_velocity_ = {0.0f, 0.0f, 0.0f};
        current_climb_action_ = ClimbAction::NONE;
        current_climb_height_ = 0;
        
        // 初始化TF2
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/AT_R2/tf", 10,
            [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
                for (const auto & transform : msg->transforms) {
                    tf_buffer_->setTransform(transform, "default_authority", false);
                }
            });
        tf_static_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/AT_R2/tf_static", 10,
            [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
                for (const auto & transform : msg->transforms) {
                    tf_buffer_->setTransform(transform, "default_authority", true);
                }
            });

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
        
        // 订阅爬楼梯话题
        climb_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/AT_R2/climb_stair", 10,
            std::bind(&VirtualSerialPortNode::climb_callback, this, std::placeholders::_1));
        
        // 订阅下楼梯话题
        descend_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/AT_R2/descend_stair", 10,
            std::bind(&VirtualSerialPortNode::descend_callback, this, std::placeholders::_1));
        
        // 创建爬楼梯状态发布器
        climber_status_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/AT_R2/climber_status", 10);
        
        // 定时持续发布最新状态（10Hz）
        status_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() {
                auto msg = std_msgs::msg::Int32();
                msg.data = static_cast<int32_t>(latest_climber_status_);
                climber_status_pub_->publish(msg);
            });
        
        // 启动发送线程（独立线程，8ms周期）
        send_thread_ = std::thread(&VirtualSerialPortNode::send_thread_func, this);
        
        // 启动USB事件处理线程
        usb_thread_ = std::thread(&VirtualSerialPortNode::usb_thread_func, this);
        
        // 启动TF监听线程
        tf_thread_ = std::thread(&VirtualSerialPortNode::tf_thread_func, this);
        
        RCLCPP_INFO(this->get_logger(), "虚拟串口节点已启动，订阅话题: %s, 发送周期: %dms", 
            cmd_vel_topic.c_str(), send_interval_ms_);
        RCLCPP_INFO(this->get_logger(), "已订阅爬楼梯话题: /AT_R2/climb_stair, /AT_R2/descend_stair");
        RCLCPP_INFO(this->get_logger(), "已创建状态发布器: /AT_R2/climber_status");
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
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // 只更新速度值，不在回调中发送
        std::lock_guard<std::mutex> lock(velocity_mutex_);
        current_velocity_.vx = static_cast<float>(msg->linear.x);
        current_velocity_.vy = static_cast<float>(msg->linear.y);
        current_velocity_.omega = static_cast<float>(msg->angular.z);
    }
    
    void climb_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "收到爬楼梯指令: 高度 %.2f m ", msg->data);
        std::lock_guard<std::mutex> lock(velocity_mutex_);
        current_climb_action_ = ClimbAction::CLIMB;
        double h = msg->data;
        current_climb_height_ = (h <= 0.3) ? 1 : 2;
        climb_send_once_ = true;
        RCLCPP_INFO(this->get_logger(), "收到爬楼梯指令: 高度 %.2f m -> 编码 %d", msg->data, current_climb_height_);
    }
    
    void descend_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(velocity_mutex_);
        current_climb_action_ = ClimbAction::DESCEND;
        double h = msg->data;
        current_climb_height_ = (h <= 0.3) ? 1 : 2;
        climb_send_once_ = true;
        RCLCPP_INFO(this->get_logger(), "收到下楼梯指令: 高度 %.2f m -> 编码 %d", msg->data, current_climb_height_);
    }
    
    void send_thread_func()
    {
        using namespace std::chrono_literals;
        RCLCPP_INFO(this->get_logger(), "发送线程已启动");
        
        while (running_) {
            auto now = std::chrono::steady_clock::now();
            
            // 构建数据包
            VelocityPacket packet;
            packet.header = 0xAB;
            {
                std::lock_guard<std::mutex> lock(velocity_mutex_);
                packet.vx = current_velocity_.vx;
                packet.vy = current_velocity_.vy;
                packet.omega = current_velocity_.omega;
                RCLCPP_INFO(this->get_logger(), "发送速度: vx %.2f ", current_velocity_.vx);
                // climb：触发时发送一次实际值，其余时间发送0
                if (climb_send_once_) {
                    packet.climb_action = static_cast<uint8_t>(current_climb_action_);
                    packet.climb_height = current_climb_height_;
                    climb_send_once_ = false;
                    RCLCPP_INFO(this->get_logger(), "发送爬楼梯指令: action=%d, height=%d",
                        packet.climb_action, packet.climb_height);
                } else {
                    packet.climb_action = 0;
                    packet.climb_height = 0;
                }
                
                // mode：变化时发送一次实际值，其余时间发送0
                if (mode_send_once_) {
                    packet.mode = current_mode_;
                    mode_send_once_ = false;
                    RCLCPP_INFO(this->get_logger(), "发送mode: %d", packet.mode);
                } else {
                    packet.mode = 0;
                }
            }
            packet.tail = 0xBA;
            
            // 发送并检查结果
            if (!cdc_trans_->send_struct(packet)) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "发送速度指令失败");
            }
            
            std::this_thread::sleep_until(now + 8ms);
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
                geometry_msgs::msg::TransformStamped transform_stamped =
                    tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
                
                if (!tf_available) {
                    RCLCPP_INFO(this->get_logger(), "TF变换可用: map -> base_footprint");
                    tf_available = true;
                }
                
                double x = transform_stamped.transform.translation.x;
                double y = transform_stamped.transform.translation.y;
                
                auto now = std::chrono::system_clock::now();
                if (std::chrono::duration_cast<std::chrono::seconds>(now - last_print_time).count() >= 1) {
                    RCLCPP_INFO(this->get_logger(), "位置 x: %.3f m, y: %.3f m", x, y);
                    last_print_time = now;
                }
                
                bool in_zone = (x >= zone_x_min_ && x <= zone_x_max_ &&
                                y >= zone_y_min_ && y <= zone_y_max_);
                uint8_t new_mode = in_zone ? 2 : 1;
                
                {
                    std::lock_guard<std::mutex> lock(velocity_mutex_);
                    if (current_mode_ != new_mode) {
                        current_mode_ = new_mode;
                        mode_send_once_ = true;
                        RCLCPP_INFO(this->get_logger(), "模式切换: %d (位置 x: %.3f, y: %.3f, %s区域)",
                                   new_mode, x, y, in_zone ? "进入" : "离开");
                    }
                }
                
            } catch (tf2::TransformException &ex) {
                if (tf_available) {
                    RCLCPP_WARN(this->get_logger(), "TF查询失败: %s", ex.what());
                    tf_available = false;
                } else {
                    auto now = std::chrono::system_clock::now();
                    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_print_time).count() >= 5) {
                        RCLCPP_WARN(this->get_logger(), "等待TF变换: map -> base_footprint (%s)", ex.what());
                        last_print_time = now;
                    }
                }
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        RCLCPP_INFO(this->get_logger(), "TF监听线程已退出");
    }
    
    void on_data_received(const uint8_t* data, int size)
    {
        RCLCPP_DEBUG(this->get_logger(), "收到下位机数据，长度: %d", size);
        
        // 检查数据包大小是否匹配
        if (size == sizeof(StatusPacket)) {
            StatusPacket status;
            std::memcpy(&status, data, sizeof(StatusPacket));
            
            // 校验包头包尾
            if (status.header != 0xAB || status.tail != 0xBA) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "收到的数据包头尾校验失败: header=0x%02X tail=0x%02X",
                    status.header, status.tail);
                return;
            }
            
            // 更新缓存状态
            uint8_t new_status = status.climber_status;
            
            // 打印状态变化
            static uint8_t last_status = 0xFF;
            if (new_status != last_status) {
                const char* status_names[] = {"IDLE", "RUNNING", "SUCCESS", "FAILED"};
                if (new_status <= 3) {
                    RCLCPP_INFO(this->get_logger(), 
                        "爬楼梯状态更新: %s (%d)", 
                        status_names[new_status], new_status);
                }
                last_status = new_status;
                
                // 如果任务完成（成功或失败），清除爬楼梯指令
                if (new_status == 2 || new_status == 3) {
                    std::lock_guard<std::mutex> lock(velocity_mutex_);
                    current_climb_action_ = ClimbAction::NONE;
                    current_climb_height_ = 0;
                    RCLCPP_DEBUG(this->get_logger(), "已清除爬楼梯指令");
                }
            }
            latest_climber_status_ = new_status;
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "收到的数据包大小不匹配: 期望 %zu 字节，实际 %d 字节", 
                sizeof(StatusPacket), size);
        }
    }
    
    // 当前速度结构
    struct Velocity {
        float vx;
        float vy;
        float omega;
    };
    
    std::unique_ptr<CDCTrans> cdc_trans_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr climb_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr descend_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr climber_status_pub_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    
    std::thread send_thread_;
    std::thread usb_thread_;
    std::thread tf_thread_;
    std::atomic<bool> running_;
    
    std::mutex velocity_mutex_;
    Velocity current_velocity_;
    ClimbAction current_climb_action_;
    uint8_t current_climb_height_;
    std::atomic<uint8_t> current_mode_;
    bool mode_send_once_;
    bool climb_send_once_;
    std::atomic<uint8_t> latest_climber_status_{0};  // 缓存最新状态，默认IDLE
    
    uint16_t usb_vid_;
    uint16_t usb_pid_;
    int send_interval_ms_;
    double zone_x_min_;
    double zone_x_max_;
    double zone_y_min_;
    double zone_y_max_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VirtualSerialPortNode>());
    rclcpp::shutdown();
    return 0;
}

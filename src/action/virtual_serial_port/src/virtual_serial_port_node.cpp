#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include "virtual_serial_port/cdc_trans.hpp"
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
    uint8_t mode;          // 区模式 zone_mode (连续发送当前值)
    uint8_t grasp_cmd;     // 武器头爪子命令 (仅收到话题时发一次, 其余发0)
    uint8_t climb_action;  // 爬楼梯动作类型 (0=无, 1=上, 2=下)
    uint8_t climb_height;  // 爬楼梯高度 (0=无, 1=200mm, 2=400mm)
    uint8_t tail;          // 包尾 0xBA
};
#pragma pack(pop)

// 从下位机接收的状态数据包结构
#pragma pack(push, 1)
struct StatusPacket {
    uint8_t header;          // 包头 0xAB
    uint8_t climber_status;  // 爬楼梯状态 (1=开始执行, 2=执行完成)
    uint8_t tail;            // 包尾 0xBA
};
#pragma pack(pop)

class VirtualSerialPortNode : public rclcpp::Node
{
public:
    VirtualSerialPortNode() : Node("virtual_serial_port_node"), running_(true), climb_send_once_(false)
    {
        // 声明参数
        this->declare_parameter<int>("usb_vid", 0x0483);  // STM32 默认VID
        this->declare_parameter<int>("usb_pid", 0x5740);  // CDC默认PID
        this->declare_parameter<std::string>("cmd_vel_topic", "/AT_R2/cmd_vel_nav2_result");
        this->declare_parameter<int>("send_interval_ms", 8);  // 发送周期
        
        // 获取参数
        usb_vid_ = static_cast<uint16_t>(this->get_parameter("usb_vid").as_int());
        usb_pid_ = static_cast<uint16_t>(this->get_parameter("usb_pid").as_int());
        std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
        send_interval_ms_ = this->get_parameter("send_interval_ms").as_int();
        
        // 初始化速度和爬楼梯状态
        current_velocity_ = {0.0f, 0.0f, 0.0f};
        current_climb_action_ = ClimbAction::NONE;
        current_climb_height_ = 0;
        current_mode_ = 0;
        current_grasp_cmd_ = 0;
        grasp_send_once_ = false;
        
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
            cmd_vel_topic, 10,
            std::bind(&VirtualSerialPortNode::cmd_vel_callback, this, std::placeholders::_1));
        
        // 订阅爬楼梯话题
        climb_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/AT_R2/climb_stair", 10,
            std::bind(&VirtualSerialPortNode::climb_callback, this, std::placeholders::_1));
        
        // 订阅下楼梯话题
        descend_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/AT_R2/descend_stair", 10,
            std::bind(&VirtualSerialPortNode::descend_callback, this, std::placeholders::_1));
        
        // 订阅区模式话题 (latched, mode 连续发送)
        mode_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/AT_R2/zone_mode", rclcpp::QoS(1).transient_local().reliable(),
            std::bind(&VirtualSerialPortNode::mode_callback, this, std::placeholders::_1));
        
        // 订阅武器头爪子命令话题 (grasp_cmd 收到时发一次)
        // 用 volatile(默认) QoS: 避免本节点重启时收到 latched 历史指令而误触发一次抓取
        grasp_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/AT_R2/head_gripper_cmd", 10,
            std::bind(&VirtualSerialPortNode::grasp_callback, this, std::placeholders::_1));
        
        // 创建爬楼梯状态发布器
        climber_status_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/AT_R2/climber_status", 10);
        
        // 定时持续发布最新状态（10Hz）
        // 平时发0；收到下位机1时持续发1；收到下位机2时发一次2，之后恢复发0
        status_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() {
                auto msg = std_msgs::msg::Int32();
                int32_t publish_value = 0;
                {
                    std::lock_guard<std::mutex> lock(status_mutex_);
                    if (climber_finish_once_) {
                        // 收到完成信号，发一次2，然后恢复
                        publish_value = 2;
                        climber_finish_once_ = false;
                        climber_running_ = false;
                    } else if (climber_running_) {
                        // 正在执行中，持续发1
                        publish_value = 1;
                    } else {
                        publish_value = 0;
                    }
                }
                msg.data = publish_value;
                climber_status_pub_->publish(msg);
            });
        
        // 启动发送线程（独立线程，8ms周期）
        send_thread_ = std::thread(&VirtualSerialPortNode::send_thread_func, this);
        
        // 启动USB事件处理线程
        usb_thread_ = std::thread(&VirtualSerialPortNode::usb_thread_func, this);
        
        RCLCPP_INFO(this->get_logger(), "虚拟串口节点已启动，订阅话题: %s, 发送周期: %dms", 
            cmd_vel_topic.c_str(), send_interval_ms_);
        RCLCPP_INFO(this->get_logger(), "已订阅爬楼梯话题: /AT_R2/climb_stair, /AT_R2/descend_stair");
        RCLCPP_INFO(this->get_logger(), "已订阅区模式话题: /AT_R2/zone_mode (连续发送), 抓取命令话题: /AT_R2/head_gripper_cmd (单次发送)");
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
    
    void mode_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        // mode 连续发送: 只更新当前值, 由发送线程每包带上
        std::lock_guard<std::mutex> lock(velocity_mutex_);
        current_mode_ = static_cast<uint8_t>(msg->data);
        RCLCPP_INFO(this->get_logger(), "收到区模式: %d", msg->data);
    }
    
    void grasp_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        // grasp_cmd 单次发送: 记录值并置一次性标志
        std::lock_guard<std::mutex> lock(velocity_mutex_);
        current_grasp_cmd_ = static_cast<uint8_t>(msg->data);
        grasp_send_once_ = true;
        RCLCPP_INFO(this->get_logger(), "收到抓取命令: %d (将发送一次)", msg->data);
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
                packet.vx = -current_velocity_.vx;
                packet.vy = -current_velocity_.vy;
                packet.omega = -current_velocity_.omega;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "发送速度: vx %.2f vy %.2f omega %.2f", 
                    current_velocity_.vx, current_velocity_.vy, current_velocity_.omega);
                // mode: 连续发送当前区模式值
                packet.mode = current_mode_;
                // grasp_cmd: 收到话题时发送一次实际值, 其余时间发送0
                if (grasp_send_once_) {
                    packet.grasp_cmd = current_grasp_cmd_;
                    grasp_send_once_ = false;
                    RCLCPP_INFO(this->get_logger(), "发送抓取命令: grasp_cmd=%d", packet.grasp_cmd);
                } else {
                    packet.grasp_cmd = 0;
                }
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
            }
            packet.tail = 0xBA;
            
            // 发送并检查结果
            if (!cdc_trans_->send_struct(packet)) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "发送速度指令失败");
            }
            
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
            
            uint8_t new_status = status.climber_status;
            
            // 只在状态变化时处理，避免重复打印和重复操作
            if (new_status == last_climber_raw_ ) {
                return;
            }
            last_climber_raw_ = new_status;
            
            if (new_status == 1) {
                // 开始执行
                std::lock_guard<std::mutex> lock(status_mutex_);
                climber_running_ = true;
                climber_finish_once_ = false;
                RCLCPP_INFO(this->get_logger(), "爬楼梯状态更新: 开始执行 (1)");
            } else if (new_status == 2) {
                // 执行完成
                {
                    std::lock_guard<std::mutex> lock(status_mutex_);
                    climber_finish_once_ = true;
                }
                // 清除爬楼梯指令
                {
                    std::lock_guard<std::mutex> lock(velocity_mutex_);
                    current_climb_action_ = ClimbAction::NONE;
                    current_climb_height_ = 0;
                }
                RCLCPP_INFO(this->get_logger(), "爬楼梯状态更新: 执行完成 (2)");
            }
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
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr grasp_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr climber_status_pub_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    
    std::thread send_thread_;
    std::thread usb_thread_;
    std::atomic<bool> running_;
    
    std::mutex velocity_mutex_;
    Velocity current_velocity_;
    ClimbAction current_climb_action_;
    uint8_t current_climb_height_;
    bool climb_send_once_;
    uint8_t current_mode_;        // 当前区模式 (连续发送)
    uint8_t current_grasp_cmd_;   // 当前抓取命令值
    bool grasp_send_once_;        // 抓取命令一次性发送标志
    
    std::mutex status_mutex_;
    bool climber_running_{false};       // 下位机正在执行
    bool climber_finish_once_{false};   // 下位机执行完成，待发一次2
    uint8_t last_climber_raw_{0xFF};    // 上次收到的原始状态，用于去重
    
    uint16_t usb_vid_;
    uint16_t usb_pid_;
    int send_interval_ms_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VirtualSerialPortNode>());
    rclcpp::shutdown();
    return 0;
}

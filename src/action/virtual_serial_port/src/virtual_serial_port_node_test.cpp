#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include "virtual_serial_port/cdc_trans.hpp"
#include <thread>
#include <mutex>
#include <atomic>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <memory>

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
    uint8_t action;        // 动作命令 (单次发送, 其余发0): mode=1时为抓取命令(1/2/3/4), mode=2时为爬楼梯动作(1=上,2=下)
    uint8_t climb_height;  // 爬楼梯高度 (mode=2时配合action, 0=无 1=200mm 2=400mm)
    uint8_t pump;          // 气泵使能 (连续发送当前值): 1=吸气, 0=放气
    uint8_t tail;          // 包尾 0xBA
};
#pragma pack(pop)

// 从下位机接收的状态数据包结构
#pragma pack(push, 1)
struct StatusPacket {
    uint8_t header;          // 包头 0xAB
    uint8_t climber_status;  // 爬楼梯状态 (1=开始执行, 2=执行完成)
    uint16_t distance;       // 距离
    uint8_t tail;            // 包尾 0xBA
};
#pragma pack(pop)

class VirtualSerialPortNode : public rclcpp::Node
{
public:
    VirtualSerialPortNode() : Node("virtual_serial_port_node"), running_(true), climb_send_once_(false)
    {
        // 声明参数
        this->declare_parameter<int>("usb_vid", 0x0483);
        this->declare_parameter<int>("usb_pid", 0x5740);
        this->declare_parameter<std::string>("cmd_vel_topic", "/AT_R2/cmd_vel_nav2_result");
        this->declare_parameter<std::string>("bt_cmd_vel_topic", "/AT_R2/cmd_vel_bt");
        this->declare_parameter<double>("nav_cmd_vel_timeout_sec", 0.5);
        this->declare_parameter<double>("bt_cmd_vel_timeout_sec", 0.2);
        this->declare_parameter<int>("send_interval_ms", 8);

        // 获取参数
        usb_vid_ = static_cast<uint16_t>(this->get_parameter("usb_vid").as_int());
        usb_pid_ = static_cast<uint16_t>(this->get_parameter("usb_pid").as_int());
        std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
        std::string bt_cmd_vel_topic = this->get_parameter("bt_cmd_vel_topic").as_string();
        nav_cmd_vel_timeout_sec_ = this->get_parameter("nav_cmd_vel_timeout_sec").as_double();
        bt_cmd_vel_timeout_sec_ = this->get_parameter("bt_cmd_vel_timeout_sec").as_double();
        send_interval_ms_ = this->get_parameter("send_interval_ms").as_int();

        // 初始化速度和爬楼梯状态
        nav_velocity_ = {0.0f, 0.0f, 0.0f};
        bt_velocity_ = {0.0f, 0.0f, 0.0f};
        last_nav_cmd_time_ = this->now();
        last_bt_cmd_time_ = this->now();
        nav_cmd_received_ = false;
        bt_cmd_received_ = false;
        current_climb_action_ = ClimbAction::NONE;
        current_climb_height_ = 0;
        current_mode_ = 0;
        current_grasp_cmd_ = 0;
        grasp_send_once_ = false;
        current_pump_ = 0;

        // 初始化CDC设备
        cdc_trans_ = std::make_unique<CDCTrans>();

        // 注册接收回调(必须在open之前, 否则open提交IN传输后回调为空, 数据会被静默丢弃)
        cdc_trans_->regeiser_recv_cb([this](const uint8_t* data, int size) {
            this->on_data_received(data, size);
        });

        // 尝试打开USB设备
        if (cdc_trans_->open(usb_vid_, usb_pid_)) {
            RCLCPP_INFO(this->get_logger(), "USB-CDC设备打开成功 VID:0x%04X PID:0x%04X", usb_vid_, usb_pid_);
        } else {
            RCLCPP_WARN(this->get_logger(), "USB-CDC设备打开失败，将持续尝试重连");
        }

        // 订阅速度话题: Nav2 为默认源, BT override 优先级更高
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_topic, 10,
            std::bind(&VirtualSerialPortNode::nav_cmd_vel_callback, this, std::placeholders::_1));
        bt_cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            bt_cmd_vel_topic, 10,
            std::bind(&VirtualSerialPortNode::bt_cmd_vel_callback, this, std::placeholders::_1));

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
        grasp_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/AT_R2/head_gripper_cmd", 10,
            std::bind(&VirtualSerialPortNode::grasp_callback, this, std::placeholders::_1));

        // 订阅底盘气泵使能话题 (latched, pump 连续发送当前值: 1=吸气, 0=放气)
        pump_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/AT_R2/chassis_pump_cmd", rclcpp::QoS(1).transient_local().reliable(),
            std::bind(&VirtualSerialPortNode::pump_callback, this, std::placeholders::_1));

        // 创建爬楼梯状态发布器
        climber_status_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/AT_R2/climber_status", 10);

        // 创建抓取状态发布器 (一区时由下位机 climber_status 字段复用)
        grasp_status_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/AT_R2/grasp_status", 10);

        // 创建距离发布器
        distance_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/AT_R2/distance", 10);

        // 定时持续发布最新状态（10Hz）
        status_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() {
                auto msg = std_msgs::msg::Int32();
                int32_t publish_value = 0;
                {
                    std::lock_guard<std::mutex> lock(status_mutex_);
                    if (climber_finish_once_) {
                        publish_value = 2;
                        climber_finish_once_ = false;
                        climber_running_ = false;
                    } else if (climber_running_) {
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

        RCLCPP_INFO(this->get_logger(),
            "虚拟串口节点已启动，Nav2速度话题: %s, BT速度话题: %s, 发送周期: %dms",
            cmd_vel_topic.c_str(), bt_cmd_vel_topic.c_str(), send_interval_ms_);
        RCLCPP_INFO(this->get_logger(), "已订阅爬楼梯话题: /AT_R2/climb_stair, /AT_R2/descend_stair");
        RCLCPP_INFO(this->get_logger(), "已订阅区模式话题: /AT_R2/zone_mode (连续发送), 抓取命令话题: /AT_R2/head_gripper_cmd (单次发送)");
        RCLCPP_INFO(this->get_logger(), "已订阅底盘气泵使能话题: /AT_R2/chassis_pump_cmd (连续发送, 1=吸气 0=放气)");
        RCLCPP_INFO(this->get_logger(), "已创建状态发布器: /AT_R2/climber_status, /AT_R2/grasp_status, 距离发布器: /AT_R2/distance");
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
    void nav_cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(velocity_mutex_);
        nav_velocity_.vx = static_cast<float>(msg->linear.x);
        nav_velocity_.vy = static_cast<float>(msg->linear.y);
        nav_velocity_.omega = static_cast<float>(msg->angular.z);
        last_nav_cmd_time_ = this->now();
        nav_cmd_received_ = true;
    }

    void bt_cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(velocity_mutex_);
        bt_velocity_.vx = static_cast<float>(msg->linear.x);
        bt_velocity_.vy = static_cast<float>(msg->linear.y);
        bt_velocity_.omega = static_cast<float>(msg->angular.z);
        last_bt_cmd_time_ = this->now();
        bt_cmd_received_ = true;
    }

    void climb_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(velocity_mutex_);
        if (current_mode_ != 2) {
            RCLCPP_WARN(this->get_logger(),
                "当前 mode=%d (非二区), 忽略爬楼梯指令 (高度 %.2f m)", current_mode_, msg->data);
            return;
        }
        current_climb_action_ = ClimbAction::CLIMB;
        double h = msg->data;
        current_climb_height_ = (h <= 0.3) ? 1 : 2;
        climb_send_once_ = true;
        RCLCPP_INFO(this->get_logger(), "收到爬楼梯指令: 高度 %.2f m -> 编码 %d", msg->data, current_climb_height_);
    }

    void descend_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(velocity_mutex_);
        if (current_mode_ != 2) {
            RCLCPP_WARN(this->get_logger(),
                "当前 mode=%d (非二区), 忽略下楼梯指令 (高度 %.2f m)", current_mode_, msg->data);
            return;
        }
        current_climb_action_ = ClimbAction::DESCEND;
        double h = msg->data;
        current_climb_height_ = (h <= 0.3) ? 1 : 2;
        climb_send_once_ = true;
        RCLCPP_INFO(this->get_logger(), "收到下楼梯指令: 高度 %.2f m -> 编码 %d", msg->data, current_climb_height_);
    }

    void mode_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        uint8_t new_mode = static_cast<uint8_t>(msg->data);
        uint8_t old_mode;
        {
            std::lock_guard<std::mutex> lock(velocity_mutex_);
            old_mode = current_mode_;
            current_mode_ = new_mode;
        }
        RCLCPP_INFO(this->get_logger(), "收到区模式: %d", msg->data);

        // mode 切换时清零对应的状态发布与去重缓存，避免跨区残留
        if (old_mode != new_mode) {
            std::lock_guard<std::mutex> lock(status_mutex_);
            last_climber_raw_ = 0xFF;
            if (old_mode == 1) {
                auto zero_msg = std_msgs::msg::Int32();
                zero_msg.data = 0;
                grasp_status_pub_->publish(zero_msg);
            }
            if (old_mode == 2) {
                climber_running_ = false;
                climber_finish_once_ = false;
            }
        }
    }

    void grasp_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(velocity_mutex_);
        if (current_mode_ != 1) {
            RCLCPP_WARN(this->get_logger(),
                "当前 mode=%d (非一区), 忽略抓取命令 %d", current_mode_, msg->data);
            return;
        }
        current_grasp_cmd_ = static_cast<uint8_t>(msg->data);
        grasp_send_once_ = true;
        RCLCPP_INFO(this->get_logger(), "收到抓取命令: %d (将发送一次)", msg->data);
    }

    void pump_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(velocity_mutex_);
        current_pump_ = (msg->data != 0) ? 1 : 0;
        RCLCPP_INFO(this->get_logger(), "收到气泵指令: %d (%s)",
            msg->data, current_pump_ ? "吸气" : "放气");
    }

    void send_thread_func()
    {
        using namespace std::chrono_literals;
        RCLCPP_INFO(this->get_logger(), "发送线程已启动");

        while (running_) {
            auto now = std::chrono::steady_clock::now();

            VelocityPacket packet;
            packet.header = 0xAB;
            {
                std::lock_guard<std::mutex> lock(velocity_mutex_);
                const rclcpp::Time ros_now = this->now();
                const bool bt_fresh = bt_cmd_received_ &&
                    (ros_now - last_bt_cmd_time_).seconds() <= bt_cmd_vel_timeout_sec_;
                const bool nav_fresh = nav_cmd_received_ &&
                    (ros_now - last_nav_cmd_time_).seconds() <= nav_cmd_vel_timeout_sec_;

                Velocity selected_velocity{0.0f, 0.0f, 0.0f};
                const char* selected_source = "zero";
                if (bt_fresh) {
                    selected_velocity = bt_velocity_;
                    selected_source = "bt";
                } else if (nav_fresh) {
                    selected_velocity = nav_velocity_;
                    selected_source = "nav";
                }

                packet.vx = -selected_velocity.vx;
                packet.vy = -selected_velocity.vy;
                packet.omega = -selected_velocity.omega;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "发送速度[%s]: vx %.2f vy %.2f omega %.2f mode %d",
                    selected_source, selected_velocity.vx, selected_velocity.vy, selected_velocity.omega,
                    current_mode_);
                packet.mode = current_mode_;
                packet.pump = current_pump_;
                if (grasp_send_once_) {
                    packet.action = current_grasp_cmd_;
                    packet.climb_height = 0;
                    grasp_send_once_ = false;
                    RCLCPP_INFO(this->get_logger(), "发送抓取命令: action=%d", packet.action);
                } else if (climb_send_once_) {
                    packet.action = static_cast<uint8_t>(current_climb_action_);
                    packet.climb_height = current_climb_height_;
                    climb_send_once_ = false;
                    RCLCPP_INFO(this->get_logger(), "发送爬楼梯指令: action=%d, height=%d",
                        packet.action, packet.climb_height);
                } else {
                    packet.action = 0;
                    packet.climb_height = 0;
                }
            }
            packet.tail = 0xBA;

            int send_ret = cdc_trans_->send(
                reinterpret_cast<const uint8_t*>(&packet), sizeof(packet), 100);
            if (send_ret <= 0) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "发送速度指令失败, ret=%d", send_ret);
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
        std::stringstream hex_ss;
        for (int i = 0; i < size && i < 64; i++) {
            hex_ss << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
                   << static_cast<int>(data[i]) << " ";
        }
        // RCLCPP_INFO(this->get_logger(), "收到下位机数据，长度: %d, 前64字节hex: [%s]",
        //     size, hex_ss.str().c_str());

        if (size == sizeof(StatusPacket)) {
            StatusPacket status;
            std::memcpy(&status, data, sizeof(StatusPacket));

            if (status.header != 0xAB || status.tail != 0xBA) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "收到的数据包头尾校验失败: header=0x%02X tail=0x%02X",
                    status.header, status.tail);
                return;
            }

            // 发布距离
            auto dist_msg = std_msgs::msg::Float64();
            dist_msg.data = static_cast<double>(status.distance);
            distance_pub_->publish(dist_msg);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "收到距离: %d (0x%04X)", status.distance, status.distance);

            uint8_t new_status = status.climber_status;

            if (new_status == last_climber_raw_) {
                return;
            }
            last_climber_raw_ = new_status;

            uint8_t mode_snapshot;
            {
                std::lock_guard<std::mutex> lock(velocity_mutex_);
                mode_snapshot = current_mode_;
            }

            if (mode_snapshot == 1) {
                if (new_status == 1 || new_status == 2) {
                    auto grasp_msg = std_msgs::msg::Int32();
                    grasp_msg.data = static_cast<int32_t>(new_status);
                    grasp_status_pub_->publish(grasp_msg);
                    RCLCPP_INFO(this->get_logger(),
                        "抓取状态更新: %s (%d)",
                        new_status == 1 ? "开始执行" : "执行完成", new_status);
                }
            } else if (mode_snapshot == 2) {
                if (new_status == 1) {
                    std::lock_guard<std::mutex> lock(status_mutex_);
                    climber_running_ = true;
                    climber_finish_once_ = false;
                    RCLCPP_INFO(this->get_logger(), "爬楼梯状态更新: 开始执行 (1)");
                } else if (new_status == 2) {
                    {
                        std::lock_guard<std::mutex> lock(status_mutex_);
                        climber_finish_once_ = true;
                    }
                    {
                        std::lock_guard<std::mutex> lock(velocity_mutex_);
                        current_climb_action_ = ClimbAction::NONE;
                        current_climb_height_ = 0;
                    }
                    RCLCPP_INFO(this->get_logger(), "爬楼梯状态更新: 执行完成 (2)");
                }
            }
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "收到的数据包大小不匹配: 期望 %zu 字节，实际 %d 字节",
                sizeof(StatusPacket), size);
        }
    }

    struct Velocity {
        float vx;
        float vy;
        float omega;
    };

    std::unique_ptr<CDCTrans> cdc_trans_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr bt_cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr climb_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr descend_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr grasp_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr pump_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr climber_status_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr grasp_status_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_pub_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    std::thread send_thread_;
    std::thread usb_thread_;
    std::atomic<bool> running_;

    std::mutex velocity_mutex_;
    Velocity nav_velocity_;
    Velocity bt_velocity_;
    rclcpp::Time last_nav_cmd_time_;
    rclcpp::Time last_bt_cmd_time_;
    bool nav_cmd_received_;
    bool bt_cmd_received_;
    double nav_cmd_vel_timeout_sec_;
    double bt_cmd_vel_timeout_sec_;
    ClimbAction current_climb_action_;
    uint8_t current_climb_height_;
    bool climb_send_once_;
    uint8_t current_mode_;
    uint8_t current_grasp_cmd_;
    bool grasp_send_once_;
    uint8_t current_pump_;

    std::mutex status_mutex_;
    bool climber_running_{false};
    bool climber_finish_once_{false};
    uint8_t last_climber_raw_{0xFF};

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
/**
 * @file keyboard_control_node.cpp
 * @brief AT_R2 机器人键盘控制节点 (C++)
 *
 * 控制 AT_R2 机器人全向移动、升降臂和爬台阶
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <chrono>
#include <iostream>
#include <string>

class AT_R2_KeyboardControl : public rclcpp::Node
{
public:
    AT_R2_KeyboardControl() : Node("at_r2_keyboard_control")
    {
        // 声明参数：机器人名称
        this->declare_parameter<std::string>("robot_name", "AT_R2");
        robot_name_ = this->get_parameter("robot_name").as_string();

        if (robot_name_.empty()) {
            robot_name_ = "AT_R2";
        }

        // 构建话题名称
        std::string cmd_vel_topic = "/" + robot_name_ + "/cmd_vel_nav2_result";
        std::string climb_stair_topic = "/" + robot_name_ + "/climb_stair";
        std::string descend_stair_topic = "/" + robot_name_ + "/descend_stair";
        std::string head_gripper_topic = "/" + robot_name_ + "/head_gripper_cmd";
        std::string zone_mode_topic = "/" + robot_name_ + "/zone_mode";
        std::string pump_topic = "/" + robot_name_ + "/pump_cmd";

        // 创建发布者
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
        climb_stair_pub_ = this->create_publisher<std_msgs::msg::Float64>(climb_stair_topic, 10);
        descend_stair_pub_ = this->create_publisher<std_msgs::msg::Float64>(descend_stair_topic, 10);
        // 武器头爪子指令: latched(transient_local), 确保爪子节点稍晚连上也能收到最新指令
        head_gripper_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            head_gripper_topic, rclcpp::QoS(1).transient_local().reliable());
        // 区模式: latched(transient_local), 后启动的节点也能立刻拿到当前模式
        zone_mode_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            zone_mode_topic, rclcpp::QoS(1).transient_local().reliable());
        // 气泵使能: latched(transient_local), 1=吸气 0=放气, 连续保持当前值
        pump_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            pump_topic, rclcpp::QoS(1).transient_local().reliable());

        // 速度参数
        linear_speed_ = 0.1;   // m/s
        angular_speed_ = 0.1;  // rad/s

        printHelp();
    }

    void printHelp()
    {
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "    AT_R2 机器人键盘控制");
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "【全向移动】");
        RCLCPP_INFO(this->get_logger(), "        W          前进");
        RCLCPP_INFO(this->get_logger(), "   A    S    D     左移/后退/右移");
        RCLCPP_INFO(this->get_logger(), "   Q         E     左转/右转");
        RCLCPP_INFO(this->get_logger(), "      空格         停止移动");
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "【爬楼梯/下楼梯】");
        RCLCPP_INFO(this->get_logger(), "   Z - 上楼梯 200mm");
        RCLCPP_INFO(this->get_logger(), "   X - 上楼梯 400mm");
        RCLCPP_INFO(this->get_logger(), "   C - 下楼梯 200mm");
        RCLCPP_INFO(this->get_logger(), "   V - 下楼梯 400mm");
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "【武器头爪子 /head_gripper_cmd】");
        RCLCPP_INFO(this->get_logger(), "   1 - 抬升");
        RCLCPP_INFO(this->get_logger(), "   2 - 准备抓取");
        RCLCPP_INFO(this->get_logger(), "   3 - 抓取");
        RCLCPP_INFO(this->get_logger(), "   4 - 抬起武器头");
        RCLCPP_INFO(this->get_logger(), "   5 - 对接准备动作");
        RCLCPP_INFO(this->get_logger(), "   6 - 下降");
        RCLCPP_INFO(this->get_logger(), "   T - 松开");
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "【区模式 /zone_mode】");
        RCLCPP_INFO(this->get_logger(), "   7 - 一区 (抓武器头/对接)");
        RCLCPP_INFO(this->get_logger(), "   8 - 二区 (上下台阶/抓块)");
        RCLCPP_INFO(this->get_logger(), "   9 - 三区 (预留)");
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "【气泵 /pump_cmd】");
        RCLCPP_INFO(this->get_logger(), "   G - 吸气");
        RCLCPP_INFO(this->get_logger(), "   H - 放气");
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "【参数调节】");
        RCLCPP_INFO(this->get_logger(), "   [/] - 移动速度 -/+10%%");
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "   P - 显示帮助    Ctrl+C - 退出");
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "速度: %.2f m/s", linear_speed_);
    }

    void run()
    {
        struct termios oldt;
        tcgetattr(STDIN_FILENO, &oldt);

        try {
            while (rclcpp::ok()) {
                char key = getKey();
                if (key == 0) {
                    // 超时：检查距上次按键是否超过保持时间窗口
                    auto now = std::chrono::steady_clock::now();
                    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                        now - last_key_time_).count();
                    if (elapsed > key_hold_timeout_ms_) {
                        vel_x_ = 0.0; vel_y_ = 0.0; vel_z_ = 0.0;
                    }
                } else {
                    last_key_time_ = std::chrono::steady_clock::now();
                    processKey(key);
                }
                publishAll();
                rclcpp::spin_some(this->shared_from_this());
            }
        } catch (...) {
            tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
            stop();
            throw;
        }

        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        stop();
    }

private:
    char getKey()
    {
        struct termios oldt, newt;
        char ch = 0;

        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        newt.c_cc[VMIN] = 0;
        newt.c_cc[VTIME] = 0;  // 非阻塞
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        // 用 select 实现 20ms 超时
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 20000;  // 20ms

        if (select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv) > 0) {
            read(STDIN_FILENO, &ch, 1);
        }

        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        return ch;
    }

    void processKey(char key)
    {
        switch (key) {
            // ========== 全向移动 ==========
            case 'w': case 'W':
                vel_x_ = linear_speed_; vel_y_ = 0.0; vel_z_ = 0.0;
                break;
            case 's': case 'S':
                vel_x_ = -linear_speed_; vel_y_ = 0.0; vel_z_ = 0.0;
                break;
            case 'a': case 'A':
                vel_x_ = 0.0; vel_y_ = linear_speed_; vel_z_ = 0.0;  // 左移
                break;
            case 'd': case 'D':
                vel_x_ = 0.0; vel_y_ = -linear_speed_; vel_z_ = 0.0;  // 右移
                break;
            case 'q': case 'Q':
                vel_x_ = 0.0; vel_y_ = 0.0; vel_z_ = angular_speed_;  // 左转
                break;
            case 'e': case 'E':
                vel_x_ = 0.0; vel_y_ = 0.0; vel_z_ = -angular_speed_;  // 右转
                break;
            case ' ':
                vel_x_ = 0.0; vel_y_ = 0.0; vel_z_ = 0.0;
                RCLCPP_INFO(this->get_logger(), "移动停止");
                break;

            // ========== 爬楼梯/下楼梯 ==========
            case 'z': case 'Z':
                climbStair(0.2);
                break;
            case 'x': case 'X':
                climbStair(0.4);
                break;
            case 'c': case 'C':
                descendStair(0.2);
                break;
            case 'v': case 'V':
                descendStair(0.4);
                break;

            // ========== 武器头爪子指令 ==========
            case '1':
                publishHeadGripper(1);  // 准备抓取
                break;
            case '2':
                publishHeadGripper(2);  // 抓取
                break;
            case '3':
                publishHeadGripper(3);  // 抬起武器头
                break;
            case '4':
                publishHeadGripper(4);  // 对接准备
                break;
            case '5':
                publishHeadGripper(5);  // 下降
                break;
            case '6':
                publishHeadGripper(6);  // 松开
                break;
            case 't': case 'T':
                publishHeadGripper(7);  // 抓取武器头
                break;
            // ========== 区模式切换 ==========
            case '7':
                publishZoneMode(1);  // 一区
                break;
            case '8':
                publishZoneMode(2);  // 二区
                break;
            case '9':
                publishZoneMode(3);  // 三区
                break;

            // ========== 气泵 ==========
            case 'g': case 'G':
                publishPump(1);  // 吸气
                break;
            case 'h': case 'H':
                publishPump(0);  // 放气
                break;

            // ========== 参数调节 ==========
            case '[':
                linear_speed_ *= 0.9;
                angular_speed_ *= 0.9;
                RCLCPP_INFO(this->get_logger(), "速度: %.2f m/s, %.2f rad/s", linear_speed_, angular_speed_);
                break;
            case ']':
                linear_speed_ *= 1.1;
                angular_speed_ *= 1.1;
                RCLCPP_INFO(this->get_logger(), "速度: %.2f m/s, %.2f rad/s", linear_speed_, angular_speed_);
                break;

            // ========== 帮助 ==========
            case 'p': case 'P':
                printHelp();
                break;

            // ========== 退出 ==========
            case 3:  // Ctrl+C
                RCLCPP_INFO(this->get_logger(), "退出");
                rclcpp::shutdown();
                break;
        }
    }

    void climbStair(double height)
    {
        RCLCPP_INFO(this->get_logger(), "开始上楼梯，高度: %.2f m", height);
        vel_x_ = 0.0; vel_y_ = 0.0; vel_z_ = 0.0;
        auto msg = std_msgs::msg::Float64();
        msg.data = height;
        climb_stair_pub_->publish(msg);
    }

    void descendStair(double height)
    {
        RCLCPP_INFO(this->get_logger(), "开始下楼梯，高度: %.2f m", height);
        vel_x_ = 0.0; vel_y_ = 0.0; vel_z_ = 0.0;
        auto msg = std_msgs::msg::Float64();
        msg.data = height;
        descend_stair_pub_->publish(msg);
    }

    void publishHeadGripper(int32_t command)
    {
        const char* desc = command == 1 ? "准备抓取"
                         : command == 2 ? "抓取"
                         : command == 3 ? "抬起武器头"
                         : command == 4 ? "对接准备动作"
                         : command == 5 ? "下降"
                         : command == 6 ? "松开"
                         : command == 7 ? "抓取武器头":"未知";
        RCLCPP_INFO(this->get_logger(), "武器头爪子指令: %d (%s)", command, desc);
        auto msg = std_msgs::msg::Int32();
        msg.data = command;
        head_gripper_pub_->publish(msg);
    }

    void publishZoneMode(int32_t mode)
    {
        const char* desc = mode == 1 ? "一区: 抓武器头/对接"
                         : mode == 2 ? "二区: 上下台阶/抓块"
                         : mode == 3 ? "三区: 预留" : "未知";
        RCLCPP_INFO(this->get_logger(), "切换区模式: %d (%s)", mode, desc);
        auto msg = std_msgs::msg::Int32();
        msg.data = mode;
        zone_mode_pub_->publish(msg);
    }

    void publishPump(int32_t enable)
    {
        RCLCPP_INFO(this->get_logger(), "气泵: %d (%s)", enable, enable ? "吸气" : "放气");
        auto msg = std_msgs::msg::Int32();
        msg.data = enable;
        pump_pub_->publish(msg);
    }

    void publishAll()
    {
        // 发布速度
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = vel_x_;
        twist_msg.linear.y = vel_y_;
        twist_msg.angular.z = vel_z_;
        cmd_vel_pub_->publish(twist_msg);
    }

    void stop()
    {
        vel_x_ = 0.0; vel_y_ = 0.0; vel_z_ = 0.0;
        publishAll();
        RCLCPP_INFO(this->get_logger(), "机器人已停止");
    }

    // 发布者
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr climb_stair_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr descend_stair_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr head_gripper_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr zone_mode_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pump_pub_;

    // 参数
    std::string robot_name_;
    double linear_speed_;
    double angular_speed_;
    
    // 速度状态
    double vel_x_ = 0.0;
    double vel_y_ = 0.0;
    double vel_z_ = 0.0;
    
    // 按键保持时间窗口
    std::chrono::steady_clock::time_point last_key_time_ = std::chrono::steady_clock::now();
    const int key_hold_timeout_ms_ = 500;  // 600ms，覆盖键盘重复延迟
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AT_R2_KeyboardControl>();

    try {
        node->run();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("at_r2_keyboard_control"), "异常: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}

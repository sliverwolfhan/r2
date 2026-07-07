#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include "at_r2_serial_bridge/cdc_trans.hpp"
#include <thread>
#include <mutex>
#include <atomic>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <vector>

// 从下位机接收的状态数据包结构
#pragma pack(push, 1)
struct StatusPacket {
    uint8_t header;          // 包头 0xAB
    uint8_t climber_status;  // 爬楼梯状态 (1=开始执行, 2=执行完成)
    uint8_t tail;            // 包尾 0xBA
};
#pragma pack(pop)

// 从下位机接收的 KFS 数据包结构 (遥控器发送的真假秘籍)
#pragma pack(push, 1)
struct KfsPacket {
    uint8_t header;        // 包头 0x5A
    uint8_t length;        // 包长度 20 (即 0x14)
    uint8_t cmd;           // 命令字 0x05
    uint32_t packet_id;    // 包ID
    uint8_t kfs_data[12];  // 12 个方块的数据 (0:空, 1:R1, 2:R2, 3:Fake)
    uint8_t sum;           // 校验和
};
#pragma pack(pop)

// 从下位机接收的对接状态数据包结构
#pragma pack(push, 1)
struct DockingPacket {
    uint8_t header;        // 包头 0x5A
    uint8_t length;        // 包长度 11 (即 0x0B)
    uint8_t cmd;           // 命令字 0x06
    uint32_t packet_id;    // 包ID
    uint8_t data_header;   // 数据域头 0xAA
    uint8_t dock_status;   // 对接状态 (1表示对接完成)
    uint8_t data_tail;     // 数据域尾 0xBB
    uint8_t sum;           // 校验和
};
#pragma pack(pop)

class VirtualSerialPortNode : public rclcpp::Node
{
public:
    VirtualSerialPortNode() : Node("at_r2_serial_bridge_node"), running_(true)
    {
        // 声明参数
        this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
        this->declare_parameter<int>("baudrate", 115200);

        // 获取参数
        port_name_ = this->get_parameter("port_name").as_string();
        baudrate_ = this->get_parameter("baudrate").as_int();

        // 初始化CDC设备
        cdc_trans_ = std::make_unique<CDCTrans>();

        // 尝试打开USB设备
        if (cdc_trans_->open(port_name_, baudrate_)) {
            RCLCPP_INFO(this->get_logger(), "串口设备打开成功: %s, 波特率: %d", port_name_.c_str(), baudrate_);
        } else {
            RCLCPP_WARN(this->get_logger(), "串口设备打开失败，将持续尝试重连");
        }

        // 注册接收回调
        cdc_trans_->regeiser_recv_cb([this](const uint8_t* data, int size) {
            this->on_data_received(data, size);
        });

        // 创建爬楼梯状态发布器
        climber_status_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/AT_R2/climber_status", 10);

        // 创建 KFS 数据发布器
        kfs_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
            "/AT_R2/kfs_positions", 10);

        // 创建对接状态发布器
        docking_status_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/AT_R2/meilin_mission_start", 10);

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

        // 只启动USB事件处理线程，不再向串口发送任何数据
        usb_thread_ = std::thread(&VirtualSerialPortNode::usb_thread_func, this);

        RCLCPP_INFO(this->get_logger(), "虚拟串口接收节点已启动");
        RCLCPP_INFO(this->get_logger(), "已创建状态发布器: /AT_R2/climber_status");
    }

    ~VirtualSerialPortNode()
    {
        running_ = false;
        if (usb_thread_.joinable()) {
            usb_thread_.join();
        }
    }

private:
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
        // 无论格式是否正确，先打印本次收到的原始数据
        std::ostringstream raw_msg;
        raw_msg << "收到原始串口数据(" << size << " bytes):";
        raw_msg << std::uppercase << std::hex << std::setfill('0');
        for (int i = 0; i < size; ++i) {
            raw_msg << " 0x" << std::setw(2) << static_cast<int>(data[i]);
        }
        // RCLCPP_INFO(this->get_logger(), "%s", raw_msg.str().c_str());

        // 将新收到的碎片数据追加到缓冲区
        rx_buffer_.insert(rx_buffer_.end(), data, data + size);

        // 不断从缓冲区里解析完整的包
        while (!rx_buffer_.empty()) {
            if (rx_buffer_[0] == 0x5A) {
                // 如果缓冲里已经有至少 3 个字节，就可以读出命令字和预期长度
                if (rx_buffer_.size() >= 3) {
                    uint8_t pkt_len = rx_buffer_[1];
                    uint8_t pkt_cmd = rx_buffer_[2];

                    if (pkt_cmd == 0x05 && pkt_len == sizeof(KfsPacket)) {
                        // KFS 数据包
                        if (rx_buffer_.size() >= sizeof(KfsPacket)) {
                            KfsPacket kfs_packet;
                            std::memcpy(&kfs_packet, rx_buffer_.data(), sizeof(KfsPacket));

                            // 校验和计算
                            uint8_t sum = 0;
                            for (size_t i = 0; i < sizeof(KfsPacket) - 1; i++) {
                                sum += rx_buffer_[i];
                            }
                            if (sum == kfs_packet.sum) {
                                auto msg = std_msgs::msg::Int32MultiArray();
                                for (int i = 0; i < 12; i++) {
                                    msg.data.push_back(kfs_packet.kfs_data[i]);
                                }
                                kfs_pub_->publish(msg);
                                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                    "成功接收并发布 KFS 状态 (Packet ID: %u)", kfs_packet.packet_id);
                            } else {
                                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                    "KFS数据包校验失败: 计算得 0x%02X, 收到 0x%02X", sum, kfs_packet.sum);
                            }
                            rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + sizeof(KfsPacket));
                        } else {
                            break; // 长度不够，等待新数据
                        }
                    }
                    else if (pkt_cmd == 0x06 && pkt_len == sizeof(DockingPacket)) {
                        // 对接完成状态包
                        if (rx_buffer_.size() >= sizeof(DockingPacket)) {
                            DockingPacket dock_packet;
                            std::memcpy(&dock_packet, rx_buffer_.data(), sizeof(DockingPacket));

                            uint8_t sum = 0;
                            for (size_t i = 0; i < sizeof(DockingPacket) - 1; i++) {
                                sum += rx_buffer_[i];
                            }
                            if (sum == dock_packet.sum &&
                                dock_packet.data_header == 0xAA &&
                                dock_packet.data_tail == 0xBB) {
                                auto msg = std_msgs::msg::Int32();
                                msg.data = dock_packet.dock_status;
                                docking_status_pub_->publish(msg);
                                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                    "成功接收并发布 对接状态: %d (Packet ID: %u)", msg.data, dock_packet.packet_id);
                            } else {
                                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                    "对接包校验失败: 计算得 0x%02X, 收到 0x%02X, 数据域头尾 0x%02X 0x%02X",
                                    sum, dock_packet.sum, dock_packet.data_header, dock_packet.data_tail);
                            }
                            rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + sizeof(DockingPacket));
                        } else {
                            break; // 长度不够，等待新数据
                        }
                    }
                    else {
                        // 虽然是 0x5A 开头，但指令不对或者长度不匹配，丢弃第一个字节继续找
                        rx_buffer_.erase(rx_buffer_.begin());
                    }
                } else {
                    // 数据不够 3 字节，无法判断命令字，等待
                    break;
                }
            }
            else if (rx_buffer_[0] == 0xAB) {
                // 可能是底盘状态包，长度需要 3 字节
                if (rx_buffer_.size() >= sizeof(StatusPacket)) {
                    StatusPacket status;
                    std::memcpy(&status, rx_buffer_.data(), sizeof(StatusPacket));

                    if (status.tail == 0xBA) {
                        uint8_t new_status = status.climber_status;
                        if (new_status != last_climber_raw_) {
                            last_climber_raw_ = new_status;
                            if (new_status == 1) {
                                std::lock_guard<std::mutex> lock(status_mutex_);
                                climber_running_ = true;
                                climber_finish_once_ = false;
                                RCLCPP_INFO(this->get_logger(), "爬楼梯状态更新: 开始执行 (1)");
                            } else if (new_status == 2) {
                                std::lock_guard<std::mutex> lock(status_mutex_);
                                climber_finish_once_ = true;
                                RCLCPP_INFO(this->get_logger(), "爬楼梯状态更新: 执行完成 (2)");
                            }
                        }
                        rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + sizeof(StatusPacket));
                    } else {
                        rx_buffer_.erase(rx_buffer_.begin());
                    }
                } else {
                    break; // 数据不够 3 字节
                }
            }
            else {
                // 既不是 0x5A 也不是 0xAB 的垃圾字节，直接丢弃
                rx_buffer_.erase(rx_buffer_.begin());
            }
        }
    }

    std::unique_ptr<CDCTrans> cdc_trans_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr climber_status_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr kfs_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr docking_status_pub_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    std::thread usb_thread_;
    std::atomic<bool> running_;

    std::mutex status_mutex_;
    bool climber_running_{false};       // 下位机正在执行
    bool climber_finish_once_{false};   // 下位机执行完成，待发一次2
    uint8_t last_climber_raw_{0xFF};    // 上次收到的原始状态，用于去重

    std::vector<uint8_t> rx_buffer_;
    std::string port_name_;
    int baudrate_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VirtualSerialPortNode>());
    rclcpp::shutdown();
    return 0;
}

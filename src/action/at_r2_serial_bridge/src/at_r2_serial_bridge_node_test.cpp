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
    uint8_t kfs_data[12];  // 12 个方块的数据 (0:空, 1:R1, 2:R2, 3:Fake, 4:R1未取)
    uint8_t sum;           // 校验和
};
#pragma pack(pop)

// 从下位机接收的 R2 任务数据包结构
#pragma pack(push, 1)
struct R2MissionPacket {
    uint8_t header;        // 包头 0x5A
    uint8_t length;        // 包长度 12 (即 0x0C)
    uint8_t cmd;           // 命令字 0x06
    uint32_t packet_id;    // 包ID
    uint8_t data_header;   // 数据域头 0xAA
    uint8_t dock_status;   // 对接状态 (1表示对接完成)
    uint8_t zone3_cmd;     // 3区cmd
    uint8_t data_tail;     // 数据域尾 0xBB
    uint8_t sum;           // 校验和
};
#pragma pack(pop)

// 从下位机接收的 放格子选择 数据包结构 (packet_id 之后直接是 9 字节数据域, 无 0xAA/0xBB 头尾)
//   data[0..2]: 前3位 (任一为 1 -> 发 5)
//   data[3..5]: 放格子选择位 (哪个为 1 -> 发 slot 1/2/3, 优先级 2>1>3)
//   data[6..8]: 大胜位 (任一为 1 -> 发 4, 优先级高于放格子)
// 整包 = 1+1+1+4+9+1 = 17 字节 (length 字段应为 0x11)
#pragma pack(push, 1)
struct R2SlotSelectPacket {
    uint8_t header;        // 包头 0x5A
    uint8_t length;        // 包长度 17 (即 0x11)
    uint8_t cmd;           // 命令字 0x07
    uint32_t packet_id;    // 包ID
    uint8_t data[9];       // 数据域: 9 字节 (紧跟 packet_id)
    uint8_t sum;           // 校验和
};
#pragma pack(pop)

// 从下位机接收的 遥控器控制 数据包结构 (cmd 0x01)
//   载荷 PackControl_t = 4 个摇杆 float + 1 个 uint32 Key。
//   按 Key 值映射后发布到 /AT_R2/place_retry (放块顶墙中断重试):
//     0x214 -> 1, 0x20C -> 2, 0x224 -> 3, 0x244 -> 4, 其它 -> 0。
//   整包 = 1+1+1+4+20+1 = 28 字节 (length 字段应为 0x1C)。
#define PACK_CONTROL_CMD    0x01
#pragma pack(push, 1)
struct PackControl_t {
    float rocker[4];       // 4 个摇杆值
    uint32_t Key;          // 按键值
};
struct R2ControlPacket {
    uint8_t header;        // 包头 0x5A
    uint8_t length;        // 包长度 28 (即 0x1C)
    uint8_t cmd;           // 命令字 0x01
    uint32_t packet_id;    // 包ID
    PackControl_t control; // 控制载荷 (rocker[4] + Key)
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

        // 创建 3区cmd 发布器
        zone3_cmd_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/AT_R2/zone3_cmd", 10);

        // 创建 放格子选择 发布器 (1/2/3=格子, 4=大胜)
        place_slot_select_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/AT_R2/place_slot_select", 10);

        // 创建 放块顶墙中断重试 发布器 (Key 映射 1/2/3/4, 其它 0)
        place_retry_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/AT_R2/place_retry", 10);

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

                // 持续重发最新 KFS 数据: 收到过一帧后就一直发, UI 上不再闪没
                {
                    std::lock_guard<std::mutex> lock(kfs_mutex_);
                    if (kfs_valid_) {
                        auto kfs_msg = std_msgs::msg::Int32MultiArray();
                        kfs_msg.data.assign(latest_kfs_.begin(), latest_kfs_.end());
                        kfs_pub_->publish(kfs_msg);
                    }
                }
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
                                // 校验通过: 缓存最新 KFS 数据, 由定时器持续重发
                                // (下位机是间歇发包, 若只在此处发一次, UI 上会闪一下就消失)
                                {
                                    std::lock_guard<std::mutex> lock(kfs_mutex_);
                                    latest_kfs_.assign(kfs_packet.kfs_data, kfs_packet.kfs_data + 12);
                                    kfs_valid_ = true;
                                }
                                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                    "成功接收 KFS 状态 (Packet ID: %u), 持续重发中", kfs_packet.packet_id);
                            } else {
                                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                    "KFS数据包校验失败: 计算得 0x%02X, 收到 0x%02X", sum, kfs_packet.sum);
                            }
                            rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + sizeof(KfsPacket));
                        } else {
                            break; // 长度不够，等待新数据
                        }
                    }
                    else if (pkt_cmd == 0x06 && pkt_len == sizeof(R2MissionPacket)) {
                        // R2 任务状态包
                        if (rx_buffer_.size() >= sizeof(R2MissionPacket)) {
                            R2MissionPacket mission_packet;
                            std::memcpy(&mission_packet, rx_buffer_.data(), sizeof(R2MissionPacket));

                            uint8_t sum = 0;
                            for (size_t i = 0; i < sizeof(R2MissionPacket) - 1; i++) {
                                sum += rx_buffer_[i];
                            }
                            if (sum == mission_packet.sum &&
                                mission_packet.data_header == 0xAA &&
                                mission_packet.data_tail == 0xBB) {
                                auto dock_msg = std_msgs::msg::Int32();
                                dock_msg.data = mission_packet.dock_status;
                                docking_status_pub_->publish(dock_msg);

                                auto zone3_msg = std_msgs::msg::Int32();
                                zone3_msg.data = mission_packet.zone3_cmd;
                                zone3_cmd_pub_->publish(zone3_msg);

                                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                    "成功接收并发布 对接状态: %d, 3区cmd: %d (Packet ID: %u)",
                                    dock_msg.data, zone3_msg.data, mission_packet.packet_id);
                            } else {
                                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                    "R2任务包校验失败: 计算得 0x%02X, 收到 0x%02X, 数据域头尾 0x%02X 0x%02X",
                                    sum, mission_packet.sum, mission_packet.data_header, mission_packet.data_tail);
                            }
                            rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + sizeof(R2MissionPacket));
                        } else {
                            break; // 长度不够，等待新数据
                        }
                    }
                    else if (pkt_cmd == 0x07 && pkt_len == sizeof(R2SlotSelectPacket)) {
                        // 放格子选择包 (data[9])
                        if (rx_buffer_.size() >= sizeof(R2SlotSelectPacket)) {
                            R2SlotSelectPacket slot_packet;
                            std::memcpy(&slot_packet, rx_buffer_.data(), sizeof(R2SlotSelectPacket));

                            uint8_t sum = 0;
                            for (size_t i = 0; i < sizeof(R2SlotSelectPacket) - 1; i++) {
                                sum += rx_buffer_[i];
                            }
                            if (sum == slot_packet.sum) {
                                // data[0..2]=前3位(任一=1 -> 5); data[3..5]=放格子位; data[6..8]=大胜位
                                // 前3位与后6位不会同时出现, 优先级无所谓; 这里前3位命中即发 5。
                                // 否则: 先判大胜(7-9位任一=1 -> 4); 再放格子(4-6位, 2>1>3)
                                int32_t sel = 0;
                                if (slot_packet.data[0] == 1 ||
                                    slot_packet.data[1] == 1 ||
                                    slot_packet.data[2] == 1) {
                                    sel = 5;                              // 前3位命中
                                } else if (slot_packet.data[6] == 1 ||
                                    slot_packet.data[7] == 1 ||
                                    slot_packet.data[8] == 1) {
                                    sel = 4;                              // 大胜
                                } else if (slot_packet.data[4] == 1) {
                                    sel = 2;                              // 2 优先
                                } else if (slot_packet.data[3] == 1) {
                                    sel = 1;
                                } else if (slot_packet.data[5] == 1) {
                                    sel = 3;
                                }

                                // 每包都打印 data[9] + 算出的 sel, 方便现场核对下位机发的实际数据
                                RCLCPP_INFO(this->get_logger(),
                                    "放格子包 data[9]=[%d %d %d | %d %d %d | %d %d %d] -> sel=%d (Packet ID: %u)",
                                    slot_packet.data[0], slot_packet.data[1], slot_packet.data[2],
                                    slot_packet.data[3], slot_packet.data[4], slot_packet.data[5],
                                    slot_packet.data[6], slot_packet.data[7], slot_packet.data[8],
                                    sel, slot_packet.packet_id);

                                if (sel != 0) {
                                    auto sel_msg = std_msgs::msg::Int32();
                                    sel_msg.data = sel;
                                    place_slot_select_pub_->publish(sel_msg);
                                    RCLCPP_INFO(this->get_logger(),
                                        "  -> 发布 place_slot_select: %d", sel);
                                } else {
                                    RCLCPP_INFO(this->get_logger(),
                                        "  -> 9 位全 0/未命中, 不发布 (等于未选)");
                                }
                            } else {
                                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                    "放格子选择包校验失败: 计算得 0x%02X, 收到 0x%02X",
                                    sum, slot_packet.sum);
                            }
                            rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + sizeof(R2SlotSelectPacket));
                        } else {
                            break; // 长度不够，等待新数据
                        }
                    }
                    else if (pkt_cmd == PACK_CONTROL_CMD && pkt_len == sizeof(R2ControlPacket)) {
                        // 遥控器控制包 (rocker[4] + Key), 按 Key 映射发 place_retry
                        if (rx_buffer_.size() >= sizeof(R2ControlPacket)) {
                            R2ControlPacket ctrl_packet;
                            std::memcpy(&ctrl_packet, rx_buffer_.data(), sizeof(R2ControlPacket));

                            uint8_t sum = 0;
                            for (size_t i = 0; i < sizeof(R2ControlPacket) - 1; i++) {
                                sum += rx_buffer_[i];
                            }
                            if (sum == ctrl_packet.sum) {
                                const uint32_t key = ctrl_packet.control.Key;
                                // Key 映射: 0x214->1, 0x20C->2, 0x224->3, 0x244->4, 其它->0
                                int32_t retry = 0;
                                switch (key) {
                                    case 0x214: retry = 1; break;
                                    case 0x20C: retry = 2; break;
                                    case 0x224: retry = 3; break;
                                    case 0x244: retry = 4; break;
                                    default:    retry = 0; break;
                                }

                                // 只在 Key 变化时发布一次 (去重, 避免同一 Key 连续刷)
                                if (key != last_control_key_) {
                                    last_control_key_ = key;
                                    auto retry_msg = std_msgs::msg::Int32();
                                    retry_msg.data = retry;
                                    place_retry_pub_->publish(retry_msg);
                                    RCLCPP_INFO(this->get_logger(),
                                        "控制包 Key=0x%X -> 发布 place_retry: %d (Packet ID: %u)",
                                        key, retry, ctrl_packet.packet_id);
                                }
                            } else {
                                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                    "控制包校验失败: 计算得 0x%02X, 收到 0x%02X", sum, ctrl_packet.sum);
                            }
                            rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + sizeof(R2ControlPacket));
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
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr zone3_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr place_slot_select_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr place_retry_pub_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    std::thread usb_thread_;
    std::atomic<bool> running_;

    std::mutex status_mutex_;
    bool climber_running_{false};       // 下位机正在执行
    bool climber_finish_once_{false};   // 下位机执行完成，待发一次2
    uint8_t last_climber_raw_{0xFF};    // 上次收到的原始状态，用于去重
    uint32_t last_control_key_{0xFFFFFFFF};  // 上次控制包 Key，用于去重 (只在变化时发 place_retry)

    std::mutex kfs_mutex_;
    std::vector<int32_t> latest_kfs_;   // 最新一帧 KFS 数据 (12 个方块), 由定时器持续重发
    bool kfs_valid_{false};             // 是否已收到过有效 KFS 帧

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

#include "at_r2_serial_bridge/cdc_trans.hpp"
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <thread>
#include <chrono>
#include <cstring>

CDCTrans::CDCTrans() : fd_(-1) {
    _disconnected     = true;
    _need_reconnected = false;
}

CDCTrans::~CDCTrans() {
    close();
}

bool CDCTrans::open(const std::string& port_name, int baudrate) {
    last_port_name = port_name;
    last_baudrate  = baudrate;
    
    RCLCPP_INFO(rclcpp::get_logger("serial_device"), "尝试打开串口设备 %s, 波特率: %d", port_name.c_str(), baudrate);
    
    fd_ = ::open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ == -1) {
        RCLCPP_WARN(rclcpp::get_logger("serial_device"), "打开串口失败: %s", port_name.c_str());
        return false;
    }

    // 配置串口
    struct termios options;
    if (tcgetattr(fd_, &options) != 0) {
        RCLCPP_WARN(rclcpp::get_logger("serial_device"), "获取串口属性失败");
        close();
        return false;
    }

    // 设置波特率
    speed_t speed = B115200;
    switch(baudrate) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        default:
            RCLCPP_WARN(rclcpp::get_logger("serial_device"), "不支持的波特率，使用默认 115200");
    }
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    // 8N1 (8位数据，无校验，1位停止位)
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    
    // 忽略调制解调器控制线，启用接收器
    options.c_cflag |= (CLOCAL | CREAD);
    
    // 禁用硬件流控
    options.c_cflag &= ~CRTSCTS;
    
    // 禁用软件流控
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    
    // 原始输入模式 (禁用特殊字符处理)
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    
    // 原始输出模式
    options.c_oflag &= ~OPOST;
    
    // 非阻塞读取
    options.c_cc[VMIN]  = 0;
    options.c_cc[VTIME] = 0;

    // 清空缓冲区并应用设置
    tcflush(fd_, TCIFLUSH);
    if (tcsetattr(fd_, TCSANOW, &options) != 0) {
        RCLCPP_WARN(rclcpp::get_logger("serial_device"), "设置串口属性失败");
        close();
        return false;
    }

    // 设置文件描述符为非阻塞模式
    int flags = fcntl(fd_, F_GETFL, 0);
    fcntl(fd_, F_SETFL, flags | O_NONBLOCK);

    _disconnected     = false;
    _need_reconnected = false;
    return true;
}

void CDCTrans::close() {
    if (fd_ != -1) {
        ::close(fd_);
        fd_ = -1;
    }
    _disconnected = true;
}

int CDCTrans::send(const uint8_t* data, int size, unsigned int time_out) {
    (void)time_out; // 暂不使用超时
    if (_disconnected || fd_ == -1)
        return -2;
        
    int rc = ::write(fd_, data, size);
    if (rc < 0) {
        RCLCPP_WARN(rclcpp::get_logger("package_comm"), "发送失败: 发送数据%d, 返回%d", size, rc);
        _disconnected = true;
        _need_reconnected = true;
        return -1;
    }
    return rc;
}

void CDCTrans::process_once() {
    if (_disconnected) {
        close();
        if (_need_reconnected) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            open(last_port_name, last_baudrate);
        }
        return;
    }

    if (fd_ == -1) return;

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(fd_, &read_fds);

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 10000; // 10ms wait

    int ret = select(fd_ + 1, &read_fds, NULL, NULL, &timeout);
    
    if (ret > 0 && FD_ISSET(fd_, &read_fds)) {
        uint8_t rx_buffer[2048];
        int bytes_read = ::read(fd_, rx_buffer, sizeof(rx_buffer));
        if (bytes_read > 0) {
            if (cdc_recv_cb) {
                cdc_recv_cb(rx_buffer, bytes_read);
            }
        } else if (bytes_read < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                RCLCPP_ERROR(rclcpp::get_logger("serial_device"), "读取串口错误，触发重连");
                _disconnected = true;
                _need_reconnected = true;
            }
        } else {
            // bytes_read == 0 normally indicates EOF (disconnected)
            RCLCPP_ERROR(rclcpp::get_logger("serial_device"), "串口断开连接，触发重连");
            _disconnected = true;
            _need_reconnected = true;
        }
    } else if (ret < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("serial_device"), "select 错误，触发重连");
        _disconnected = true;
        _need_reconnected = true;
    }
}

void CDCTrans::regeiser_recv_cb(std::function<void(const uint8_t* data, int size)> recv_cb) {
    cdc_recv_cb = std::move(recv_cb);
}

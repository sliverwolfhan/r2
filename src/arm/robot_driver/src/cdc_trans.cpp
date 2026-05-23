// Copyright 2026 RoboticArm Project Authors
// SPDX-License-Identifier: Apache-2.0
//
// CDC (Communication Device Class) USB 传输类的实现。
// 该文件通过 USB CDC（虚拟串口）协议实现与下位机的异步双向通信，
// 支持热插拔检测和自动重连功能。

#include <cdc_trans.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

CDCTrans::CDCTrans() {
    _disconnected     = true;   // 初始为断开状态
    _handling_events  = true;   // 处理事件标志
    _need_reconnected = false;  // 不需要重连
    interfaces_num    = 0;      // 接口数量
    libusb_init(&ctx);          // 初始化 libusb 上下文
}

CDCTrans::~CDCTrans() {
    _handling_events = false;  // 停止事件处理循环
    close();                   // 关闭 USB 设备并释放资源
    libusb_exit(ctx);          // 清理 libusb 上下文
}

bool CDCTrans::open(uint16_t vid, uint16_t pid) {
    last_vid             = vid;  // 保存 VID 用于重连
    last_pid             = pid;  // 保存 PID 用于重连
    this->interfaces_num = 1;    // CDC 使用接口 1

    // 根据 VID/PID 查找并打开设备
    handle = libusb_open_device_with_vid_pid(ctx, vid, pid);
    RCLCPP_INFO(rclcpp::get_logger("cdc_device"), "尝试打开USB-CDC设备");
    if (!handle) {
        RCLCPP_WARN(rclcpp::get_logger("cdc_device"), "USB-CDC打开失败");
        return false;
    }

    // 如果内核驱动已激活，则需要先分离
    if (libusb_kernel_driver_active(handle, 1)) {
        int ret = libusb_detach_kernel_driver(handle, 1);
        RCLCPP_INFO(rclcpp::get_logger("cdc_device"), 
                    "从系统内核卸载USB设备接口1，返回%d", ret);
    }

    // 申请 USB 接口控制权
    int ret = libusb_claim_interface(handle, 1);
    RCLCPP_INFO(rclcpp::get_logger("cdc_device"), "获取CDC设备通道1，返回%d", ret);

    // 分配异步传输结构体用于接收数据
    recv_transfer = libusb_alloc_transfer(0);
    RCLCPP_INFO(rclcpp::get_logger("cdc_device"), 
                "分配异步传输结构体，地址%p", (void*)recv_transfer);

    // 配置异步批量传输参数，设置接收回调
    libusb_fill_bulk_transfer(
        recv_transfer, handle, EP_IN, cdc_rx_buffer,
        sizeof(cdc_rx_buffer),
        [](libusb_transfer* transfer) -> void {
            auto self = static_cast<CDCTrans*>(transfer->user_data);

            // 检查事件处理标志，如果已停止则取消传输
            if (!self->_handling_events) {
                libusb_cancel_transfer(transfer);
                return;
            }

            // 检查传输状态，失败则标记为断开
            if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
                self->_disconnected = true;
                return;
            }

            // 调用用户注册的接收回调函数
            if (self->cdc_recv_cb)
                self->cdc_recv_cb(transfer->buffer, transfer->actual_length);

            // 重新提交异步接收请求，保持持续接收
            int rc = libusb_submit_transfer(transfer);
            if (rc != 0)
                self->_disconnected = true;
        },
        this, 0);

    // 注册热插拔回调（如果平台支持）
    if (libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG)) {
        int rc = libusb_hotplug_register_callback(
            ctx,
            static_cast<libusb_hotplug_event>(
                LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT | LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED),
            LIBUSB_HOTPLUG_NO_FLAGS, vid, pid, LIBUSB_HOTPLUG_MATCH_ANY,
            [](libusb_context* ctx, libusb_device* device, 
               libusb_hotplug_event event, void* user_data) -> int {
                static_cast<CDCTrans*>(user_data)->on_hotplug(event);
                return 0;
            },
            this, &hotplug_handle);

        if (rc != LIBUSB_SUCCESS) {
            RCLCPP_WARN(rclcpp::get_logger("cdc_device"), "热插拔回调注册失败");
        }
    }

    // 提交第一个异步接收请求，启动持续接收循环
    if (libusb_submit_transfer(recv_transfer) != 0) {
        RCLCPP_WARN(rclcpp::get_logger("cdc_device"), "请求执行数据接收失败");
    }

    _disconnected     = false;
    _handling_events  = true;
    _need_reconnected = false;
    return true;
}

void CDCTrans::close() {
    // 取消并释放异步传输结构体
    if (recv_transfer) {
        libusb_cancel_transfer(recv_transfer);
        libusb_free_transfer(recv_transfer);
        recv_transfer = nullptr;
    }

    // 释放 USB 接口并关闭设备
    if (handle) {
        libusb_release_interface(handle, interfaces_num);
        libusb_close(handle);
        handle = nullptr;
    }
}

int CDCTrans::send(const uint8_t* data, int size, unsigned int time_out) {
    int actual_size;

    // 检查设备连接状态
    if (_disconnected)
        return -2;

    // 执行同步批量传输
    int rc = libusb_bulk_transfer(
        handle, EP_OUT, (uint8_t*)data, size, &actual_size, time_out);

    if (rc != 0) {
        RCLCPP_WARN(rclcpp::get_logger("package_comm"),
                    "发送失败: 发送数据%d, 实际传输%d, 返回%d", 
                    size, actual_size, rc);
        return -1;
    }

    return actual_size;
}

void CDCTrans::process_once() {
    timeval tv = {.tv_sec = 0, .tv_usec = 50000};  // 50ms 超时

    // 处理所有待处理的 USB 事件
    libusb_handle_events_timeout_completed(ctx, &tv, nullptr);

    // 如果设备已断开，关闭连接
    if (_disconnected) {
        close();
    }

    // 如果需要重连，延迟后尝试重新打开设备
    if (_need_reconnected) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        open(last_vid, last_pid);
    }
}

void CDCTrans::register_recv_cb(
    std::function<void(const uint8_t* data, int size)> recv_cb) {
    cdc_recv_cb = std::move(recv_cb);
}

void CDCTrans::on_hotplug(libusb_hotplug_event event) {
    if (event == LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT) {
        _disconnected = true;       // 设备已拔出
    } else if (event == LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED) {
        _need_reconnected = true;   // 设备已插入，需要重连
    }
}

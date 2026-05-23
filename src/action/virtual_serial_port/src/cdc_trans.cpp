#include "virtual_serial_port/cdc_trans.hpp"
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

CDCTrans::CDCTrans() {
    _disconnected     = true;
    _handling_events  = true;
    _need_reconnected = false;
    interfaces_num    = 0;
    libusb_init(&ctx);
}

CDCTrans::~CDCTrans() {
    _handling_events = false;
    close();
    libusb_exit(ctx);
}

bool CDCTrans::open(uint16_t vid, uint16_t pid ){
    last_vid             = vid;
    last_pid             = pid;
    this->interfaces_num = 1;
    handle = libusb_open_device_with_vid_pid(ctx, vid, pid);
    RCLCPP_INFO(rclcpp::get_logger("cdc_device"),"尝试打开USB-CDC设备");
    if (!handle)
    {
        RCLCPP_WARN(rclcpp::get_logger("cdc_device"),"USB-CDC打开失败");
        return false;
    }
    
    if (libusb_kernel_driver_active(handle, 1))
    {
        int ret=libusb_detach_kernel_driver(handle, 1);
        RCLCPP_INFO(rclcpp::get_logger("cdc_device"),"从系统内核卸载USB设备接口1，返回%d",ret);
    }
    
    int ret=libusb_claim_interface(handle, 1); // 获取通道1
    RCLCPP_INFO(rclcpp::get_logger("cdc_device"),"获取CDC设备通道1，返回%d",ret);


    // 分配异步传输结构体
    recv_transfer = libusb_alloc_transfer(0);
    RCLCPP_INFO(rclcpp::get_logger("cdc_device"),"分配异步传输结构体，地址%p",(void*)recv_transfer);
    
    // 填写异步传输结构体参数
    libusb_fill_bulk_transfer(
        recv_transfer, handle, EP_IN, cdc_rx_buffer,
        sizeof(cdc_rx_buffer),
        [](libusb_transfer* transfer) -> void {
            auto self = static_cast<CDCTrans*>(transfer->user_data);
            //RCLCPP_INFO(rclcpp::get_logger("cdc_device"),"USB传输事件完成，状态%d",transfer->status);
            if (!self->_handling_events) {
                libusb_cancel_transfer(transfer);
                return;
            }
            if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
                self->_disconnected = true;
                return;
            }

            if (self->cdc_recv_cb)
                self->cdc_recv_cb(transfer->buffer, transfer->actual_length);

            int rc = libusb_submit_transfer(transfer);
            if (rc != 0)
                self->_disconnected = true;
        },
        this, 0);

    // 如果平台支持热插拔
    if (libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG)) {
        int rc = libusb_hotplug_register_callback(
            ctx,
            static_cast<libusb_hotplug_event>(
                LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT | LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED),
            LIBUSB_HOTPLUG_NO_FLAGS, vid, pid, LIBUSB_HOTPLUG_MATCH_ANY,
            [](libusb_context* ctx, libusb_device* device, libusb_hotplug_event event,
               void* user_data) -> int {
                static_cast<CDCTrans*>(user_data)->on_hotplug(event);
                return 0;
            },
            this, &hotplug_handle);

        if (rc != LIBUSB_SUCCESS) {}
    }

    // 提交异步接收请求
    if (libusb_submit_transfer(recv_transfer) != 0) {
        RCLCPP_WARN(rclcpp::get_logger("cdc_device"),"请求执行数据接收失败");
    }

    _disconnected     = false;
    _handling_events  = true;
    _need_reconnected = false;
    return true;
}

void CDCTrans::close() {
    if (recv_transfer) {
        libusb_cancel_transfer(recv_transfer);
        libusb_free_transfer(recv_transfer);
        recv_transfer = nullptr;
    }
    if (handle) {
        libusb_release_interface(handle, interfaces_num);
        libusb_close(handle);
        handle = nullptr;
    }
}

int CDCTrans::send(const uint8_t* data, int size, unsigned int time_out) {
    int actual_size;
    if(_disconnected)
        return -2;
    int rc = libusb_bulk_transfer(handle, EP_OUT, (uint8_t*)data, size, &actual_size, time_out);
    if (rc != 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("package_comm"),"发送失败:发送数据%d,实际传输%d,返回%d",size,actual_size,rc);
        
        // 如果是严重错误（设备断开、超时、管道错误），标记为断开并触发重连
        if (rc == LIBUSB_ERROR_NO_DEVICE || rc == LIBUSB_ERROR_TIMEOUT || 
            rc == LIBUSB_ERROR_PIPE || rc == LIBUSB_ERROR_IO) {
            RCLCPP_ERROR(rclcpp::get_logger("package_comm"),"USB设备通信异常，触发重连");
            _disconnected = true;
            _need_reconnected = true;
        }
        
        return -1;
    }
    else
        return actual_size;
}

void CDCTrans::process_once() {
    timeval tv = {.tv_sec = 0, .tv_usec = 50000};   //50ms
    //RCLCPP_INFO(rclcpp::get_logger("cdc_device"),"进行一次事件处理");
    libusb_handle_events_timeout_completed(ctx, &tv, nullptr);
    if (_disconnected) {
        close();
    }
    if (_need_reconnected) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        open(last_vid, last_pid);
    }
}

void CDCTrans::regeiser_recv_cb(std::function<void(const uint8_t* data, int size)> recv_cb) {
    cdc_recv_cb = std::move(recv_cb);
}

void CDCTrans::on_hotplug(libusb_hotplug_event event) {
    if (event == LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT) {
        _disconnected = true;
    } else if (event == LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED) {
        _need_reconnected = true;
    }
}

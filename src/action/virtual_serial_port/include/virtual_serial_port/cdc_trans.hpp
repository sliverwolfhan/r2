#ifndef __CDC_TRANS_H__
#define __CDC_TRANS_H__

#include <atomic>
#include <functional>
#include <libusb-1.0/libusb.h>

class CDCTrans {
public:
    static constexpr uint8_t EP_OUT = 0x01;
    static constexpr uint8_t EP_IN  = 0x81;

    CDCTrans();

    ~CDCTrans();

    bool open(uint16_t vid, uint16_t pid);

    void close();

    int send(const uint8_t* data, int size, unsigned int time_out=5); // 异步发送数据包

    void regeiser_recv_cb(
        std::function<void(const uint8_t* data, int size)> recv_cb);      // 注册数据包接收回调

    template <typename T>
    bool send_struct(const T& pack,unsigned int time_out=5) {
        static_assert(std::is_standard_layout<T>::value, "结构体不是标准构型");
        static_assert(std::is_trivial<T>::value, "数据包必须是可复制的");

        constexpr int pack_size = sizeof(T);

        send(reinterpret_cast<const uint8_t*>(&pack), pack_size,time_out);
        return true;
    }

    void process_once();                                            // 处理一次事件
private:
    void on_hotplug(libusb_hotplug_event event);                    // 热插拔中

    uint16_t last_vid, last_pid;
    int interfaces_num;
    std::function<void(const uint8_t* data, int size)> cdc_recv_cb;
    std::atomic_bool _disconnected;
    std::atomic_bool _need_reconnected;
    std::atomic_bool _handling_events;
    uint8_t cdc_rx_buffer[2048];
    libusb_transfer* recv_transfer;
    libusb_context* ctx;
    libusb_device_handle* handle;
    libusb_hotplug_callback_handle hotplug_handle;
};

#endif
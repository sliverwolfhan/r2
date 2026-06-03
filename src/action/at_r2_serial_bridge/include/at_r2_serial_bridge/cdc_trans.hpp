#ifndef __CDC_TRANS_H__
#define __CDC_TRANS_H__

#include <atomic>
#include <functional>
#include <string>

class CDCTrans {
public:
    CDCTrans();
    ~CDCTrans();

    bool open(const std::string& port_name, int baudrate);
    void close();

    int send(const uint8_t* data, int size, unsigned int time_out=5); // 异步/同步发送数据包

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
    std::string last_port_name;
    int last_baudrate;
    int fd_;
    std::function<void(const uint8_t* data, int size)> cdc_recv_cb;
    std::atomic_bool _disconnected;
    std::atomic_bool _need_reconnected;
};

#endif
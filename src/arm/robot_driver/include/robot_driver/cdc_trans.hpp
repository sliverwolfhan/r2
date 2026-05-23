// Copyright 2026 RoboticArm Project Authors
// SPDX-License-Identifier: Apache-2.0
//
// CDC (Communication Device Class) USB transmission class.
// This file provides asynchronous USB CDC communication capabilities,
// including device connection management, data transmission, and
// hot-plug event handling.

#ifndef __CDC_TRANS_H__
#define __CDC_TRANS_H__

#include <atomic>
#include <functional>
#include <libusb-1.0/libusb.h>

// CDCTrans provides USB CDC (Communication Device Class) communication
// functionality using libusb library. It supports asynchronous data
// transmission, hot-plug detection, and callback-based data reception.
//
// This class is designed for communicating with USB CDC devices such as
// virtual serial ports. It handles device connection/disconnection events
// and provides a simple interface for sending and receiving data.
//
// Example usage:
//   CDCTrans trans;
//   trans.open(0x1234, 0x5678);  // Open device with VID/PID
//   trans.register_recv_cb([](const uint8_t* data, int size) {
//     // Handle received data
//   });
//   trans.send(data, size);
//   trans.process_once();  // Process USB events
class CDCTrans {
public:
    // USB endpoint addresses for CDC communication
    static constexpr uint8_t EP_OUT = 0x01;  // OUT endpoint for sending data
    static constexpr uint8_t EP_IN  = 0x81;  // IN endpoint for receiving data

    // Constructs a CDCTrans instance with uninitialized state.
    CDCTrans();

    // Destructs the instance and releases all USB resources.
    ~CDCTrans();

    // Opens a USB CDC device with the specified vendor and product ID.
    //
    // Args:
    //   vid: USB vendor ID
    //   pid: USB product ID
    //
    // Returns:
    //   true if the device was successfully opened, false otherwise.
    bool open(uint16_t vid, uint16_t pid);

    // Closes the USB device and releases associated resources.
    void close();

    // Asynchronously sends a data packet to the USB device.
    //
    // Args:
    //   data: Pointer to the data buffer to send
    //   size: Number of bytes to send
    //   time_out: Timeout in milliseconds (default: 5ms)
    //
    // Returns:
    //   Number of bytes actually sent, or negative value on error.
    int send(const uint8_t* data, int size, unsigned int time_out=5);

    // Registers a callback function to handle received data packets.
    //
    // Args:
    //   recv_cb: Callback function that receives data buffer and size
    void register_recv_cb(
        std::function<void(const uint8_t* data, int size)> recv_cb);

    // Sends a structure as a data packet.
    //
    // This template method serializes and sends a structure directly.
    // The structure must have standard layout and be trivially copyable.
    //
    // Template args:
    //   T: Structure type to send
    //
    // Args:
    //   pack: Reference to the structure instance to send
    //   time_out: Timeout in milliseconds (default: 5ms)
    //
    // Returns:
    //   true if the send operation was initiated successfully
    template <typename T>
    bool send_struct(const T& pack, unsigned int time_out=5) {
        static_assert(std::is_standard_layout<T>::value, 
            "Structure must have standard layout");
        static_assert(std::is_trivial<T>::value, 
            "Structure must be trivially copyable");

        constexpr int pack_size = sizeof(T);
        send(reinterpret_cast<const uint8_t*>(&pack), pack_size, time_out);
        return true;
    }

    // Processes pending USB events once.
    // This should be called regularly in the main loop to handle
    // asynchronous transfers and hot-plug events.
    void process_once();

private:
    // Handles USB hot-plug events (connection/disconnection).
    //
    // Args:
    //   event: The hot-plug event type
    void on_hotplug(libusb_hotplug_event event);

    // Last opened device vendor ID
    uint16_t last_vid;
    // Last opened device product ID
    uint16_t last_pid;
    // Number of interfaces claimed from the device
    int interfaces_num;
    // Callback function for received data
    std::function<void(const uint8_t* data, int size)> cdc_recv_cb;
    // Flag indicating device disconnection
    std::atomic_bool _disconnected;
    // Flag indicating need for reconnection
    std::atomic_bool _need_reconnected;
    // Flag indicating events are being processed
    std::atomic_bool _handling_events;
    // Buffer for receiving CDC data
    uint8_t cdc_rx_buffer[2048];
    // Libusb transfer structure for asynchronous reception
    libusb_transfer* recv_transfer;
    // Libusb context
    libusb_context* ctx;
    // Libusb device handle
    libusb_device_handle* handle;
    // Handle for hot-plug callback registration
    libusb_hotplug_callback_handle hotplug_handle;
};

#endif
#pragma once
#include <thread>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <iostream>
#include <atomic>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

class UdpReadingThread {
    public:
        UdpReadingThread();
        ~UdpReadingThread();

        int sock_;
        
        int socketOpen(std::string ip, int port);
        void socketClose();
        void loop();

        std::atomic<int16_t> joystick_x_;
        std::atomic<int16_t> joystick_y_;
        std::atomic<uint16_t> joystick_btn_;

        std::mutex joystick_mutex_;
    private:
        
};

// joystick (buffer[2] | buffer[3] << 8) // 메뉴얼에는 -127 ... 128 for each coordinate
// +y 0x0000 ~ 0x7f00 ---hex to decimal---> 0, 32512
// -y 0x0000 ~ 0x8200 ---hex to decimal---> 0, -32256
// +x 0x0000 ~ 0x007f ---hex to decimal---> 0, 127
// -x 0x0000 ~ 0x0081 ---hex to decimal---> 0, 129

// joystick button (buffer[4] | buffer[5] << 8)
// 0x0000, 0x0100 ---hex to decimal---> 0, 
// - button
// 0x0000, 0x0200 ---hex to decimal--->
// + button
// 0x0000, 0x0400 ---hex to decimal--->
// grid button
// 0x0000, 0x0800 ---hex to decimal--->
// jogging button
// 0x0000, 0x1000 ---hex to decimal--->
// f1 button
// 0x0000, 0x2000 ---hex to decimal--->
// f2 button
// 0x0000, 0x4000 ---hex to decimal--->
// f3 button
// 0x0000, 0x8000 ---hex to decimal--->
// f4 button
// 0x0000, 0x0001 ---hex to decimal--->
// 나침반 1 button
// 0x0000, 0x0002 ---hex to decimal--->
// 나침반 2 button
// 0x0000, 0x0004 ---hex to decimal--->

// enabling button 릴레이 연결 후 가능할듯?
// emergency button 릴레이 연결 후 가능할듯?

// Robot's max vel is 1m/s (linear.x, linear.y, angular.z)
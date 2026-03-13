#pragma once
#include <iostream>
#include <chrono>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <joystick_controller/udp_read_thread.hpp>
#include <joystick_controller/data_processor.hpp>

class JoystickControllerNode : public rclcpp::Node {
    public :
        JoystickControllerNode();
        ~JoystickControllerNode();

    private :
        enum class DrivingMode {
            MANULFINISH,
            STAY,
            CURVE,
            CRAB,
            ROTATE
        };

        std::string joystick_ip_;
        int joystick_port_;
        float max_vel_x_;
        float max_vel_y_;
        float max_vel_z_;
        float accel_lin_;
        float accel_ang_;
        float dt_;

        DrivingMode current_driving_mode_;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr manual_vel_pub_;

        rclcpp::TimerBase::SharedPtr manual_vel_timer_;
        std::thread udp_thread_;
        
        UdpReadingThread udp_reading_;
        DataProcessor data_processor_;

        void manual_vel_pub_callback();
        void set_driving_mode_status(DrivingMode new_driving_mode);
        void switch_driving_mode();

        // udp 데이터 읽기 w/ thread

        // data processor stored data read

        // publish
};
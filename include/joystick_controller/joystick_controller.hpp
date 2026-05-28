#pragma once
#include <iostream>
#include <chrono>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <joystick_controller/udp_read_thread.hpp>
#include <joystick_controller/data_processor.hpp>
#include <joystick_controller/driving_mode.hpp>

class JoystickControllerNode : public rclcpp::Node {
    public :
        JoystickControllerNode();
        ~JoystickControllerNode();

    private :
        std::string joystick_ip_;
        int joystick_port_;
        float max_vel_x_;
        float max_vel_y_;
        float max_vel_z_;
        float accel_lin_;
        float accel_ang_;
        float dt_;
        bool manual_mode_msg_;
        bool prev_speed_level_btn_status_ = false;
        int speed_level_ = 1;

        DrivingMode current_driving_mode_;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr manual_vel_pub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_mode_sub_;

        rclcpp::TimerBase::SharedPtr manual_vel_timer_;
        rclcpp::TimerBase::SharedPtr speed_level_btn_timer_;
        std::thread udp_thread_;
        
        UdpReadingThread udp_reading_;
        DataProcessor data_processor_;

        void manual_vel_pub_callback();
        void manual_mode_sub_callback(const std_msgs::msg::Bool::SharedPtr msg);
        void speed_level_btn_callback();
        void set_driving_mode_status(DrivingMode new_driving_mode);
        void switch_driving_mode();

        // udp 데이터 읽기 w/ thread

        // data processor stored data read

        // publish
};
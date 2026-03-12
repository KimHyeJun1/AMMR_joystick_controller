#include <joystick_controller/data_processor.hpp>

DataProcessor::DataProcessor(UdpReadingThread* udp_thread) {
    udp_thread_ = udp_thread;
};

void DataProcessor::joy_xy_to_vel_mapping(){
    max_vel_x_ = 1.0;
    max_vel_y_ = 1.0;
    max_vel_z_ = 1.0;

    int16_t joy_x = udp_thread_->joystick_x_;
    int16_t joy_y = udp_thread_->joystick_y_;

    manual_vel_x_.store((joy_x / 126.0) * max_vel_x_); // crab, curve mode 
    manual_vel_y_.store((joy_y / 126.0) * max_vel_y_); // crab
    manual_vel_z_.store((joy_y / 126.0) * max_vel_z_); // crab, curve mode
}

DataProcessor::~DataProcessor() {};


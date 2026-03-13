#include <joystick_controller/data_processor.hpp>

DataProcessor::DataProcessor(UdpReadingThread* udp_thread) {
    udp_thread_ = udp_thread;
};

void DataProcessor::joy_xy_to_vel_mapping(const float max_vel_x, const float max_vel_y, const float max_vel_z){
    int16_t joy_x = udp_thread_->joystick_x_;
    int16_t joy_y = udp_thread_->joystick_y_;

    target_vel_x_ = (joy_x / 126.0) * max_vel_x;
    target_vel_y_ = (joy_y / 126.0) * max_vel_y;
    target_vel_z_ = (joy_y / 126.0) * max_vel_z;

    // manual_vel_x_.store((joy_x / 126.0) * max_vel_x); // crab, curve mode 
    // manual_vel_y_.store((joy_y / 126.0) * max_vel_y); // crab
    // manual_vel_z_.store((joy_y / 126.0) * max_vel_z); // crab, curve mode
}

float DataProcessor::ramp(float target, float current, float accel, float dt) {
    float diff = target - current;
    double max_step = accel * dt;

    if (fabs(diff) < max_step)
        return target;
    
    if (diff > 0)
        return current + max_step;
    else
        return current - max_step;
}

void DataProcessor::update_velocity(
    float accel_lin,
    float accel_ang,
    float dt)
{
    current_vel_x_ = ramp(target_vel_x_, current_vel_x_, accel_lin, dt);
    current_vel_y_ = ramp(target_vel_y_, current_vel_y_, accel_lin, dt);
    current_vel_z_ = ramp(target_vel_z_, current_vel_z_, accel_ang, dt);

    manual_vel_x_.store(current_vel_x_);
    manual_vel_y_.store(current_vel_y_);
    manual_vel_z_.store(current_vel_z_);
}

DataProcessor::~DataProcessor() {};


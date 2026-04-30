#include <joystick_controller/data_processor.hpp>

DataProcessor::DataProcessor(UdpReadingThread* udp_thread) {
    udp_thread_ = udp_thread;
};

void DataProcessor::joy_xy_to_vel_mapping(const float max_vel_x, const float max_vel_y, const float max_vel_z, DrivingMode mode){
    current_mode_ = mode;
    int16_t joy_x = udp_thread_->joystick_x_;
    int16_t joy_y = udp_thread_->joystick_y_;

    if (abs(joy_x) < 1) {
        joy_x = 0;
    }
    if (abs(joy_y) < 1) {
        joy_y = 0;
    }

    float norm_x = joy_x / 126.0;
    float norm_y = joy_y / 126.0;

    if (current_mode_ == DrivingMode::CURVE) {
        target_vel_x_ = norm_x * max_vel_x;
        target_vel_y_ = 0;
        target_vel_z_ = norm_y * max_vel_y;
    } else if (current_mode_ == DrivingMode::CRAB) {
        float angle = atan2(norm_y, norm_x);
        float angle_deg = angle * 180.0 / M_PI;

        if (angle_deg < 0) {
            angle_deg += 360.0;
        }

        float snapped_deg = round(angle_deg / 30.0) * 30.0;
        float snapped_rad = snapped_deg * M_PI / 180.0;

        float mag = sqrt(norm_x * norm_x + norm_y * norm_y);

        target_vel_x_ = cos(snapped_rad) * mag * max_vel_x;
        target_vel_y_ = sin(snapped_rad) * mag * max_vel_y;
        target_vel_z_ = 0;
        if (fabs(target_vel_x_) < 1e-5) target_vel_x_ = 0;
        if (fabs(target_vel_y_) < 1e-5) target_vel_y_ = 0;

    } else if (current_mode_ == DrivingMode::ROTATE) {
        target_vel_x_ = 0;
        target_vel_y_ = 0;
        target_vel_z_ = norm_y * max_vel_z;
    }
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


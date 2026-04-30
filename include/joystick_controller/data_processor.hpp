#pragma once
#include <atomic>
#include <cmath>
#include <joystick_controller/udp_read_thread.hpp>
#include <joystick_controller/driving_mode.hpp>
class DataProcessor {
    public:
        DataProcessor(UdpReadingThread* udp_thread);
        ~DataProcessor();
        std::atomic<float> manual_vel_x_;
        std::atomic<float> manual_vel_y_;
        std::atomic<float> manual_vel_z_;

        DrivingMode current_mode_;

        void joy_xy_to_vel_mapping(const float max_vel_x, const float max_vel_y, const float max_vel_z, DrivingMode mode);
        void update_velocity(const float accel_lin, const float accel_ang, const float dt);

    private:
        UdpReadingThread* udp_thread_;

        float target_vel_x_;
        float target_vel_y_;
        float target_vel_z_;

        float current_vel_x_;
        float current_vel_y_;
        float current_vel_z_;

        float ramp(float target, float current, float accel, float dt);
        
        // 20. 데이터 처리 - 변환 (analog -> m/s)
        // 21. 데이터 처리 - 매핑? (몇 부터 몇까지 몇 m/s)
        // 30. 데이터 처리 - 스케일링
        // 40. 데이터 처리 - 데이터 저장
};
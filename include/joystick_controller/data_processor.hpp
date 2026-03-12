#pragma once
#include <atomic>
#include <joystick_controller/udp_read_thread.hpp>

class DataProcessor {
    public:
        DataProcessor(UdpReadingThread* udp_thread);
        ~DataProcessor();
        std::atomic<float> manual_vel_x_;
        std::atomic<float> manual_vel_y_;
        std::atomic<float> manual_vel_z_;

        void joy_xy_to_vel_mapping(const float max_vel_x, const float max_vel_y, const float max_vel_z);

    private:
        UdpReadingThread* udp_thread_;
        // 20. 데이터 처리 - 변환 (analog -> m/s)
        // 21. 데이터 처리 - 매핑? (몇 부터 몇까지 몇 m/s)
        // 30. 데이터 처리 - 스케일링
        // 40. 데이터 처리 - 데이터 저장
};
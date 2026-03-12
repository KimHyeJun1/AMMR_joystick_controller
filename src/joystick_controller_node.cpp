#include <joystick_controller/joystick_controller.hpp>

JoystickControllerNode::JoystickControllerNode() : Node("joystick_controller_node"), 
    data_processor_(&udp_reading_) {
    udp_reading_.socketOpen();
    RCLCPP_INFO(this->get_logger(), "socket is opened");

    udp_thread_ = std::thread(&UdpReadingThread::loop, &udp_reading_);

    manual_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("manual_vel", 10);

    manual_vel_timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&JoystickControllerNode::manual_vel_pub_callback, this));

}

void JoystickControllerNode::manual_vel_pub_callback() {
    auto msg = geometry_msgs::msg::Twist();

    data_processor_.joy_xy_to_vel_mapping();

    float vel_x = data_processor_.manual_vel_x_;
    float vel_y = data_processor_.manual_vel_y_;
    float vel_z = data_processor_.manual_vel_z_;

    switch_driving_mode();
    switch (current_driving_mode_)
    {
    case DrivingMode::STAY:
        msg.linear.x = 0;
        msg.linear.y = 0;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = 0;
        break;
    case DrivingMode::CURVE:
        msg.linear.x = vel_x;
        msg.linear.y = 0;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = vel_z;
        break;
    case DrivingMode::CRAB:
        if (fabs(vel_x) > fabs(vel_y)) {
            msg.linear.x = vel_x;
            msg.linear.y = 0;
            msg.linear.z = 0;
            msg.angular.x = 0;
            msg.angular.y = 0;
            msg.angular.z = 0;
        } else {
            msg.linear.x = 0;
            msg.linear.y = vel_y;
            msg.linear.z = 0;
            msg.angular.x = 0;
            msg.angular.y = 0;
            msg.angular.z = 0;
        }
        break;
    default:
        break;
    }

    RCLCPP_INFO(this->get_logger(), "vel_x: %f, vel_z: %f", vel_x, vel_z);
    manual_vel_pub_->publish(msg);
}

JoystickControllerNode::~JoystickControllerNode() {
    udp_reading_.socketClose();

    if(udp_thread_.joinable())
        udp_thread_.join();
}

void JoystickControllerNode::set_driving_mode_status(DrivingMode new_driving_mode) {
    current_driving_mode_ = new_driving_mode;
}

void JoystickControllerNode::switch_driving_mode() {
    switch (udp_reading_.joystick_btn_) {
    // case 0x0100:
    //     RCLCPP_INFO(this->get_logger(), "joystick button selected");
    //     break;
    // case 0x0200:
    //     RCLCPP_INFO(this->get_logger(), "- button selected");
    //     break;
    // case 0x0400:
    //     RCLCPP_INFO(this->get_logger(), "+ button selected");
    //     break;
    // case 0x0800:
    //     RCLCPP_INFO(this->get_logger(), "grid button selected");
    //     break;
    // case 0x1000:
    //     RCLCPP_INFO(this->get_logger(), "jogging button selected");
    //     break;
    case 32:
        set_driving_mode_status(DrivingMode::STAY);
        RCLCPP_INFO(this->get_logger(), "f1 button selected");
        break;
    case 64:
        set_driving_mode_status(DrivingMode::CURVE);
        RCLCPP_INFO(this->get_logger(), "f2 button selected");
        break;
    case 128:
        set_driving_mode_status(DrivingMode::CRAB);
        RCLCPP_INFO(this->get_logger(), "f3 button selected");
        break;
    // case 0x0001:
    //     RCLCPP_INFO(this->get_logger(), "f4 button selected");
    //     break;
    // case 0x0002:
    //     RCLCPP_INFO(this->get_logger(), "나침반 button selected");
    //     break;
    // case 0x0004:
    //     RCLCPP_INFO(this->get_logger(), "나침반2 button selected");
    //     break;
    default:
        // if (current_driving_mode_ == DrivingMode::STAY) {
        //     RCLCPP_INFO(this->get_logger(), "1");
        // } else if (current_driving_mode_ == DrivingMode::CURVE) {
        //     RCLCPP_INFO(this->get_logger(), "2");
        // } else {
        //     RCLCPP_INFO(this->get_logger(), "3");
        // }
        break;
    }
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoystickControllerNode>());
    rclcpp::shutdown();
    return 0;
}
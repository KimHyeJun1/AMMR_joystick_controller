#include <joystick_controller/joystick_controller.hpp>

using std::placeholders::_1;

JoystickControllerNode::JoystickControllerNode() : Node("joystick_controller_node"), 
    data_processor_(&udp_reading_) {
    this->declare_parameter<std::string>("joystick_ip", "192.168.10.67");
    this->declare_parameter<int>("joystick_port", 52922);
    this->declare_parameter<float>("max_vel_x", 1.0);
    this->declare_parameter<float>("max_vel_y", 1.0);
    this->declare_parameter<float>("max_vel_z", 1.0);
    this->declare_parameter<float>("accel_lin", 1.0);
    this->declare_parameter<float>("accel_ang", 1.0);
    this->declare_parameter<float>("dt", 0.02);

    joystick_ip_ = this->get_parameter("joystick_ip").as_string();
    joystick_port_ = this->get_parameter("joystick_port").as_int();
    max_vel_x_ = this->get_parameter("max_vel_x").as_double();
    max_vel_y_ = this->get_parameter("max_vel_y").as_double();
    max_vel_z_ = this->get_parameter("max_vel_z").as_double();
    accel_lin_ = this->get_parameter("accel_lin").as_double();
    accel_ang_ = this->get_parameter("accel_ang").as_double();
    dt_ = this->get_parameter("dt").as_double();

    udp_reading_.socketOpen(joystick_ip_, joystick_port_);
    RCLCPP_INFO(this->get_logger(), "joystick_ip : %s, joystick_port : %d socket is opened", joystick_ip_.c_str(), joystick_port_);
    RCLCPP_INFO(this->get_logger(), "max_vel_x : %f, max_vel_y : %f max_vel_z : %f, accel_lin : %f, accel_ang : %f, dt : %f", max_vel_x_, max_vel_y_, max_vel_z_, accel_lin_, accel_ang_, dt_);

    udp_thread_ = std::thread(&UdpReadingThread::loop, &udp_reading_);

    manual_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("joy_vel", 10);

    manual_mode_sub_ = this->create_subscription<std_msgs::msg::Bool>("manual_mode", 10, std::bind(&JoystickControllerNode::manual_mode_sub_callback, this, _1));

    manual_vel_timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&JoystickControllerNode::manual_vel_pub_callback, this));

    speed_level_btn_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&JoystickControllerNode::speed_level_btn_callback, this));
}

void JoystickControllerNode::speed_level_btn_callback() {
    bool current_speed_level_btn_status_ = false;

    switch (udp_reading_.joystick_btn_) {
        case 2:
            // RCLCPP_DEBUG(this->get_logger(), "- button selected");
            current_speed_level_btn_status_ = true;
            if (current_speed_level_btn_status_ && !prev_speed_level_btn_status_) {
                if (speed_level_ <= 1) {
                   speed_level_ = 1;
                } else {
                    speed_level_--;
                }
                prev_speed_level_btn_status_ = current_speed_level_btn_status_;
                RCLCPP_INFO(this->get_logger(), "Set Speed Level = %d", speed_level_);
            } else {
                RCLCPP_DEBUG(this->get_logger(), "Speed Level Btn Input Duplicate.");
            }
            
            break;
        case 4:
            // RCLCPP_DEBUG(this->get_logger(), "+ button selected");
            current_speed_level_btn_status_ = true;
            if (current_speed_level_btn_status_ && !prev_speed_level_btn_status_) {
                if (speed_level_ >= 10) {
                   speed_level_ = 10;
                    RCLCPP_DEBUG(this->get_logger(), "Set Speed Level = %d", speed_level_);
                } else {
                    speed_level_++;
                }
                RCLCPP_INFO(this->get_logger(), "Set Speed Level = %d", speed_level_);
            } else {
                RCLCPP_DEBUG(this->get_logger(), "Speed Level Btn Input Duplicate.");
            }
            break;
        default:
            current_speed_level_btn_status_ = false;
            break;
    }
    prev_speed_level_btn_status_ = current_speed_level_btn_status_;
}

void JoystickControllerNode::manual_mode_sub_callback (const std_msgs::msg::Bool::SharedPtr msg) {
    manual_mode_msg_ = msg->data;
    RCLCPP_DEBUG(this->get_logger(), "manual_mode's msg %d", manual_mode_msg_);
}

void JoystickControllerNode::manual_vel_pub_callback() {
    auto msg = geometry_msgs::msg::Twist();

    // if (manual_mode_msg_ == true) {
        data_processor_.joy_xy_to_vel_mapping(max_vel_x_, max_vel_y_, max_vel_z_, current_driving_mode_);
        data_processor_.update_velocity(accel_lin_, accel_ang_, dt_);

        float vel_x = data_processor_.manual_vel_x_ * (speed_level_ / 10);
        float vel_y = data_processor_.manual_vel_y_ * (speed_level_ / 10);
        float vel_z = data_processor_.manual_vel_z_ * (speed_level_ / 10);

        switch_driving_mode();

        if (current_driving_mode_ == DrivingMode::STAY) {
            msg.linear.x = 0;
            msg.linear.y = 0;
            msg.linear.z = 0;
            msg.angular.x = 0;
            msg.angular.y = 0;
            msg.angular.z = 0;
        } else if (current_driving_mode_ == DrivingMode::CURVE) {
            msg.linear.x = vel_x;
            msg.linear.y = 0;
            msg.linear.z = 0;
            msg.angular.x = 0;
            msg.angular.y = 0;
            if (vel_x >= 0) {
                msg.angular.z = vel_z * -1;
            } else {
                msg.angular.z = vel_z;
            }
        } else if (current_driving_mode_ == DrivingMode::CRAB) {
            msg.linear.x = vel_x;
            msg.linear.y = vel_y * -1;
            msg.linear.z = 0;
            msg.angular.x = 0;
            msg.angular.y = 0;
            msg.angular.z = 0;
            // if (fabs(vel_x) > fabs(vel_y)) {
            //     msg.linear.x = vel_x;
            //     msg.linear.y = 0;
            //     msg.linear.z = 0;
            //     msg.angular.x = 0;
            //     msg.angular.y = 0;
            //     msg.angular.z = 0;
            // } else {
            //     msg.linear.x = 0;
            //     msg.linear.y = vel_y * -1;
            //     msg.linear.z = 0;
            //     msg.angular.x = 0;
            //     msg.angular.y = 0;
            //     msg.angular.z = 0;
            // }
        } else if (current_driving_mode_ == DrivingMode::ROTATE) {
            msg.linear.x = 0;
            msg.linear.y = 0;
            msg.linear.z = 0;
            msg.angular.x = 0;
            msg.angular.y = 0;
            msg.angular.z = vel_z * -1;
        }
        // switch (current_driving_mode_)
        // {
        // case DrivingMode::STAY:
        //     msg.linear.x = 0;
        //     msg.linear.y = 0;
        //     msg.linear.z = 0;
        //     msg.angular.x = 0;
        //     msg.angular.y = 0;
        //     msg.angular.z = 0;
        //     break;
        // case DrivingMode::CURVE:
        //     msg.linear.x = vel_x;
        //     msg.linear.y = 0;
        //     msg.linear.z = 0;
        //     msg.angular.x = 0;
        //     msg.angular.y = 0;
        //     msg.angular.z = vel_z;
        //     break;
        // case DrivingMode::CRAB:
        //     if (fabs(vel_x) > fabs(vel_y)) {
        //         msg.linear.x = vel_x;
        //         msg.linear.y = 0;
        //         msg.linear.z = 0;
        //         msg.angular.x = 0;
        //         msg.angular.y = 0;
        //         msg.angular.z = 0;
        //     } else {
        //         msg.linear.x = 0;
        //         msg.linear.y = vel_y;
        //         msg.linear.z = 0;
        //         msg.angular.x = 0;
        //         msg.angular.y = 0;
        //         msg.angular.z = 0;
        //     }
        //     break;
        // default:
        //     msg.linear.x = 0;
        //     msg.linear.y = 0;
        //     msg.linear.z = 0;
        //     msg.angular.x = 0;
        //     msg.angular.y = 0;
        //     msg.angular.z = 0;
        //     break;
        // }

        // RCLCPP_DEBUG(this->get_logger(), "vel_x: %f, vel_z: %f", vel_x, vel_z);
        if (current_driving_mode_ == DrivingMode::MANULFINISH) {
            RCLCPP_DEBUG(this->get_logger(), "manual mode finished.");
        } else {
            // RCLCPP_DEBUG(this->get_logger(), "lin_x: %f, lin_y: %f, ang_z: %f", msg.linear.x, msg.linear.y, msg.angular.z);
            manual_vel_pub_->publish(msg);
        }
    // } else {
    //     return;
    // }
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
    // case 1:
    //     RCLCPP_DEBUG(this->get_logger(), "joystick button selected");
    //     break;
    // case 2:
        // RCLCPP_DEBUG(this->get_logger(), "- button selected");
        // break;
    // case 4:
    //     RCLCPP_DEBUG(this->get_logger(), "+ button selected");
    //     break;
    case 8:
        set_driving_mode_status(DrivingMode::MANULFINISH);
        // RCLCPP_DEBUG(this->get_logger(), "grid button selected");
        RCLCPP_DEBUG(this->get_logger(), "Driving Mode : MANUALFINISH");
        break;
    // case 16:
    //     RCLCPP_DEBUG(this->get_logger(), "jogging button selected");
    //     break;
    case 32:
        set_driving_mode_status(DrivingMode::STAY);
        // RCLCPP_DEBUG(this->get_logger(), "f1 button selected");
        RCLCPP_DEBUG(this->get_logger(), "Driving Mode : STAY");
        break;
    case 64:
        set_driving_mode_status(DrivingMode::CURVE);
        // RCLCPP_DEBUG(this->get_logger(), "f2 button selected");
        RCLCPP_DEBUG(this->get_logger(), "Driving Mode : CURVE");
        break;
    case 128:
        set_driving_mode_status(DrivingMode::CRAB);
        // RCLCPP_DEBUG(this->get_logger(), "f3 button selected");
        RCLCPP_DEBUG(this->get_logger(), "Driving Mode : CRAB");
        break;
    case 256:
        set_driving_mode_status(DrivingMode::ROTATE);
        // RCLCPP_DEBUG(this->get_logger(), "f4 button selected");
        RCLCPP_DEBUG(this->get_logger(), "Driving Mode : ROTATE");
        break;
    // case 512:
    //     RCLCPP_DEBUG(this->get_logger(), "나침반 button selected");
    //     break;
    // case 1024:
    //     RCLCPP_DEBUG(this->get_logger(), "나침반2 button selected");
    //     break;
    default:
        // if (current_driving_mode_ == DrivingMode::STAY) {
        //     RCLCPP_DEBUG(this->get_logger(), "1");
        // } else if (current_driving_mode_ == DrivingMode::CURVE) {
        //     RCLCPP_DEBUG(this->get_logger(), "2");
        // } else {
        //     RCLCPP_DEBUG(this->get_logger(), "3");
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
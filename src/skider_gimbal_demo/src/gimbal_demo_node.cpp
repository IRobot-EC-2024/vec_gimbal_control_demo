/*
 * @Description: 
 * @Version: 1.0
 * @Autor: Qylann
 * @Date: 2022-03-30 19:48:35
 * @LastEditors: Qylann
 * @LastEditTime: 2022-04-05 12:42:33
 */
#include "gimbal_demo_node.hpp"

GimbalControlerDemoNode::GimbalControlerDemoNode(const rclcpp::NodeOptions & options)
{
    gimbal_controler_demo_node_ = std::make_shared<rclcpp::Node>("gimbal_controler_node", options);
    RCLCPP_INFO(gimbal_controler_demo_node_->get_logger(), "Node Begin");

    std::string imu_subscribe_topic_name_("/skider/imu/data");
    RCLCPP_INFO(gimbal_controler_demo_node_->get_logger(), "Subscribe IMU data : \"%s\"", imu_subscribe_topic_name_.c_str());
    imu_subscription_ = gimbal_controler_demo_node_->create_subscription<sensor_msgs::msg::Imu>(
        imu_subscribe_topic_name_, 10, std::bind(&GimbalControlerDemoNode::imu_msg_callback, this, std::placeholders::_1));

    std::string joy_subscribe_topic_name_("/skider/joy/data");
    RCLCPP_INFO(gimbal_controler_demo_node_->get_logger(), "Subscribe JOY data : \"%s\"", joy_subscribe_topic_name_.c_str());
    joy_subscription_ = gimbal_controler_demo_node_->create_subscription<sensor_msgs::msg::Joy>(
        joy_subscribe_topic_name_, 10, std::bind(&GimbalControlerDemoNode::joy_msg_callback, this, std::placeholders::_1));

    std::string gimbal_output_publish_topic_name_("/skider/output/gimbal");
    RCLCPP_INFO(gimbal_controler_demo_node_->get_logger(), "Init Gimbal Output Publisher : \"%s\"", gimbal_output_publish_topic_name_.c_str());
    gimbal_output_publisher_ = gimbal_controler_demo_node_->create_publisher<skider_excutor::msg::GimbalOutput>(
        gimbal_output_publish_topic_name_, 10);

    std::string shooter_output_publish_topic_name_("/skider/output/shooter");
    RCLCPP_INFO(gimbal_controler_demo_node_->get_logger(), "Init Gimbal Output Publisher : \"%s\"", shooter_output_publish_topic_name_.c_str());
    shooter_output_publisher_ = gimbal_controler_demo_node_->create_publisher<skider_excutor::msg::ShooterOutput>(
        shooter_output_publish_topic_name_, 10);

    RCLCPP_INFO(gimbal_controler_demo_node_->get_logger(), "Finish Init");

    yaw_angle_set_ = 0;
    pitch_angle_set_ = 0;
}

inline double speed_limit(double input, double max)
{
    if (input > max) {
        return max;
    }
    else if (input < -max) {
        return -max;
    }
    return input;
}

inline double get_relative_angle(double angle_aim, double angle_ref)
{
    double reletive_angle = angle_aim - angle_ref;

    while (reletive_angle > M_PI) {
        reletive_angle -= 2*M_PI;
    }
    while (reletive_angle < -M_PI) {
        reletive_angle += 2*M_PI;
    }
    
    return reletive_angle;
}

inline double aim_loop(double angle_aim)
{
    while (angle_aim > M_PI) {
        angle_aim -= 2*M_PI;
    }
    while (angle_aim < -M_PI) {
        angle_aim += 2*M_PI;
    }
    
    return angle_aim;
}

inline double aim_limut(double angle_aim, double max, double min)
{
    while (angle_aim > max) {
        return max;
    }
    while (angle_aim < min) {
        return min;
    }
    
    return angle_aim;
}



void GimbalControlerDemoNode::joy_msg_callback(const sensor_msgs::msg::Joy & msg)
{
    skider_excutor::msg::GimbalOutput gimbal;
    gimbal.header.set__frame_id("Controler Demo Gimbal Output");
    gimbal.header.set__stamp(gimbal_controler_demo_node_->get_clock()->now());
    if ((msg.buttons[1] == true)  ||  (msg.buttons[2] == true)) {
        yaw_angle_set_ = aim_loop(yaw_angle_set_ + (-msg.axes[2])*0.1);
        pitch_angle_set_ = aim_limut((pitch_angle_set_ + (-msg.axes[3])*0.03), 0.25, -0.4);

        double yaw_relative = get_relative_angle(yaw_angle_set_, imu_yaw_);
        int16_t yaw_speed_set = speed_limit((2000*yaw_relative), 500);
        double pitch_relative = get_relative_angle(pitch_angle_set_, imu_pitch_);
        int16_t pitch_speed_set = speed_limit((2000*pitch_relative), 300);

        gimbal.set__enable(true);
        gimbal.set__pitch_kp(300);
        gimbal.set__yaw_kp(300);
        gimbal.set__pitch_speed(pitch_speed_set);
        gimbal.set__yaw_speed(yaw_speed_set);

        // RCLCPP_INFO(gimbal_controler_demo_node_->get_logger(), "=>> yaw_set: %f, yaw_ref: %f, yaw_relative: %f, yaw_output: %d", yaw_angle_set_, imu_yaw_, yaw_relative, yaw_speed_set);
        RCLCPP_INFO(gimbal_controler_demo_node_->get_logger(), "=>> pitch_set: %f, pitch_ref: %f, pitch_relative: %f, pitch_output: %d", pitch_angle_set_, imu_pitch_, pitch_relative, pitch_speed_set);
    }
    else {
        gimbal.set__enable(false);
        gimbal.set__pitch_kp(0);
        gimbal.set__yaw_kp(0);
        gimbal.set__pitch_speed(0);
        gimbal.set__yaw_speed(0);
    }
    gimbal_output_publisher_->publish(gimbal);
    // RCLCPP_INFO(gimbal_controler_demo_node_->get_logger(), "=>> yaw_set: %f, yaw_ref: %f, pitch_set: %f, pitch_ref: %f", yaw_angle_set, imu_[0], pitch_angle_set, imu_[1]);
    
    skider_excutor::msg::ShooterOutput shooter;
    shooter.header.set__frame_id("Controler Demo Shooter Output");
    shooter.header.set__stamp(gimbal_controler_demo_node_->get_clock()->now());
    if (msg.buttons[2] == true) {
        shooter.set__enable(true);
        shooter.set__rotor_kp(10);
        shooter.set__ammol_kp(20);
        shooter.set__ammor_kp(20);
        shooter.set__rotor_speed(-msg.axes[4]*10000);
        shooter.set__ammol_speed(-5000);
        shooter.set__ammor_speed(5000);
    }
    else {
        shooter.set__enable(false);
        shooter.set__rotor_kp(0);
        shooter.set__ammol_kp(0);
        shooter.set__ammor_kp(0);
        shooter.set__rotor_speed(0);
        shooter.set__ammol_speed(0);
        shooter.set__ammor_speed(0);
    }
    shooter_output_publisher_->publish(shooter);
}


void GimbalControlerDemoNode::imu_msg_callback(const sensor_msgs::msg::Imu & msg)
{
    // Eigen::Quaterniond imu_quat(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
    // // Eigen::Quaterniond imu_quat;
    // // imu_quat.w() = msg.orientation.w;
    // // imu_quat.x() = msg.orientation.x;
    // // imu_quat.y() = msg.orientation.y;
    // // imu_quat.z() = msg.orientation.z;
    
    // imu_ = imu_quat.matrix().eulerAngles(2, 1, 0);

    tf2::Matrix3x3 mat(tf2::Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w));
    mat.getEulerYPR(imu_yaw_, imu_pitch_, imu_roll_);

    // RCLCPP_INFO(gimbal_controler_demo_node_->get_logger(), "=>> yaw: %f, pitch: %f, roll: %f", imu_yaw_, imu_pitch_, imu_roll_);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto gimbal_controler_demo_node = std::make_shared<GimbalControlerDemoNode>();
    rclcpp::spin(gimbal_controler_demo_node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}



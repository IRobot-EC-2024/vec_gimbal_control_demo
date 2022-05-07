/*
 * @Description: 
 * @Version: 1.0
 * @Autor: Qylann
 * @Date: 2022-04-05 15:37:07
 * @LastEditors: Qylann
 * @LastEditTime: 2022-04-05 16:20:23
 */
#include "gimbal_hw_node.hpp"

GimbalHWNode::GimbalHWNode(const rclcpp::NodeOptions & options)
{
    gimbal_hw_node_ = std::make_shared<rclcpp::Node>("gimbal_hw_node", options);
    RCLCPP_INFO(gimbal_hw_node_->get_logger(), "GimbalHWNode Begin");

    // ****** get parameters ******
    
    std::map<std::string, std::string> topic_name_params {
        {"imu_raw_publish_topic_name", ""},
        {"sbus_publish_topic_name", ""},
        {"gimbal_output_subscribe_topic_name", ""},
        {"shooter_output_subscribe_topic_name", ""},
    };

    std::map<std::string, int> transporter_params {
        {"gimbal_interface_usb_vid", 0},
        {"gimbal_interface_usb_pid", 0},
        {"gimbal_interface_usb_read_endpoint", 0},
        {"gimbal_interface_usb_write_endpoint", 0},
        {"gimbal_interface_usb_read_timeout", 0},
        {"gimbal_interface_usb_write_timeout", 0},
    };

    gimbal_hw_node_->declare_parameters("", topic_name_params);
    // topic name
    gimbal_hw_node_->get_parameter<std::string>("imu_raw_publish_topic_name", imu_raw_publish_topic_name_);
    gimbal_hw_node_->get_parameter<std::string>("sbus_publish_topic_name", sbus_publish_topic_name_);
    gimbal_hw_node_->get_parameter<std::string>("gimbal_output_subscribe_topic_name", gimbal_output_subscribe_topic_name_);
    gimbal_hw_node_->get_parameter<std::string>("shooter_output_subscribe_topic_name", shooter_output_subscribe_topic_name_);

    gimbal_hw_node_->declare_parameters("", transporter_params);
    // transporter parameter
    gimbal_hw_node_->get_parameter<int>("gimbal_interface_usb_vid", gimbal_interface_usb_vid_);
    gimbal_hw_node_->get_parameter<int>("gimbal_interface_usb_pid", gimbal_interface_usb_pid_);
    gimbal_hw_node_->get_parameter<int>("gimbal_interface_usb_read_endpoint", gimbal_interface_usb_read_endpoint_);
    gimbal_hw_node_->get_parameter<int>("gimbal_interface_usb_write_endpoint", gimbal_interface_usb_write_endpoint_);
    gimbal_hw_node_->get_parameter<int>("gimbal_interface_usb_read_timeout", gimbal_interface_usb_read_timeout_);
    gimbal_hw_node_->get_parameter<int>("gimbal_interface_usb_write_timeout", gimbal_interface_usb_write_timeout_);





    RCLCPP_INFO(gimbal_hw_node_->get_logger(), "Init imu_raw Publisher");
    imu_raw_publisher_ = gimbal_hw_node_->create_publisher<sensor_msgs::msg::Imu>(
        imu_raw_publish_topic_name_, 10);

    RCLCPP_INFO(gimbal_hw_node_->get_logger(), "Init sbus Publisher");
    sbus_publisher_ = gimbal_hw_node_->create_publisher<skider_excutor::msg::Sbus>(
        sbus_publish_topic_name_, 10);

    RCLCPP_INFO(gimbal_hw_node_->get_logger(), "Subscribe Gimbal Output");
    gimbal_output_subscription_ = gimbal_hw_node_->create_subscription<skider_excutor::msg::GimbalOutput>(
        gimbal_output_subscribe_topic_name_, 10, std::bind(&GimbalHWNode::gimbal_output_msg_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(gimbal_hw_node_->get_logger(), "Subscribe Shooter Output");
    shooter_output_subscription_ = gimbal_hw_node_->create_subscription<skider_excutor::msg::ShooterOutput>(
        shooter_output_subscribe_topic_name_, 10, std::bind(&GimbalHWNode::shooter_output_msg_callback, this, std::placeholders::_1));

    RCLCPP_INFO(gimbal_hw_node_->get_logger(), "Init Timer 1000Hz");
    timer_1000Hz_ = gimbal_hw_node_->create_wall_timer(1ms, std::bind(&GimbalHWNode::loop_1000Hz, this));


    // shooter 
    RCLCPP_INFO(gimbal_hw_node_->get_logger(), "Command Topic Timeout Callback");
    gimbal_command_offline_timer_ = gimbal_hw_node_->create_wall_timer(20ms, [this]() {
        gimbal_command_timeout_ = true;
    });
    shooter_command_offline_timer_ = gimbal_hw_node_->create_wall_timer(20ms, [this]() {
        shooter_command_timeout_ = true;
    });




    RCLCPP_INFO(gimbal_hw_node_->get_logger(), "Init Transporter");
    transporter_ = std::make_shared<transporter_sdk::UsbcdcTransporter>(
        gimbal_interface_usb_vid_, 
        gimbal_interface_usb_pid_, 
        gimbal_interface_usb_read_endpoint_, 
        gimbal_interface_usb_write_endpoint_, 
        gimbal_interface_usb_read_timeout_, 
        gimbal_interface_usb_write_timeout_
    );

    RCLCPP_INFO(gimbal_hw_node_->get_logger(), "Open Transporter");
    if (transporter_->open() == true) {
        RCLCPP_INFO(gimbal_hw_node_->get_logger(), "Success");
    }
    else {
        RCLCPP_INFO(gimbal_hw_node_->get_logger(), "FAILED!!!");
    }
    RCLCPP_INFO(gimbal_hw_node_->get_logger(), "Finish Init");
}


void GimbalHWNode::loop_1000Hz()
{
    transport_package::GimbalHWReceivePackage package;

    int read_size = transporter_->read((unsigned char *)&package, 32);
    RCLCPP_INFO(gimbal_hw_node_->get_logger(), "read size : %d", read_size);

    if (read_size == 32) {

        sensor_msgs::msg::Imu imu_raw;
        
        imu_raw.header.set__frame_id("imu_raw");
        imu_raw.header.set__stamp(gimbal_hw_node_->get_clock()->now());

        imu_raw.angular_velocity.set__x(package.imu.gyro_x / 32768.0 *2000.0 /180.0 *M_PI);
        imu_raw.angular_velocity.set__y(package.imu.gyro_y / 32768.0 *2000.0 /180.0 *M_PI);
        imu_raw.angular_velocity.set__z(package.imu.gyro_z / 32768.0 *2000.0 /180.0 *M_PI);
        imu_raw.linear_acceleration.set__x(package.imu.accl_x / 32768.0 *3.0 *9.7944);
        imu_raw.linear_acceleration.set__y(package.imu.accl_y / 32768.0 *3.0 *9.7944);
        imu_raw.linear_acceleration.set__z(package.imu.accl_z / 32768.0 *3.0 *9.7944);

        imu_raw_publisher_->publish(imu_raw);


        skider_excutor::msg::Sbus sbus;
        sbus.header.set__frame_id("joy_sbus_frame");
        sbus.header.set__stamp(gimbal_hw_node_->get_clock()->now());
        for (uint i = 0; i < 18; i++) {
            sbus.ch.push_back(package.sbus[i]);
        }
        
        sbus_publisher_->publish(sbus);
    }


    // command send
    transmit_package_.header._SOF_ = 0x55;
    transmit_package_.header.frame_type = 0x01;     // frame type: SCM
    transmit_package_.header.frame_size = 24;
    transmit_package_.header._EOF_ = 0xAA;

    transmit_package_.command._SOF_ = 0x66;
    transmit_package_.command._EOF_ = 0x88;
    if (gimbal_command_timeout_ == true) {
        transmit_package_.command.gimbal_state = 0x00;
    }
    if (shooter_command_timeout_ == true) {
        transmit_package_.command.shooter_state = 0x00;
    }
    
    transporter_->write((unsigned char *)&transmit_package_, sizeof(transport_package::GimbalHWTransmitPackage));


}

void GimbalHWNode::gimbal_output_msg_callback(const skider_excutor::msg::GimbalOutput & msg)
{
    gimbal_command_offline_timer_->reset();
    gimbal_command_timeout_ = false;

    if (msg.enable == true) {
        transmit_package_.command.gimbal_state = 0x01;
        transmit_package_.command.yaw_kp = msg.yaw_kp;
        transmit_package_.command.yaw_command = msg.yaw_speed;
        transmit_package_.command.pitch_kp = msg.pitch_kp;
        transmit_package_.command.pitch_command = msg.pitch_speed;
    }
    else {
        transmit_package_.command.gimbal_state = 0x00;
        transmit_package_.command.yaw_kp = 0;
        transmit_package_.command.yaw_command = 0;
        transmit_package_.command.pitch_kp = 0;
        transmit_package_.command.pitch_command = 0;
    }
}

void GimbalHWNode::shooter_output_msg_callback(const skider_excutor::msg::ShooterOutput & msg)
{
    shooter_command_offline_timer_->reset();
    shooter_command_timeout_ = false;

    if (msg.enable == true) {
        transmit_package_.command.shooter_state = 0x01;
        transmit_package_.command.rotor_kp = msg.rotor_kp;
        transmit_package_.command.rotor_command = msg.rotor_speed;
        transmit_package_.command.ammol_kp = msg.ammol_kp;
        transmit_package_.command.ammol_command = msg.ammol_speed;
        transmit_package_.command.ammor_kp = msg.ammor_kp;
        transmit_package_.command.ammor_command = msg.ammor_speed;
    }
    else {
        transmit_package_.command.shooter_state = 0x00;
        transmit_package_.command.rotor_kp = 0;
        transmit_package_.command.rotor_command = 0;
        transmit_package_.command.ammol_kp = 0;
        transmit_package_.command.ammol_command = 0;
        transmit_package_.command.ammor_kp = 0;
        transmit_package_.command.ammor_command = 0;
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto imu_sensor_node = std::make_shared<GimbalHWNode>();
    rclcpp::spin(imu_sensor_node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}


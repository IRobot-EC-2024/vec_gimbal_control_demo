/*
 * @Description: 
 * @Version: 1.0
 * @Autor: Qylann
 * @Date: 2022-04-05 15:19:09
 * @LastEditors: Qylann
 * @LastEditTime: 2022-04-05 16:07:50
 */
#pragma once

#include <memory>
#include <string>
#include <functional>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <skider_excutor/msg/sbus.hpp>
#include <skider_excutor/msg/gimbal_output.hpp>
#include <skider_excutor/msg/shooter_output.hpp>

#include <sensor_msgs/msg/imu.hpp>


#include "usbcdc_transporter.hpp"
#include "transport_package.h"

using namespace std::chrono_literals;

class GimbalHWNode
{
public:
    explicit GimbalHWNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() {
        return gimbal_hw_node_->get_node_base_interface();
    }
private:
    void loop_1000Hz();
    void gimbal_output_msg_callback(const skider_excutor::msg::GimbalOutput & msg);
    void shooter_output_msg_callback(const skider_excutor::msg::ShooterOutput & msg);


private:
    rclcpp::Node::SharedPtr gimbal_hw_node_;
    rclcpp::TimerBase::SharedPtr timer_1000Hz_;
    //
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_raw_publisher_;
    
    
    rclcpp::Publisher<skider_excutor::msg::Sbus>::SharedPtr sbus_publisher_;

    rclcpp::Subscription<skider_excutor::msg::GimbalOutput>::SharedPtr gimbal_output_subscription_;

    rclcpp::Subscription<skider_excutor::msg::ShooterOutput>::SharedPtr shooter_output_subscription_;

    std::shared_ptr<transporter_sdk::TransporterInterface> transporter_;

    transport_package::GimbalHWTransmitPackage   transmit_package_;

    rclcpp::TimerBase::SharedPtr gimbal_command_offline_timer_;
    rclcpp::TimerBase::SharedPtr shooter_command_offline_timer_;

private:
    // protive topic timeout
    bool gimbal_command_timeout_;
    bool shooter_command_timeout_;

private:
    // params
    std::string imu_raw_publish_topic_name_;
    std::string sbus_publish_topic_name_;
    std::string gimbal_output_subscribe_topic_name_;
    std::string shooter_output_subscribe_topic_name_;
    int gimbal_interface_usb_vid_;
    int gimbal_interface_usb_pid_;
    int gimbal_interface_usb_read_endpoint_;
    int gimbal_interface_usb_write_endpoint_;
    int gimbal_interface_usb_read_timeout_;
    int gimbal_interface_usb_write_timeout_;
    
};


/*
 * @Description: 
 * @Version: 1.0
 * @Autor: Qylann
 * @Date: 2022-03-30 19:48:35
 * @LastEditors: Qylann
 * @LastEditTime: 2022-04-05 12:42:33
 */
#pragma once

#include <memory>
#include <string>
#include <functional>
#include <chrono>

#include <iostream>

#include <Eigen/Eigen>
#include <tf2/LinearMath/Matrix3x3.h>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <skider_excutor/msg/gimbal_output.hpp>
#include <skider_excutor/msg/shooter_output.hpp>

using namespace std::chrono_literals;


class GimbalControlerDemoNode
{
public:
    explicit GimbalControlerDemoNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() {
        return gimbal_controler_demo_node_->get_node_base_interface();
    }

private:
    void joy_msg_callback(const sensor_msgs::msg::Joy & msg);
    void imu_msg_callback(const sensor_msgs::msg::Imu & msg);
    
private:
    rclcpp::Node::SharedPtr gimbal_controler_demo_node_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
    rclcpp::Publisher<skider_excutor::msg::GimbalOutput>::SharedPtr gimbal_output_publisher_;
    rclcpp::Publisher<skider_excutor::msg::ShooterOutput>::SharedPtr shooter_output_publisher_;


private:
    double imu_yaw_;
    double imu_pitch_;
    double imu_roll_;
    
    double yaw_angle_set_;
    double pitch_angle_set_;
};
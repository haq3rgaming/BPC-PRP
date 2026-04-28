#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "algorithms/structs.hpp"

namespace nodes {
    class IoNode : public rclcpp::Node {
    public:
        IoNode();
        ~IoNode() override = default;
    private:
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr led_pub_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr lidar_sub_;
        rclcpp::TimerBase::SharedPtr control_timer_;

        void controlLoop();

        void lidar_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
        Around lidar_around_;
    };
}
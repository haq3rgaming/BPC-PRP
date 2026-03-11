#pragma once

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include <std_msgs/msg/int8.hpp>

namespace nodes {
    class MotorNode : public rclcpp::Node {
    public:
        MotorNode();
        ~MotorNode() override = default;

        void spin() const;
        void on_error_line_bulish(const std_msgs::msg::Int8 msg);
    private:
        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr line_error_subscriber_;
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motor_command_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        float rightVector{0};
        float leftVector{0};
    };
}
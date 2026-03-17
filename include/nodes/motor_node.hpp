#pragma once

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>

namespace nodes {
    class MotorNode : public rclcpp::Node {
    public:
        MotorNode();
        ~MotorNode() override = default;

        void spin() const;
        void on_error_line(const std_msgs::msg::Float64 msg);
        void on_line_found(const std_msgs::msg::Bool msg);
    private:
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr line_error_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr line_found_subscriber_;
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motor_command_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        double rightSpeed{0};
        double leftSpeed{0};
        bool enabled{false};

        double integral = 0;
        double previous_error = 0;
    };
}
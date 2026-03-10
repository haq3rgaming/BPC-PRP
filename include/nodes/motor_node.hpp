#pragma once

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/u_int8_multi_array.hpp"

namespace nodes {
    class MotorNode : public rclcpp::Node {
    public:
        MotorNode();
        ~MotorNode() override = default;

        void spin() const;
    private:
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motor_command_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
    };
}
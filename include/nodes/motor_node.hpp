#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
// #include "algorithms/pid.hpp"

namespace nodes {
    class MotorNode : public rclcpp::Node {
    public:
        MotorNode();
    private:
        const double WHEEL_DIAMETER = 0.071;
        const double WHEEL_RADIUS = WHEEL_DIAMETER / 2.0;
        const double WHEEL_BASE = 0.127;
        const double MAX_SPEED_RPS = 20.0; // Max wheel speed in radians per second
        const double MAX_SPEED_MPS = MAX_SPEED_RPS * WHEEL_RADIUS;
        const float STOP = 127.0;

        double target_left_;
        double target_right_;

        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motor_pub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
        rclcpp::TimerBase::SharedPtr control_timer_;

        void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
        void controlLoop();
    };
}
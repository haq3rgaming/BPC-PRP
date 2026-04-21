#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace nodes {
    class IoNode : public rclcpp::Node {
    public:
        IoNode();
        ~IoNode() override = default;
        
        int get_button_pressed() const;
    private:
        int button_pressed_ = -1;
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr button_subscriber_; 
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr move_command_publisher_;
 
        void on_button_callback(const std_msgs::msg::UInt8::SharedPtr msg);
    };
}
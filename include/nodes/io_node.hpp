#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>

namespace nodes {
    class IoNode : public rclcpp::Node {
    public:
        IoNode();
        ~IoNode() override = default;
        
        int get_button_pressed() const;
    private:
        int button_pressed_ = -1;
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr button_subscriber_; 
 
        void on_button_callback(const std_msgs::msg::UInt8::SharedPtr msg);
    };
}
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace nodes {
    class CameraNode : public rclcpp::Node {
    public:
        CameraNode();
        ~CameraNode() override = default;
    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr line_error_publisher_;
        void on_image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    };
}
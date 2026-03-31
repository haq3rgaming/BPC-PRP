#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace nodes {
    class LidarNode : public rclcpp::Node {
    public:
        LidarNode();
        ~LidarNode() override = default;

    private:
        int lidar_data_ = -1;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_; 
 
        void on_lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    };
}
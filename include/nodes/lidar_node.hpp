#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <vector>

#include "algorithms/structs.hpp"

namespace nodes {
    class LidarNode : public rclcpp::Node {
        public:
        LidarNode();
        ~LidarNode() override = default;
        
        private:
        std::vector<float> lidar_data_;
        float angle_step;
        Around filter_data;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr filtered_lidar;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
 
        void filter();
        void on_lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    };
}
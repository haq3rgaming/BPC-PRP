#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "algorithms/enums.hpp"
#include "algorithms/structs.hpp"
#include "algorithms/queue.hpp"

namespace nodes {
    class FSMNode : public rclcpp::Node {
    public:
        FSMNode();
        ~FSMNode() override = default;
    private:
        const int TICKS_PER_REV = 576;

        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr aruco_sub_;
        rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr encoder_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr lidar_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr imu_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;

        rclcpp::TimerBase::SharedPtr control_timer_;
        volatile FSMState current_state_ = CALIBRATION;

        void aruco_callback(const std_msgs::msg::UInt8::SharedPtr msg);
        void encoder_callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg);
        void lidar_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
        void imu_callback(const std_msgs::msg::Float64::SharedPtr msg);
        void publish_velocity(double linear_x, double angular_z);
        void controlLoop();

        int number_of_walls();
        bool is_wall(float distance);

        Queue exit_queue_ {};
        Queue treasure_queue_ {};
        FSMNextIntersection convert_marker_to_intersection(ArucoMarkerID marker);
        uint32_t left_encoder_ticks_ = 0;
        uint32_t right_encoder_ticks_ = 0;
        Around lidar_around_;
        double current_angle_ = 0.0;
        double target_angle_ = 0.0;
    };
}

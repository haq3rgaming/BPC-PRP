#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "algorithms/pid.hpp"

namespace nodes {
    class MotorNode : public rclcpp::Node {
    public:
        MotorNode();
    private:
        // === CONSTANTS ===
        const double WHEEL_DIAMETER = 0.04; // 40 mm
        const double WHEEL_RADIUS = WHEEL_DIAMETER / 2.0;
        const double WHEEL_BASE = 0.15;

        const int TICKS_PER_REV = 576;

        const int STOP = 127;

        // === PID ===
        PID pid_left_;
        PID pid_right_;

        // === State ===
        double target_left_;
        double target_right_;

        double measured_left_;
        double measured_right_;

        int last_ticks_left_;
        int last_ticks_right_;

        rclcpp::Time last_time_;

        // === Distance control ===
        bool move_active_;
        double target_distance_;

        int start_ticks_left_;
        int start_ticks_right_;

        // === ROS ===
        rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr motor_pub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr move_sub_;
        rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr enc_sub_;
        rclcpp::TimerBase::SharedPtr control_timer_;

        // === Methods ===
        void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
        void encoderCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
        void controlLoop();

        void startMove(double distance_m, double speed_mps);
        void moveCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    };
}
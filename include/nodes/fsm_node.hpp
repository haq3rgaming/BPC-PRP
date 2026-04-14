#pragma once

#include "../../include/algorithms/enums.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>

namespace nodes {
    class FSMNode : public rclcpp::Node {
    public:
        FSMNode();
        ~FSMNode() override = default;
    private:
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr aruco_code_subscriber;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr command_finished_subscriber;
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr command_data_publisher;
        bool command_finished_ = false;
        FSMState current_state_ = START;
        FSMNextIntersection next_intersection_ = NONE;

        void on_aruco_code_callback(const std_msgs::msg::UInt8::SharedPtr msg);
        void on_command_finished_callback(const std_msgs::msg::Bool::SharedPtr msg);
        void execute_current_command();
        void update_state();
    };
}
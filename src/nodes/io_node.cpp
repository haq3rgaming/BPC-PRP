#include <nodes/io_node.hpp>

namespace nodes {
    IoNode::IoNode() : Node("io_node") {
        button_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/bpc_prp_robot/buttons",
            1,
            std::bind(&IoNode::on_button_callback, this, std::placeholders::_1)
        );
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/bpc_prp_robot/cmd_vel",
            10
        );
        move_command_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/bpc_prp_robot/move_command",
            10
        );
        RCLCPP_INFO(this->get_logger(), "IoNode started"); 
    }

    int IoNode::get_button_pressed() const {
        return button_pressed_;
    }

    void IoNode::on_button_callback(const std_msgs::msg::UInt8::SharedPtr msg){
        button_pressed_=msg->data;
        

        if (button_pressed_ == 0) { // forward
                auto vel_msg = geometry_msgs::msg::Twist();
                vel_msg.linear.x = 0.2;
                cmd_vel_publisher_->publish(vel_msg);
        } else if (button_pressed_ == 1) { // backward
                auto vel_msg = geometry_msgs::msg::Twist();
                vel_msg.linear.x = -0.2;
                cmd_vel_publisher_->publish(vel_msg);
        } else if (button_pressed_ == 2) { // forward 20cm
                auto move_msg = std_msgs::msg::Float32MultiArray();
                move_msg.data = {0.2f, 5.0f}; // distance, seconds
                move_command_publisher_->publish(move_msg);
        }

        RCLCPP_INFO(this->get_logger(), "Button pressed %d", msg->data);
        
    }
}
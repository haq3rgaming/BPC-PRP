#include "../../include/nodes/motor_node.hpp"

namespace nodes {
    MotorNode::MotorNode() : Node("motor_node") {
        motor_command_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
            "/bpc_prp_robot/set_motor_speeds",
            1
        );

        line_error_subscriber_ = this->create_subscription<std_msgs::msg::Int8>(
            "/bpc_prp_robot/line_error",
            1,
            std::bind(&MotorNode::on_error_line_bulish, this, std::placeholders::_1)
        )

        timer_ = create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&MotorNode::spin, this)
        );
    }

    void MotorNode::spin() const {
        std_msgs::msg::UInt8MultiArray motor_command_msg;
        
        motor_command_msg.data = {leftVector*128+127, leftVector*128+127};
        motor_command_publisher_->publish(motor_command_msg);
    }

    void MotorNode::on_error_line_bulish(const std_msgs::msg::Int8 msg){
        if(msg>0){
            leftVector = 1.0 - msg/127;
            rightVector = 1.0;
        }else{
            leftVector = 1.0;
            rightVector = 1.0 + msg/127;
        }
        RCLCPP_INFO(this->get_logger(), "L: %f R: %f", leftVector,rightVector);
    }
}
#include "../../include/nodes/motor_node.hpp"

#define stop_speed 127
#define base_speed 135
#define Kp 0.1
#define Ki 0.01
#define Kd 0.05

#define PID

namespace nodes {
    MotorNode::MotorNode() : Node("motor_node") {
        motor_command_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
            "/bpc_prp_robot/set_motor_speeds",
            1
        );

        line_error_subscriber_ = this->create_subscription<std_msgs::msg::Int8>(
            "/bpc_prp_robot/line_error",
            1,
            std::bind(&MotorNode::on_error_line, this, std::placeholders::_1)
        );

        line_found_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            "/bpc_prp_robot/line_found",
            1,
            std::bind(&MotorNode::on_line_found, this, std::placeholders::_1)
        );

        timer_ = create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&MotorNode::spin, this)
        );
    }

    void MotorNode::spin() const {
        #ifdef DEBUG
        RCLCPP_INFO(this->get_logger(), "Publishing motor speeds - Left: %d, Right: %d", leftSpeed, rightSpeed);
        #endif
        
        std_msgs::msg::UInt8MultiArray motor_command_msg;
        motor_command_msg.data = {enabled ? leftSpeed : stop_speed, enabled ? rightSpeed : stop_speed};
        motor_command_publisher_->publish(motor_command_msg);
    }

    void MotorNode::on_error_line(const std_msgs::msg::Int8 msg){
        int error = msg.data;

        #ifdef PID
        integral += error;
        double derivative = error - previous_error;
        double output = Kp * error + Ki * integral + Kd * derivative;
        previous_error = error;
        error = static_cast<int>(output);
        #else
        error = error * Kp;
        #endif

        leftSpeed = std::clamp((uint8_t)(base_speed + error), (uint8_t)base_speed, (uint8_t)255);
        rightSpeed = std::clamp((uint8_t)(base_speed - error), (uint8_t)base_speed, (uint8_t)255);
    }

    void MotorNode::on_line_found(const std_msgs::msg::Bool msg){
        enabled = msg.data;
    }
}
#include "../../include/nodes/motor_node.hpp"

#define stop_speed 127.0
#define base_forward_speed 150.0

#define Kp 0.1
#define Ki 0.01
#define Kd 0.01

#define PID

namespace nodes {
    MotorNode::MotorNode() : Node("motor_node") {
        motor_command_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
            "/bpc_prp_robot/set_motor_speeds",
            1
        );

        line_error_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
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
        RCLCPP_INFO(this->get_logger(), "Publishing motor speeds - Left: %f, Right: %f", leftSpeed, rightSpeed);
        #endif
        
        std_msgs::msg::UInt8MultiArray motor_command_msg;
        motor_command_msg.data = {enabled ? (uint8_t)leftSpeed : (uint8_t)stop_speed, enabled ? (uint8_t)rightSpeed : (uint8_t)stop_speed};
        motor_command_publisher_->publish(motor_command_msg);
    }

    void MotorNode::on_error_line(const std_msgs::msg::Float64 msg){
        double error = msg.data;

        #ifdef PID
        if (abs(error) < 0.01 || abs(integral) > 50) integral = 0; // Reset integral if error is very small or too large to prevent windup
        else integral += error; // Accumulate integral
        double derivative = error - previous_error;
        double output = Kp * error + Ki * integral + Kd * derivative;
        previous_error = error;
        #else
        double output = error * Kp;
        #endif

        leftSpeed = std::clamp(base_forward_speed + output, stop_speed, 255.0);
        rightSpeed = std::clamp(base_forward_speed - output, stop_speed, 255.0);
    }

    void MotorNode::on_line_found(const std_msgs::msg::Bool msg){
        enabled = msg.data;
    }
}
#include <nodes/motor_node.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>

#define Kp 5.0
#define Ki 0.0
#define Kd 0.0

namespace nodes {
    MotorNode::MotorNode() : Node("motor_node")
    {
        target_left_ = 0.0;
        target_right_ = 0.0;

        motor_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
            "/bpc_prp_robot/set_motor_speeds", 1
        );

        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/bpc_prp_robot/cmd_vel", 1,
            std::bind(&MotorNode::cmdCallback, this, std::placeholders::_1)
        );

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&MotorNode::controlLoop, this)
        );

        RCLCPP_INFO(this->get_logger(), "MotorNode initialized");
    }

    void MotorNode::controlLoop() {
        auto msg = std_msgs::msg::UInt8MultiArray();
        msg.data.resize(2);

        uint8_t speed_left = static_cast<uint8_t>(std::clamp(STOP + (target_left_ / MAX_SPEED_MPS) * STOP, 0.0, 255.0));
        uint8_t speed_right = static_cast<uint8_t>(std::clamp(STOP + (target_right_ / MAX_SPEED_MPS) * STOP, 0.0, 255.0));
        msg.data[0] = speed_left;
        msg.data[1] = speed_right;
        motor_pub_->publish(msg);
    }

    void MotorNode::cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        target_left_ = msg->linear.x - (msg->angular.z * WHEEL_BASE / 2.0);
        target_right_ = msg->linear.x + (msg->angular.z * WHEEL_BASE / 2.0);
    }
}

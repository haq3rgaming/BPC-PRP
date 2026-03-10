#include "../../include/nodes/motor_node.hpp"

namespace nodes {
    MotorNode::MotorNode() : Node("motor_node") {
        motor_command_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
            "/bpc_prp_robot/set_motor_speeds",
            1
        );

        timer_ = create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&MotorNode::spin, this)
        );
    }

    void MotorNode::spin() const {
        std_msgs::msg::UInt8MultiArray motor_command_msg;
        motor_command_msg.data = {150, 104};
        motor_command_publisher_->publish(motor_command_msg);
    }
}
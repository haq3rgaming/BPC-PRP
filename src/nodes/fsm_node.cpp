#include <nodes/fsm_node.hpp>
#include <algorithms/enums.hpp>

namespace nodes {
    FSMNode::FSMNode() : Node("fsm_node") {
        aruco_code_subscriber = this->create_subscription<std_msgs::msg::UInt8>(
            "/bpc_prp_robot/marker_data",
            1,
            std::bind(&FSMNode::on_aruco_code_callback, this, std::placeholders::_1)
        );
        command_finished_subscriber = this->create_subscription<std_msgs::msg::Bool>(
            "/bpc_prp_robot/command_finished",
            1,
            std::bind(&FSMNode::on_command_finished_callback, this, std::placeholders::_1)
        );
        command_data_publisher = this->create_publisher<std_msgs::msg::UInt8>("/bpc_prp_robot/command_data", 1);
        RCLCPP_INFO(this->get_logger(), "FSMNode initialized and subscribed to /bpc_prp_robot/command_finished");
    }

    void FSMNode::on_aruco_code_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
        ArucoMarkerID marker = static_cast<ArucoMarkerID>(msg->data);
        if (next_intersection_ == NONE) {
            switch (marker) {
                case EXIT_FW:
                    next_intersection_ = FW;
                    break;
                case EXIT_LEFT:
                    next_intersection_ = LEFT;
                    break;
                case EXIT_RIGHT:
                    next_intersection_ = RIGHT;
                    break;
                case TREASURE_FW:
                case TREASURE_LEFT:
                case TREASURE_RIGHT:
                    break; // Ignore treasure markers for now
                default:
                    RCLCPP_WARN(this->get_logger(), "Received unknown Aruco marker ID: %d", msg->data);
            }
            RCLCPP_INFO(this->get_logger(), "Updated next intersection to: %d", next_intersection_);
        }
    }

    void FSMNode::on_command_finished_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        command_finished_ = msg->data;
        if (command_finished_) {
            RCLCPP_INFO(this->get_logger(), "Received command finished signal. Executing next command.");
            execute_current_command();
        }
    }

    void FSMNode::execute_current_command() {
        command_finished_ = false; // Reset the flag for the next command
        std_msgs::msg::UInt8 command_data_msg;
        command_data_msg.data = static_cast<uint8_t>(current_state_);
        command_data_publisher->publish(command_data_msg);
        RCLCPP_INFO(this->get_logger(), "Published command data: %d", command_data_msg.data);
        while (!command_finished_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        } 
        update_state(); // Update the state after the command is finished
    }

    void FSMNode::update_state() {
        //TODO: Im lazy now
    }
}
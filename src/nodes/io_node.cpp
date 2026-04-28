#include <nodes/io_node.hpp>

#define IS_WALL_THRESHOLD 0.3
#define IS_WALL(distance) (distance < IS_WALL_THRESHOLD)
#define LED_ON 127

namespace nodes {
    IoNode::IoNode() : Node("io_node") {
        led_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
            "/bpc_prp_robot/rgb_leds", 1
        );

        lidar_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/bpc_prp_robot/filtered_lidar", 1,
            std::bind(&IoNode::lidar_callback, this, std::placeholders::_1)
        );

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5),
            std::bind(&IoNode::controlLoop, this)
        );

        RCLCPP_INFO(this->get_logger(), "IoNode initialized"); 
    }

    void IoNode::controlLoop() {
        std_msgs::msg::UInt8MultiArray led_msg;
        led_msg.data = {0,0,0,0,0,0,0,0,0,0,0,0};
        if (IS_WALL(lidar_around_.back)) {
            led_msg.data[0] = LED_ON;
            led_msg.data[1] = LED_ON;
            led_msg.data[2] = LED_ON;
        }
        if (IS_WALL(lidar_around_.front)) {
            led_msg.data[3] = LED_ON;
            led_msg.data[4] = LED_ON;
            led_msg.data[5] = LED_ON;
        }
        if (IS_WALL(lidar_around_.left)) {
            led_msg.data[6] = LED_ON;
            led_msg.data[7] = LED_ON;
            led_msg.data[8] = LED_ON;
        }
        if (IS_WALL(lidar_around_.right)) {
            led_msg.data[9] = LED_ON;
            led_msg.data[10] = LED_ON;
            led_msg.data[11] = LED_ON;
        }
        led_pub_->publish(led_msg);
    }

    void IoNode::lidar_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 4) {
            RCLCPP_WARN(this->get_logger(), "Received lidar message with insufficient data");
            return;
        }
        lidar_around_.front = msg->data[0];
        lidar_around_.back = msg->data[1];
        lidar_around_.left = msg->data[2];
        lidar_around_.right = msg->data[3];
    }
}
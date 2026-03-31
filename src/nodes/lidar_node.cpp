#include "../../include/nodes/lidar_node.hpp"

namespace nodes {
    LidarNode::LidarNode() : Node("lidar_node") {
        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/bpc_prp_robot/lidar",
            1,
            std::bind(&LidarNode::on_lidar_callback, this, std::placeholders::_1)
        );
    }

    void LidarNode::on_lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){

        RCLCPP_INFO(this->get_logger(), "Lidar publish");
    }
}
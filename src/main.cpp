#include <rclcpp/rclcpp.hpp>
#include "../include/nodes/io_node.hpp"
#include "../include/nodes/motor_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // Create an executor (for handling multiple nodes)
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    // Create nodes
    auto io_node = std::make_shared<nodes::IoNode>();
    auto motor_node = std::make_shared<nodes::MotorNode>();

    // Add nodes to the executor
    executor->add_node(io_node);
    executor->add_node(motor_node);

    // Run the executor (handles callbacks for both nodes)
    executor->spin();

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
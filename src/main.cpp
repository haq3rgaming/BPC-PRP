#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "../include/nodes/io_node.hpp"
#include "../include/nodes/motor_node.hpp"
#include "../include/nodes/camera_node.hpp"
#include "../include/nodes/lidar_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // Enable intra-process communication for better performance
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    
    // Create nodes
    auto motor_node = std::make_shared<nodes::MotorNode>();
    //auto camera_node = std::make_shared<nodes::CameraNode>();
    auto lidar_node = std::make_shared<nodes::LidarNode>();

    // Create an executor (for handling multiple nodes)
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    
    // Add nodes to the executor
    executor->add_node(motor_node);
    //executor->add_node(camera_node);
    executor->add_node(lidar_node);

    // Run the executor (handles callbacks for both nodes)
    executor->spin();

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
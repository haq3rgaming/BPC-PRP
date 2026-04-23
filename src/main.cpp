#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "nodes/io_node.hpp"
#include "nodes/motor_node.hpp"
#include "nodes/aruco_node.hpp"
#include "nodes/lidar_node.hpp"
#include "nodes/imu_node.hpp"
#include "nodes/fsm_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // Enable intra-process communication for better performance
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    
    // Create nodes
    auto io_node = std::make_shared<nodes::IoNode>();
    auto motor_node = std::make_shared<nodes::MotorNode>();
    //auto aruco_node = std::make_shared<nodes::ArucoNode>();
    auto lidar_node = std::make_shared<nodes::LidarNode>();
    auto imu_node = std::make_shared<nodes::ImuNode>();
    auto fsm_node = std::make_shared<nodes::FSMNode>();

    // Create an executor (for handling multiple nodes)
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    
    // Add nodes to the executor
    executor->add_node(io_node);
    executor->add_node(motor_node);
    //executor->add_node(aruco_node);
    executor->add_node(lidar_node);
    executor->add_node(imu_node);
    executor->add_node(fsm_node);

    // Run the executor (handles callbacks for both nodes)
    executor->spin();

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
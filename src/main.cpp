#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <algorithm>

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
    auto io_node = std::make_shared<nodes::IoNode>(options);
    auto motor_node = std::make_shared<nodes::MotorNode>(options);
    auto aruco_node = std::make_shared<nodes::ArucoNode>(options);
    auto lidar_node = std::make_shared<nodes::LidarNode>(options);
    auto imu_node = std::make_shared<nodes::ImuNode>(options);
    auto fsm_node = std::make_shared<nodes::FSMNode>(options);

    auto executor_options = rclcpp::ExecutorOptions();
    // Use half of the available hardware threads, at least 1
    unsigned int hw_threads = std::thread::hardware_concurrency();
    unsigned int num_threads = std::max<unsigned int>(1u, hw_threads);
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();//(executor_options, num_threads, false);
    
    // Add nodes to the executor
    executor->add_node(io_node);
    executor->add_node(motor_node);
    executor->add_node(aruco_node);
    executor->add_node(lidar_node);
    executor->add_node(imu_node);
    executor->add_node(fsm_node);

    // Run the executor (handles callbacks for both nodes)
    executor->spin();

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
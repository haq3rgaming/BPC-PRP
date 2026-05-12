#include <nodes/aruco_node.hpp>

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include "algorithms/perf.hpp"


// #define DEBUG

namespace nodes {
    ArucoNode::ArucoNode(rclcpp::NodeOptions options) : Node("aruco_node", options) {
        image_subscriber_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/bpc_prp_robot/camera/compressed",
            1,
            std::bind(&ArucoNode::on_image_callback, this, std::placeholders::_1)
        );
        marker_data_publisher_ = this->create_publisher<std_msgs::msg::UInt8>("/bpc_prp_robot/marker_data", 1);
        cv::setNumThreads(1);

        RCLCPP_INFO(this->get_logger(), "ArucoNode initialized");
    }

    void ArucoNode::flip_image(cv::Mat& frame) {
        cv::flip(frame, frame, -1);
    }

    void ArucoNode::process_camera_frame(const cv::Mat& frame) {
        if (frame.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Aruco callback received an empty frame.");
            return;
        }

        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;

        cv::aruco::detectMarkers(frame, _dictionary, marker_corners, marker_ids);

        if (marker_ids.empty()) {
            #ifdef DEBUG
            RCLCPP_WARN(this->get_logger(), "No Aruco markers detected in the current frame.");
            #endif
            return;
        }

        publish_marker_data(marker_ids[0]);
    }

    void ArucoNode::thread_process_camera_frame(const cv::Mat& frame) {
        try
        {
            TimePerformance aruco_perf("aruco");
            if (frame_.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Aruco callback decoded an empty frame.");
                return;
            }

            flip_image(frame_);
            process_camera_frame(frame_);
        }
        catch (const cv::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Standard exception: %s", e.what());
        }
        catch (...)
        {
            RCLCPP_ERROR(this->get_logger(), "Unknown exception occurred in ArucoNode.");
        }
    }

    void ArucoNode::publish_marker_data(int data) {
        auto marker_data_msg = std_msgs::msg::UInt8();
        marker_data_msg.data = data;
        marker_data_publisher_->publish(marker_data_msg);

        #ifdef DEBUG
        RCLCPP_INFO(this->get_logger(), "Marker data published: %d", data);
        #endif
    }

    void ArucoNode::on_image_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
        if (!msg || msg->data.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Aruco callback received an empty compressed image message.");
            return;
        }
        if (processing_thread_.joinable()) {
            RCLCPP_WARN(this->get_logger(), "Previous processing thread is still running. Skipping this frame.");
            return;
        }
        frame_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
        processing_thread_ = std::thread(&ArucoNode::thread_process_camera_frame, this, frame_);
        processing_thread_.detach();
    }
}
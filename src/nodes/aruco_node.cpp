#include "../../include/nodes/aruco_node.hpp"

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>


#define HEADLESS
// #define DEBUG

namespace nodes {
    ArucoNode::ArucoNode() : Node("aruco_node") {
        image_subscriber_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/bpc_prp_robot/camera/compressed",
            1,
            std::bind(&ArucoNode::on_image_callback, this, std::placeholders::_1)
        );
        marker_data_publisher_ = this->create_publisher<std_msgs::msg::UInt8>("/bpc_prp_robot/marker_data", 1);
        marker_found_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/bpc_prp_robot/marker_found", 1);
        RCLCPP_INFO(this->get_logger(), "ArucoNode initialized and subscribed to /bpc_prp_robot/camera/compressed");
    }

    void ArucoNode::flip_image(cv::Mat& frame) {
        cv::flip(frame, frame, -1);
    }

    void ArucoNode::process_camera_frame(const cv::Mat& frame) {
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;

        cv::aruco::detectMarkers(frame, _dictionary, marker_corners, marker_ids);

        if (marker_ids.empty()) {
            publish_detection_warn("No Aruco markers detected in the current frame.");
            return;
        }

        detected_markers_.clear();
        for (size_t i = 0; i < marker_ids.size(); ++i) {
            detected_markers_.push_back({marker_corners[i], marker_ids[i]});
        }

        publish_marker_data(marker_ids[0]);
        publish_marker_found(true);
    }

    void ArucoNode::draw_debug_info(cv::Mat& frame, const std::vector<ArucoMarker>& markers) {
        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<int> ids;
        for (const auto& marker : markers) {
            corners.push_back(marker.corners);
            ids.push_back(marker.id);
        }
        cv::aruco::drawDetectedMarkers(frame, corners, ids);
    }

    void ArucoNode::publish_marker_data(int data) {
        auto marker_data_msg = std_msgs::msg::UInt8();
        marker_data_msg.data = data;
        marker_data_publisher_->publish(marker_data_msg);

        #ifdef DEBUG
        RCLCPP_INFO(this->get_logger(), "Marker data published: %d", data);
        #endif
    }

    void ArucoNode::publish_marker_found(bool found) {
        auto marker_found_msg = std_msgs::msg::Bool();
        marker_found_msg.data = found;
        marker_found_publisher_->publish(marker_found_msg);

        #ifdef DEBUG
        RCLCPP_INFO(this->get_logger(), "Marker found published: %s", found ? "true" : "false");
        #endif
    }

    void ArucoNode::publish_detection_warn(const std::string& msg) {
        #ifdef DEBUG
        RCLCPP_WARN(this->get_logger(), msg.c_str());
        #endif

        publish_marker_data(0);
        publish_marker_found(false);
    }

    void ArucoNode::on_image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        try
        {
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            flip_image(frame);
            process_camera_frame(frame);
            
            #ifndef HEADLESS
            draw_debug_info(frame, detected_markers_);
            cv::imshow("Camera Feed", frame);
            cv::waitKey(1);
            #endif

        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
}
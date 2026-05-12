#include <nodes/aruco_node.hpp>

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

    void ArucoNode::on_image_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
        if (!msg || msg->data.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Aruco callback received an empty compressed image message.");
            return;
        }

        try
        {
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            if (frame.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Aruco callback decoded an empty frame.");
                return;
            }

            flip_image(frame);
            process_camera_frame(frame);
            
            #ifndef HEADLESS
            draw_debug_info(frame, detected_markers_);
            cv::imshow("Camera Feed", frame);
            cv::waitKey(1);
            #endif
        }
        catch (const cv::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
}
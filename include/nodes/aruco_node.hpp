#pragma once

#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>

namespace nodes {
    class ArucoNode : public rclcpp::Node {
    public:
        ArucoNode(rclcpp::NodeOptions options = rclcpp::NodeOptions());
        ~ArucoNode() override = default;
    private:
        struct ArucoMarker {
            std::vector<cv::Point2f> corners;
            int id;
        };
        cv::Ptr<cv::aruco::Dictionary> _dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        std::vector<ArucoMarker> detected_markers_;
        cv::Mat frame_;
        std::thread processing_thread_;


        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_subscriber_;
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr marker_data_publisher_;
        void on_image_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg);

        void flip_image(cv::Mat& frame);
        void process_camera_frame(const cv::Mat& frame);
        void thread_process_camera_frame(const cv::Mat& frame);
        
        void publish_marker_data(int data);
    };
}
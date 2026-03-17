#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <opencv2/core.hpp>

namespace nodes {
    class CameraNode : public rclcpp::Node {
    public:
        CameraNode();
        ~CameraNode() override = default;
    private:
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_subscriber_;
        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr line_error_publisher_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr line_found_publisher_;
        void on_image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

        cv::Mat preprocess_image(const cv::Mat& frame);
        cv::Mat create_mask(const cv::Mat& hsv);
        void apply_morphology(cv::Mat& mask);
        cv::Mat get_roi(const cv::Mat& mask, int roi_start_y);
        std::vector<std::vector<cv::Point>> find_contours(const cv::Mat& roi);
        int find_largest_contour(const std::vector<std::vector<cv::Point>>& contours);

        void publish_line_error(int error);
        void publish_line_found(bool found);
        void publish_detection_warn(const std::string& msg);

        void draw_debug_info(cv::Mat& frame, const std::vector<cv::Point>& contour, int roi_start_y, cv::Point center);
    };
}
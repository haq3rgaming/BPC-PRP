#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
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
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr line_error_publisher_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr line_found_publisher_;
        void on_image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

        float roi_start_x_;
        float roi_start_y_;
        float roi_end_x_;
        float roi_end_y_;

        void flip_image(cv::Mat& frame);
        void process_camera_frame(cv::Mat& frame);
        
        cv::Mat convert_to_hsv(const cv::Mat& frame);
        cv::Mat create_mask(const cv::Mat& hsv);
        void apply_morphology(cv::Mat& mask);
        cv::Rect create_roi_rect(const cv::Mat& mask, float start_x, float start_y, float end_x, float end_y);
        std::vector<std::vector<cv::Point>> find_contours(const cv::Mat& roi);
        int find_largest_contour(const std::vector<std::vector<cv::Point>>& contours);

        void publish_line_error(double error);
        void publish_line_found(bool found);
        void publish_detection_warn(const std::string& msg);

        void draw_debug_info(cv::Mat& frame, const std::vector<cv::Point>& contour, cv::Rect roiRect, cv::Point center);
    };
}
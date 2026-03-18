#include "../../include/nodes/camera_node.hpp"

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>


// #define HEADLESS
// #define DEBUG

#define AREA_THRESHOLD 500
#define ERROR_NORMALIZATION_FACTOR 0.3 // toto by sa malo nastavit dynamicky podla ROI, aby hodnoty boli +-100

#define ROI_START_X 0.1
#define ROI_START_Y 0.5
#define ROI_END_X 0.9
#define ROI_END_Y 1.0

#define ROI_RECT_COLOR cv::Scalar(0,255,0)
#define CONTOUR_COLOR cv::Scalar(0,0,255)
#define CENTER_COLOR cv::Scalar(255,0,0)
#define CENTER_LINE_COLOR cv::Scalar(255,0,255)

namespace nodes {
    CameraNode::CameraNode() : Node("camera_node") {
        image_subscriber_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/bpc_prp_robot/camera/compressed",
            1,
            std::bind(&CameraNode::on_image_callback, this, std::placeholders::_1)
        );
        line_error_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/bpc_prp_robot/line_error", 1);
        line_found_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/bpc_prp_robot/line_found", 1);
        RCLCPP_INFO(this->get_logger(), "CameraNode initialized and subscribed to /bpc_prp_robot/camera/compressed");
    
        roi_start_x_ = ROI_START_X;
        roi_start_y_ = ROI_START_Y;
        roi_end_x_ = ROI_END_X;
        roi_end_y_ = ROI_END_Y;
    }

    void CameraNode::flip_image(cv::Mat& frame) {
        cv::flip(frame, frame, -1);
    }

    cv::Mat CameraNode::convert_to_hsv(const cv::Mat& frame) {
        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        return hsv;
    }

    cv::Mat CameraNode::create_mask(const cv::Mat& hsv) {
        cv::Mat mask;
        cv::inRange(hsv, cv::Scalar(0,0,0), cv::Scalar(180,255,60), mask);
        return mask;
    }

    void CameraNode::apply_morphology(cv::Mat& mask) {
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    }

    cv::Rect CameraNode::create_roi_rect(const cv::Mat& mask, float start_x, float start_y, float end_x, float end_y) {
        return cv::Rect(mask.cols * start_x, mask.rows * start_y, mask.cols * end_x, mask.rows * end_y);
    }

    std::vector<std::vector<cv::Point>> CameraNode::find_contours(const cv::Mat& roi) {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(roi, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        return contours;
    }

    int CameraNode::find_largest_contour(const std::vector<std::vector<cv::Point>>& contours) {
        double max_area = 0;
        int largest_index = -1;
        for (size_t i = 0; i < contours.size(); i++) {
            double area = cv::contourArea(contours[i]);
            if (area > max_area) {
                max_area = area;
                largest_index = i;
            }
        }
        return largest_index;
    }

    void CameraNode::publish_line_error(double error) {
        auto line_error_msg = std_msgs::msg::Float64();
        line_error_msg.data = error * ERROR_NORMALIZATION_FACTOR;
        line_error_publisher_->publish(line_error_msg);

        #ifdef DEBUG
        RCLCPP_INFO(this->get_logger(), "Line error published: %f", error);
        #endif
    }

    void CameraNode::publish_line_found(bool found) {
        auto line_found_msg = std_msgs::msg::Bool();
        line_found_msg.data = found;
        line_found_publisher_->publish(line_found_msg);

        #ifdef DEBUG
        RCLCPP_INFO(this->get_logger(), "Line found published: %s", found ? "true" : "false");
        #endif
    }

    void CameraNode::publish_detection_warn(const std::string& msg) {
        #ifdef DEBUG
        RCLCPP_WARN(this->get_logger(), msg.c_str());
        #endif

        publish_line_error(0);
        publish_line_found(false);
    }

    void CameraNode::process_camera_frame(cv::Mat& frame) {
        cv::Mat hsv = convert_to_hsv(frame);
        cv::Mat mask = create_mask(hsv);
        apply_morphology(mask);
        
        cv::Rect roiRect = create_roi_rect(mask, roi_start_x_, roi_start_y_, roi_end_x_, roi_end_y_);
        std::vector<std::vector<cv::Point>> contours = find_contours(mask(roiRect));
        if (contours.empty()) {
            publish_detection_warn("Line not detected");
            return;
        }

        int largest_index = find_largest_contour(contours);
        if (largest_index < 0)
        {
            publish_detection_warn("No valid contours found");
            return;
        }

        auto contour = contours[largest_index];
        cv::Moments M = cv::moments(contour);
        if (M.m00 <= AREA_THRESHOLD)
        {
            publish_detection_warn("No contour meets area threshold");
            return;
        }

        #ifdef DEBUG
        RCLCPP_INFO(this->get_logger(), "Largest contour area: %f", M.m00);
        #endif

        cv::Point center = cv::Point(M.m10 / M.m00 + roiRect.x, M.m01 / M.m00 + roiRect.y);

        #ifndef HEADLESS
        draw_debug_info(frame, contour, roiRect, center);
        #endif

        double error = (center.x - frame.cols/2);
        publish_line_error(error);
        publish_line_found(true);
    }

    void CameraNode::draw_debug_info(cv::Mat& frame, const std::vector<cv::Point>& contour, cv::Rect roiRect, cv::Point center) {
        // Move contour points back to original frame coordinates from ROI
        std::vector<cv::Point> contour_shifted;
        for (const auto &pt : contour)
            contour_shifted.push_back(cv::Point(pt.x + roiRect.x, pt.y + roiRect.y));

        // Draw the ROI rectangle
        cv::rectangle(frame, roiRect, ROI_RECT_COLOR, 2);

        // Draw the shifted contour on the full frame
        cv::drawContours(frame, std::vector<std::vector<cv::Point>>{contour_shifted}, 0, CONTOUR_COLOR, 3);
        cv::circle(frame, center, 8, CENTER_COLOR, -1);

        // Draw image center line
        cv::line(frame, cv::Point(frame.cols/2,0), cv::Point(frame.cols/2,frame.rows), CENTER_LINE_COLOR, 2);
    }

    void CameraNode::on_image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        try
        {
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            flip_image(frame);
            process_camera_frame(frame);
            
            #ifndef HEADLESS
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
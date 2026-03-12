#include "../../include/nodes/camera_node.hpp"

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>


// #define HEADLESS
// #define DEBUG
// #define PID

namespace nodes {
    CameraNode::CameraNode() : Node("camera_node") {
        image_subscriber_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/bpc_prp_robot/camera/compressed",
            1,
            std::bind(&CameraNode::on_image_callback, this, std::placeholders::_1)
        );
        line_error_publisher_ = this->create_publisher<std_msgs::msg::Int8>("/bpc_prp_robot/line_error", 1);
        line_found_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/bpc_prp_robot/line_found", 1);
        RCLCPP_INFO(this->get_logger(), "CameraNode initialized and subscribed to /bpc_prp_robot/camera/compressed");
    }

    cv::Mat CameraNode::preprocess_image(const cv::Mat& frame) {
        cv::flip(frame, frame, -1);
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

    cv::Mat CameraNode::get_roi(const cv::Mat& mask, int roi_start_y) {
        return mask(cv::Rect(0, roi_start_y, mask.cols, mask.rows - roi_start_y));
    } // TODO: pozriet ci to berie spodnu cast, lebo mam pocit ze to berie hornu cast obrazka

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

    int CameraNode::error_processor(int error) {
        #ifdef PID
        // --- PID Controller
        static double integral = 0;
        static double previous_error = 0;
        double Kp = 0.1; // Proportional gain
        double Ki = 0.01; // Integral gain
        double Kd = 0.05; // Derivative gain

        integral += error;
        double derivative = error - previous_error;
        double output = Kp * error + Ki * integral + Kd * derivative;
        previous_error = error;
        error = static_cast<int>(output);
        #endif
        return error;
    }

    void CameraNode::publish_line_error(int error) {
        auto line_error_msg = std_msgs::msg::Int8();
        line_error_msg.data = error;
        line_error_publisher_->publish(line_error_msg);

        #ifdef DEBUG
        RCLCPP_INFO(this->get_logger(), "Line error published: %d", error);
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

    void CameraNode::publish_detection_warn(std::string msg) {
        #ifdef DEBUG
        RCLCPP_WARN(this->get_logger(), msg);
        #endif

        publish_line_error(0);
        publish_line_found(false);
    }

    void CameraNode::draw_debug_info(cv::Mat& frame, const std::vector<cv::Point>& contour, int roi_start_y, cv::Point center) {
        // Move contour points back to original frame coordinates from ROI
        std::vector<cv::Point> contour_shifted;
        for (const auto &pt : contour)
        {
            contour_shifted.push_back(cv::Point(pt.x, pt.y + roi_start_y));
        }

        // Draw the shifted contour on the full frame
        cv::drawContours(frame, std::vector<std::vector<cv::Point>>{contour_shifted}, 0, cv::Scalar(0,255,0), 3);
        cv::circle(frame, center, 8, cv::Scalar(0,0,255), -1);

        // Draw image center line
        cv::line(frame, cv::Point(frame.cols/2,0), cv::Point(frame.cols/2,frame.rows), cv::Scalar(255,0,0), 2);
    }

    void CameraNode::on_image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        try
        {
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            int height = frame.rows;
            int width = frame.cols;

            // --- Rotate 180° (flip both axes) and convert to HSV
            cv::Mat hsv = preprocess_image(frame);

            // --- Black color mask
            cv::Mat mask = create_mask(hsv);

            // --- Morphological filtering to remove noise
            apply_morphology(mask);

            // --- Focus on bottom 40% of the image
            int roi_start_y = static_cast<int>(height * 0.6); //TODO: pozriet rovnako ako pri funkcii get_roi
            cv::Mat roi = get_roi(mask, roi_start_y); //TODO: mozno prerobit na definiciu roi ako x,y,w,h a tak s tym robit dalej, nie len ako jeden value

            // --- Find contours
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(roi, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            
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
            if (M.m00 <= 0)
            {
                publish_detection_warn("Contour has zero area");
                return;
            }

            int cx = M.m10 / M.m00;
            int cy = M.m01 / M.m00 + roi_start_y; // adjust for ROI

            #ifndef HEADLESS
            draw_debug_info(frame, contour, roi_start_y, cv::Point(cx, cy));
            #endif

            int error = std::clamp(error_processor(cx - width/2), -128, 127); // Ensure error fits in int8
            publish_line_error(error);
            publish_line_found(true);

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
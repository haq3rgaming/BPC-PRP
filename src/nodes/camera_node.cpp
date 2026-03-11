#include "../../include/nodes/camera_node.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

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

    void CameraNode::on_image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        try
        {
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

            // --- Rotate 180° (flip both axes)
            cv::flip(frame, frame, -1);

            int height = frame.rows;
            int width = frame.cols;

            // --- Convert to HSV
            cv::Mat hsv;
            cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

            // --- Black color mask
            cv::Mat mask;
            cv::inRange(hsv, cv::Scalar(0,0,0), cv::Scalar(180,255,60), mask);

            // --- Morphological filtering to remove noise
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));
            cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
            cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

            // --- Focus on bottom 40% of the image
            int roi_start_y = static_cast<int>(height * 0.6);
            cv::Rect roi_rect(0, roi_start_y, width, height - roi_start_y);
            cv::Mat roi = mask(roi_rect);

            // --- Find contours
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(roi, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            if (!contours.empty())
            {
                // Find largest contour
                double max_area = 0;
                int largest_index = -1;
                for (size_t i = 0; i < contours.size(); i++)
                {
                    double area = cv::contourArea(contours[i]);
                    if (area > max_area)
                    {
                        max_area = area;
                        largest_index = i;
                    }
                }

                if (largest_index >= 0)
                {
                    cv::Moments M = cv::moments(contours[largest_index]);
                    if (M.m00 > 0)
                    {
                        int cx = M.m10 / M.m00;
                        int cy = M.m01 / M.m00 + height*0.6; // adjust for ROI

                        auto line_found_msg = std_msgs::msg::Bool();
                        line_found_msg.data = true;
                        line_found_publisher_->publish(line_found_msg);

                        #ifndef HEADLESS
                        // Move contour points back to original frame coordinates from ROI
                        std::vector<cv::Point> contour_shifted;
                        for (const auto &pt : contours[largest_index])
                        {
                            contour_shifted.push_back(cv::Point(pt.x, pt.y + roi_start_y));
                        }

                        // Draw the shifted contour on the full frame
                        cv::drawContours(frame, std::vector<std::vector<cv::Point>>{contour_shifted}, 0, cv::Scalar(0,255,0), 3);
                        cv::circle(frame, cv::Point(cx, cy), 8, cv::Scalar(0,0,255), -1);

                        // Draw image center line
                        cv::line(frame, cv::Point(width/2,0), cv::Point(width/2,height), cv::Scalar(255,0,0), 2);
                        #endif
                        
                        #ifndef PID
                        int error = cx - width/2;
                        #else
                        // --- PID Controller
                        static double integral = 0;
                        static double previous_error = 0;
                        double Kp = 0.1; // Proportional gain
                        double Ki = 0.01; // Integral gain
                        double Kd = 0.05; // Derivative gain

                        int error = cx - width/2;
                        integral += error;
                        double derivative = error - previous_error;
                        double output = Kp * error + Ki * integral + Kd * derivative;
                        previous_error = error;
                        error = static_cast<int>(output);
                        #endif

                        auto msg = std_msgs::msg::Int8();
                        msg.data = std::clamp(error, -128, 127); // Ensure error fits in int8
                        line_error_publisher_->publish(msg);
                        
                        #ifdef DEBUG
                        RCLCPP_INFO(this->get_logger(), "Line error published: %d", error);
                        #endif
                    }
                }
            }
            else
            {
                #ifdef DEBUG
                RCLCPP_WARN(this->get_logger(), "Line not detected");
                #endif

                auto line_found_msg = std_msgs::msg::Bool();
                line_found_msg.data = false;
                line_found_publisher_->publish(line_found_msg);
            }

            #ifndef HEADLESS
            cv::imshow("Camera Feed Rotated 180", frame);
            cv::waitKey(1);
            #endif

        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
}
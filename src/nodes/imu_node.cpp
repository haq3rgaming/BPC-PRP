#include "nodes/imu_node.hpp"

#include <cmath>

namespace nodes {

    ImuNode::ImuNode() : Node("imu_node") {
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/bpc_prp_robot/imu",
            1,
            std::bind(&ImuNode::on_imu_msg, this, std::placeholders::_1)
        );

        angle_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/bpc_prp_robot/angle", 1);
        mode = ImuNodeMode::CALIBRATE;
        theta_ = 0.0;
        gyro_offset_ = 0.0;
        has_last_timestamp_ = false;

        RCLCPP_INFO(this->get_logger(), "ImuNode initialized");
    }

    void ImuNode::setMode(ImuNodeMode mode) {
        this->mode = mode;
    }

    ImuNodeMode ImuNode::getMode() {
        return this->mode;
    }


    void ImuNode::reset_imu() {
        gyro_calibration_samples_.clear();
        theta_ = 0.0;
        has_last_timestamp_ = false;
    }

    void ImuNode::calibrate() {
        // Simple average for gyro bias
        double sum = 0.0;
        for (float sample : gyro_calibration_samples_) {
            sum += sample;
        }
        double bias = sum / gyro_calibration_samples_.size();
        this->gyro_offset_ = bias;
    }

    void ImuNode::on_imu_msg(const sensor_msgs::msg::Imu::SharedPtr msg) {
        if (mode == ImuNodeMode::CALIBRATE) {
            // Collect gyro calibration samples
            gyro_calibration_samples_.push_back(msg->angular_velocity.z);
            if (gyro_calibration_samples_.size() >= 1000) {
                calibrate();
                setMode(ImuNodeMode::INTEGRATE);
                last_timestamp_ = rclcpp::Time(msg->header.stamp);
                has_last_timestamp_ = true;
            }
        } else if (mode == ImuNodeMode::INTEGRATE) {
            const rclcpp::Time current_timestamp(msg->header.stamp);

            if (!has_last_timestamp_) {
                last_timestamp_ = current_timestamp;
                has_last_timestamp_ = true;
                return;
            }

            const double dt = (current_timestamp - last_timestamp_).seconds();
            last_timestamp_ = current_timestamp;

            if (dt <= 0.0 || dt > 0.5) {
                RCLCPP_WARN(this->get_logger(), "Skipping IMU sample with invalid dt: %.6f", dt);
                return;
            }

            double z_velocity = msg->angular_velocity.z - this->gyro_offset_;
            if (std::abs(z_velocity) < 0.01) {
                z_velocity = 0.0;
            }

            this->theta_ += z_velocity * dt;
            //RCLCPP_INFO(this->get_logger(), "Integrated Yaw: %f radians", this->theta_);
            auto angle_msg = std_msgs::msg::Float64();
            angle_msg.data = this->theta_;
            angle_publisher_->publish(angle_msg);
        }
    }
}

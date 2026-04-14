#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <vector>
#include <std_msgs/msg/float64.hpp>

namespace nodes {

    enum class ImuNodeMode {
        CALIBRATE,
        INTEGRATE,
    };

    class ImuNode : public rclcpp::Node {
    public:
        ImuNode();
        ~ImuNode() override = default;

        // Set the IMU mode
        void setMode(ImuNodeMode mode);

        // Get the current IMU mode
        ImuNodeMode getMode();

        // Reset the class
        void reset_imu();

    private:

        void calibrate();
        void integrate();

        ImuNodeMode mode = ImuNodeMode::INTEGRATE;

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angle_publisher_;

        std::vector<float> gyro_calibration_samples_;

        double theta_;       // Integrated yaw angle (radians)
        double gyro_offset_; // Estimated gyro bias
        rclcpp::Time last_timestamp_; // Last timestamp for integration
        bool has_last_timestamp_;

        void on_imu_msg(const sensor_msgs::msg::Imu::SharedPtr msg);
    };
}

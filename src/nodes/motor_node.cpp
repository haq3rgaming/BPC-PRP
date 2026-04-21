#include <nodes/motor_node.hpp>

#define Kp 0.1
#define Ki 0.01
#define Kd 0.01

#define DEADBAND_UPPER 140
#define DEADBAND_LOWER 110

namespace nodes {
    MotorNode::MotorNode() : Node("motor_node"),
    pid_left_(Kp, Ki, Kd),
    pid_right_(Kp, Ki, Kd)
    {
        target_left_ = 0.0;
        target_right_ = 0.0;
        measured_left_ = 0.0;
        measured_right_ = 0.0;
        last_ticks_left_ = 0;
        last_ticks_right_ = 0;
        move_active_ = false;
        target_distance_ = 0.0;

        motor_pub_ = this->create_publisher<std_msgs::msg::Int8MultiArray>(
            "/bpc_prp_robot/set_motor_speeds", 10
        );

        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/bpc_prp_robot/cmd_vel", 10,
            std::bind(&MotorNode::cmdCallback, this, std::placeholders::_1)
        );

        move_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/bpc_prp_robot/move_command", 10,
            std::bind(&MotorNode::moveCallback, this, std::placeholders::_1)
        );

        enc_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/bpc_prp_robot/encoders", 10,
            std::bind(&MotorNode::encoderCallback, this, std::placeholders::_1)
        );

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&MotorNode::controlLoop, this)
        );

        last_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "MotorNode started");
    }

    void MotorNode::cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (move_active_) return;

        double v = msg->linear.x;
        double w = msg->angular.z;

        double v_left = v - (w * WHEEL_BASE / 2.0);
        double v_right = v + (w * WHEEL_BASE / 2.0);

        target_left_ = v_left / WHEEL_RADIUS;
        target_right_ = v_right / WHEEL_RADIUS;
    }

    void MotorNode::encoderCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 2) return;

        auto now = this->now();
        double dt = (now - last_time_).seconds();

        if (dt <= 0.0) return;

        int ticks_left = msg->data[0];
        int ticks_right = msg->data[1];

        int delta_left = ticks_left - last_ticks_left_;
        int delta_right = ticks_right - last_ticks_right_;

        last_ticks_left_ = ticks_left;
        last_ticks_right_ = ticks_right;

        measured_left_ = (2.0 * M_PI * delta_left) / (TICKS_PER_REV * dt);
        measured_right_ = (2.0 * M_PI * delta_right) / (TICKS_PER_REV * dt);

        last_time_ = now;
    }

    void MotorNode::controlLoop()
    {
        double dt = 0.02;

        // ===== Distance control =====
        if (move_active_)
        {
            int delta_left = last_ticks_left_ - start_ticks_left_;
            int delta_right = last_ticks_right_ - start_ticks_right_;

            double dist_left = (2.0 * M_PI * WHEEL_RADIUS * delta_left) / TICKS_PER_REV;
            double dist_right = (2.0 * M_PI * WHEEL_RADIUS * delta_right) / TICKS_PER_REV;

            double distance = (dist_left + dist_right) / 2.0;

            double remaining = target_distance_ - distance;

            if (remaining <= 0.0)
            {
                move_active_ = false;
                target_left_ = 0.0;
                target_right_ = 0.0;

                RCLCPP_INFO(this->get_logger(), "Target reached");
            }
            else if (remaining < 0.05) // slow down near target
            {
                double slow_speed = 0.02;
                target_left_ = slow_speed / WHEEL_RADIUS;
                target_right_ = slow_speed / WHEEL_RADIUS;
            }
        }

        // ===== PID =====
        double effort_left = pid_left_.update(target_left_, measured_left_, dt);
        double effort_right = pid_right_.update(target_right_, measured_right_, dt);

        int cmd_left = STOP + static_cast<int>(effort_left);
        int cmd_right = STOP + static_cast<int>(effort_right);

        cmd_left = std::clamp(cmd_left, 0, 255);
        cmd_right = std::clamp(cmd_right, 0, 255);

        // Deadband compensation
        if (cmd_left > 127 && cmd_left < DEADBAND_UPPER) cmd_left = DEADBAND_UPPER;
        if (cmd_left < 127 && cmd_left > DEADBAND_LOWER) cmd_left = DEADBAND_LOWER;

        if (cmd_right > 127 && cmd_right < DEADBAND_UPPER) cmd_right = DEADBAND_UPPER;
        if (cmd_right < 127 && cmd_right > DEADBAND_LOWER) cmd_right = DEADBAND_LOWER;

        std_msgs::msg::Int8MultiArray msg;
        msg.data = {(int8_t)cmd_left, (int8_t)cmd_right};

        motor_pub_->publish(msg);
    }

    void MotorNode::startMove(double distance_m, double speed_mps)
    {
        target_distance_ = distance_m;

        start_ticks_left_ = last_ticks_left_;
        start_ticks_right_ = last_ticks_right_;

        move_active_ = true;

        target_left_ = speed_mps / WHEEL_RADIUS;
        target_right_ = speed_mps / WHEEL_RADIUS;

        RCLCPP_INFO(this->get_logger(), "Starting move: %.2f m", distance_m);
    }

    void MotorNode::moveCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 2) return;

        double distance = msg->data[0]; // meters
        double time = msg->data[1];     // seconds

        double speed = distance / time;

        startMove(distance, speed);
    }
}
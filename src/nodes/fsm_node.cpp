#include <nodes/fsm_node.hpp>
#include <cmath>

#define DEG_TO_RAD(x) (M_PI * (x)/180.0)
#define RAD_TO_DEG(x) ((x)*180.0/M_PI)
namespace nodes {
    FSMNode::FSMNode() : Node("fsm_node") {
        aruco_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/bpc_prp_robot/marker_data", 1,
            std::bind(&FSMNode::aruco_callback, this, std::placeholders::_1)
        );

        encoder_sub_ = this->create_subscription<std_msgs::msg::UInt32MultiArray>(
            "/bpc_prp_robot/encoders", 1,
            std::bind(&FSMNode::encoder_callback, this, std::placeholders::_1)
        );

        lidar_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/bpc_prp_robot/filtered_lidar", 1,
            std::bind(&FSMNode::lidar_callback, this, std::placeholders::_1)
        );

        imu_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/bpc_prp_robot/angle", 1,
            std::bind(&FSMNode::imu_callback, this, std::placeholders::_1)
        );

        vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/bpc_prp_robot/cmd_vel", 1
        );

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5),
            std::bind(&FSMNode::controlLoop, this)
        );

        RCLCPP_INFO(this->get_logger(), "FSMNode initialized");
    }

    void FSMNode::controlLoop() {
        float fw_speed = 0.0;
        float turn = 0.0;
        //next_state();
        switch (current_state_)
        {
            case CALIBRATION: // IMU calibration, wait for first IMU message
                fw_speed = 0.0;
                turn = 0.0;
                break;
            case CORRIDOR: // Drive forward until we detect an intersection
                if(number_of_walls() == 2 && lidar_around_.front < 0.2 && (is_wall(lidar_around_.left) || is_wall(lidar_around_.right))) {
                    if(is_wall(lidar_around_.left)){
                        target_angle_ = current_angle_- M_PI / 2;
                        current_state_ = TURN;
                        break;
                    }
                    if(is_wall(lidar_around_.right)){
                        target_angle_ = current_angle_ + M_PI / 2;
                        current_state_ = TURN;
                        break;
                    }
                }
                        
                fw_speed = std::clamp((lidar_around_.front - 0.1) / 2, 0.0, 0.1);
                turn = std::clamp(
                    (
                        std::clamp(lidar_around_.left, 0.0f, 0.2f) -
                        std::clamp(lidar_around_.right, 0.0f, 0.2f)
                    ) * 3, -1.0f, 1.0f);
                break;
            case INTERSECTION: // Stop and decide which way to turn
                fw_speed = 0.0;
                turn = 0.0;
                break;
            case TURN: // Execute the turn, then pop the intersection queue
                if (std::abs(target_angle_ - current_angle_) < 0.1) {
                    if (is_wall(lidar_around_.back))
                    {
                        fw_speed = 0.1;
                        turn = 0.0;
                    }else{
                        current_state_ = CORRIDOR;
                        break;
                    }
                    
                }else {
                    turn = std::clamp((target_angle_ - current_angle_), -1.0, 1.0);
                    fw_speed = 0.0;
                }
                break;
            case STOP: // End or invalid state, stop the robot
                fw_speed = 0.0;
                turn = 0.0;
                break;
            default:
                current_state_=STOP;
                break;
        }
        publish_velocity(fw_speed, turn);
    }

    void FSMNode::next_state() {
        if (working) return;
        int walls = number_of_walls();
        RCLCPP_INFO(this->get_logger(), "Walls: %d", walls);
        if (walls == 2 && !is_wall(lidar_around_.front)) {
            current_state_ = CORRIDOR;
        } else if (walls == 2 && is_wall(lidar_around_.front)) {
            current_state_ = TURN;
            working = true;
            if (!is_wall(lidar_around_.left)) {
                target_angle_ = current_angle_ + M_PI / 2;
            } else if (!is_wall(lidar_around_.right)) {
                target_angle_ = current_angle_ - M_PI / 2;
            } else {
                target_angle_ = current_angle_ + M_PI; // U-turn
            }
        }
        RCLCPP_INFO(this->get_logger(), "Current state: %d", current_state_);
    }

    bool FSMNode::is_wall(float distance) {
        return distance < 0.3;
    }

    int FSMNode::number_of_walls() {
        int count = 0;
        if (is_wall(lidar_around_.front)) count++;
        // if (is_wall(lidar_around_.back)) count++;
        if (is_wall(lidar_around_.left)) count++;
        if (is_wall(lidar_around_.right)) count++;
        return count;
    }

    void FSMNode::aruco_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
        ArucoMarkerID marker = static_cast<ArucoMarkerID>(msg->data);
        auto marker_intersection = convert_marker_to_intersection(marker);
        if (marker_intersection == NONE) return; // Invalid marker
        if (intersection_queue.empty()) {
            intersection_queue.push_back(marker_intersection);
            return;
        }
        auto first_element = intersection_queue.front();
        if (marker_intersection != first_element) {
            intersection_queue.push_back(marker_intersection);
        }
    }

    FSMNextIntersection FSMNode::convert_marker_to_intersection(ArucoMarkerID marker) {
        switch (marker) {
            case EXIT_FW:
                return FW;
            case EXIT_LEFT:
                return LEFT;
            case EXIT_RIGHT:
                return RIGHT;
            default:
                return NONE;
        }
    }

    FSMNextIntersection FSMNode::get_next_intersection() {
        if (intersection_queue.empty()) return NONE;
        auto next = intersection_queue.front();
        intersection_queue.erase(intersection_queue.begin());
        return next;
    }

    FSMNextIntersection FSMNode::peek_next_intersection() {
        if (intersection_queue.empty()) return NONE;
        return intersection_queue.front();
    }

    void FSMNode::encoder_callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "Received encoder message with insufficient data");
            return;
        }
        left_encoder_ticks_ = msg->data[0];
        right_encoder_ticks_ = msg->data[1];
    }

    void FSMNode::lidar_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 4) {
            RCLCPP_WARN(this->get_logger(), "Received lidar message with insufficient data");
            return;
        }
        lidar_around_.front = msg->data[0];
        lidar_around_.back = msg->data[1];
        lidar_around_.left = msg->data[2];
        lidar_around_.right = msg->data[3];
    }

    void FSMNode::imu_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        if (current_state_ == CALIBRATION) {
            working = false;
            current_state_ = CORRIDOR;
        }
        current_angle_ = msg->data;
    }

    void FSMNode::publish_velocity(double linear_x, double angular_z) {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = linear_x;
        cmd_vel.angular.z = angular_z;
        vel_publisher_->publish(cmd_vel);
    }
}

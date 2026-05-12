#include <nodes/fsm_node.hpp>
#include <cmath>

#define DEG_TO_RAD(x) (M_PI * (x)/180.0)
#define RAD_TO_DEG(x) ((x)*180.0/M_PI)
#define IN_RANGE(x, lower, upper) ((x) > (lower) && (x) < (upper))

#define WALL_DISTANCE_THRESHOLD 0.3

#define ARUCO

namespace nodes {
    FSMNode::FSMNode(rclcpp::NodeOptions options) : Node("fsm_node", options) {
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
            std::chrono::milliseconds(25),
            std::bind(&FSMNode::controlLoop, this)
        );

        RCLCPP_INFO(this->get_logger(), "FSMNode initialized");
    }

    double FSMNode::normalize_angle(double angle) {
        double target_angle = 0;
                    if (IN_RANGE(angle, DEG_TO_RAD(-20), DEG_TO_RAD(20))) {
                        target_angle = 0.0;
                    } else if (IN_RANGE(angle, DEG_TO_RAD(70), DEG_TO_RAD(110))) {
                        target_angle = DEG_TO_RAD(90);
                    } else if (IN_RANGE(angle, DEG_TO_RAD(160), DEG_TO_RAD(200))) {
                        if (current_angle_ > 0) {
                            target_angle = DEG_TO_RAD(180);
                        } else {
                            target_angle = DEG_TO_RAD(-180);
                        }
                        //target_angle = DEG_TO_RAD(180);
                    } else if (IN_RANGE(angle, DEG_TO_RAD(-110), DEG_TO_RAD(-70))) {
                        target_angle = DEG_TO_RAD(-90);
                    } else if (IN_RANGE(angle, DEG_TO_RAD(-200), DEG_TO_RAD(-160))) {
                        if (current_angle_ > 0) {
                            target_angle = DEG_TO_RAD(180);
                        } else {
                            target_angle = DEG_TO_RAD(-180);
                        }
                    } else {
                        target_angle = angle; // If it's not close to any of the cardinal directions, just use the raw angle (this should be rare)
                    }
        return target_angle;
    }
    void FSMNode::controlLoop() {
        float fw_speed = 0.0;
        float turn = 0.0;
        if (lidarDog.is_expired() && current_state_ != CALIBRATION) {
            if (lidarExpired == false) {
                RCLCPP_WARN(this->get_logger(), "Lidar data expired, stopping robot");
                lidarExpired = true;
            }
            publish_velocity(0.0, 0.0);
            return;
        } else if (lidarExpired && !lidarDog.is_expired()) {
            RCLCPP_INFO(this->get_logger(), "Lidar data received, resuming normal operation");
            lidarExpired = false;
        }
        // RCLCPP_INFO(this->get_logger(), "Lidar data age: %ld ms", lidarDog.age().count());

        switch (current_state_)
        {
            case CALIBRATION: // IMU calibration, wait for first IMU message
                fw_speed = 0.0;
                turn = 0.0;
                break;
            case CORRIDOR: // Drive forward until we detect an intersection
                if (
                    number_of_walls() >= 2 &&
                    lidar_around_.front < 0.2 &&
                    (
                        is_wall(lidar_around_.left) ||
                        is_wall(lidar_around_.right)
                    )
                    ) {
                    double notNormalized_angle = 0.0;
                    if (is_wall(lidar_around_.left)&& !is_wall(lidar_around_.right)) {
                        notNormalized_angle = current_angle_- M_PI / 2;
                    } else if(is_wall(lidar_around_.right)&& !is_wall(lidar_around_.left)) {
                        notNormalized_angle = current_angle_ + M_PI / 2;
                    } else {
                        notNormalized_angle = current_angle_ + M_PI; // U-turn
                    }
                    
                    notNormalized_angle = std::remainder(notNormalized_angle, 2.0 * M_PI); // Wrap angle to [-pi, pi]
                    //RCLCPP_INFO(this->get_logger(), "notNormalized angle: %f deg", RAD_TO_DEG(notNormalized_angle));
                    // Normalize target angle to 1/2pi,pi,-pi/2,-pi
                    target_angle_ = normalize_angle(notNormalized_angle);

                    RCLCPP_INFO(this->get_logger(), "target angle: %f deg, notnormalized %f deg", RAD_TO_DEG(target_angle_), RAD_TO_DEG(notNormalized_angle ));
                    set_state(TURN);
                    helper_angle_ = current_angle_;
                    break;
                }
                else if (lidar_around_.front < 0.2 &&
                        !is_wall(lidar_around_.left) &&
                        !is_wall(lidar_around_.right)) {
                    #ifdef ARUCO
                    double notNormalized_angle = aruco_target_angle();
                    #else
                    double notNormalized_angle = decide_target_angle();
                    #endif
                    notNormalized_angle = std::remainder(notNormalized_angle, 2.0 * M_PI); // Wrap angle to [-pi, pi]
                    target_angle_ = normalize_angle(notNormalized_angle);
                    set_state(INTERSECTION); // Treat it as an intersection to decide turn direction
                    break;
                }else if (lidar_around_.front > 0.7 &&
                        (!is_wall(lidar_around_.left) ||
                        !is_wall(lidar_around_.right))) {
                        double notNormalized_angle = current_angle_;
                        notNormalized_angle = std::remainder(notNormalized_angle, 2.0 * M_PI); // Wrap angle to [-pi, pi]
                        target_angle_ = normalize_angle(notNormalized_angle);
                        target_lidar_front_ = lidar_around_.front- 0.12; // Set target front distance slightly ahead of current reading to encourage moving forward
                        set_state(NO_FRONT_WALL_INTERSECTION); // Special state to handle intersections without a front wall
                        break;

                }
                        
                fw_speed = std::clamp((lidar_around_.front - 0.15), 0.0, 0.1);
                turn = turn_pid_.update(
                    0, 
                    std::clamp(lidar_around_.right, 0.0f, 0.2f)-
                    std::clamp(lidar_around_.left, 0.0f, 0.2f),
                    0.005);
                //RCLCPP_INFO(this->get_logger(), "fw_speed: %f, turn: %f", fw_speed, turn);
                break;
            case INTERSECTION: // Stop and decide which way to turn
                if (std::abs(target_angle_ - current_angle_) < 0.1) {
                    //if (!is_wall(lidar_around_.left)) {
                    //    fw_speed = 0.1;
                    //    turn = 0.0;
                    //} else {
                        set_state(GO_TO);
                        target_lidar_front_ = lidar_around_.front - 0.1;
                        fw_speed = 0.1;
                        turn = 0.0;
                        break;
                    //}
                    
                } else {
                    turn = std::clamp((target_angle_ - current_angle_), -1.0, 1.0);
                    fw_speed = 0.0;
                }
                break;
            case NO_FRONT_WALL_INTERSECTION: // Handle intersections without a front wall by treating them like regular intersections but with a more lenient turn condition
                if (std::abs(target_angle_ - current_angle_) < 0.1) {
                    if (lidar_around_.front > target_lidar_front_) {
                        fw_speed = 0.1;
                        turn = 0.0;
                    } else {
                        if(is_wall(lidar_around_.left) && is_wall(lidar_around_.right)) {
                            set_state(CORRIDOR);
                        }else{
                            #ifdef ARUCO
                            double notNormalized_angle = aruco_target_angle();
                            #else
                            double notNormalized_angle = decide_target_angle();
                            #endif
                            notNormalized_angle = std::remainder(notNormalized_angle, 2.0 * M_PI); // Wrap angle to [-pi, pi]
                            target_angle_ = normalize_angle(notNormalized_angle);
                            set_state(INTERSECTION);
                        }
                    }
                    
                } else {
                    turn = std::clamp((target_angle_ - current_angle_), -1.0, 1.0);
                    fw_speed = 0.0;
                }
                break;
            case TURN: // Execute the turn, then pop the intersection queue
                {
                    double angle_error = std::remainder(target_angle_ - current_angle_, 2.0 * M_PI);
                    if (std::abs(angle_error) < 0.1) {
                        if (lidar_around_.back<0.4) {
                            fw_speed = 0.1;
                            turn = 0.0;
                        } else {
                            set_state(GO_TO);
                            target_lidar_front_ = lidar_around_.front - 0.04;
                            fw_speed = 0.1;
                            turn = 0.0;
                        }
                    } else {
                        turn = static_cast<float>(std::fmax(-1.0, std::fmin(1.0, angle_error)));
                        fw_speed = 0.0;
                    }
                }
                break;
            case GO_TO: // Move towards the target (used for intersections without a front wall)
                if (lidar_around_.front > target_lidar_front_) {
                    fw_speed = 0.1;
                    turn = 0.0;
                } else {
                    set_state(CORRIDOR);
                }
                break;
            case STOP: // End or invalid state, stop the robot
                fw_speed = 0.0;
                turn = 0.0;
                break;
            default:
                set_state(STOP);
                break;
        }
        publish_velocity(fw_speed, turn);
    }

    bool FSMNode::is_wall(float distance) {
        return distance < WALL_DISTANCE_THRESHOLD;
    }

    int FSMNode::number_of_walls() {
        int count = 0;
        if (is_wall(lidar_around_.front)) count++;
        // if (is_wall(lidar_around_.back)) count++;
        if (is_wall(lidar_around_.left)) count++;
        if (is_wall(lidar_around_.right)) count++;
        return count;
    }

    void FSMNode::set_state(FSMState new_state) {
        if (current_state_ != new_state) {
            RCLCPP_INFO(this->get_logger(), "Transitioning from state %s to state %s", FSMStateNames[current_state_], FSMStateNames[new_state]);
            current_state_ = new_state;
        }
    }

    double FSMNode::decide_target_angle() {
        if (!is_wall(lidar_around_.right)) { // prefer right turns if both sides are open
            return current_angle_ - M_PI / 2;
        }
        if (!is_wall(lidar_around_.front)) { // if we cant go right, prefer going straight if possible
            return current_angle_; // go straight if possible
        }
        if (!is_wall(lidar_around_.left)) {
            return current_angle_ + M_PI / 2;
        }
        return current_angle_ + M_PI; // U-turn
    }

    double FSMNode::aruco_target_angle() {
        if (exit_queue_.is_empty()) {
            RCLCPP_WARN(this->get_logger(), "No intersections in queue, defaulting to simple turn logic");
            return decide_target_angle();
        }
        
        switch (exit_queue_.pop())
        {
            case FW:
                return current_angle_;
            case LEFT:
                if (!is_wall(lidar_around_.left)) {
                    return current_angle_ + M_PI / 2;
                }
                RCLCPP_WARN(this->get_logger(), "Left turn indicated by marker but left wall detected, defaulting to simple turn logic");
            case RIGHT:
                if (!is_wall(lidar_around_.right)) {
                    return current_angle_ - M_PI / 2;
                }
                RCLCPP_WARN(this->get_logger(), "Right turn indicated by marker but right wall detected, defaulting to simple turn logic");
            default:
                RCLCPP_WARN(this->get_logger(), "Invalid intersection type in queue, defaulting to simple turn logic");
        }
        return decide_target_angle();
    }

    void FSMNode::aruco_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
        ArucoMarkerID marker = static_cast<ArucoMarkerID>(msg->data);
        FSMNextIntersection marker_intersection = convert_marker_to_intersection(marker);
        if (marker_intersection == NONE) return;
        if (exit_queue_.push(marker_intersection)) {
            RCLCPP_INFO(this->get_logger(), "Added intersection %s to queue", FSMNextIntersectionNames[marker_intersection]);
        }
    }
    FSMNextIntersection FSMNode::convert_marker_to_intersection(ArucoMarkerID marker) {
        // Only consider exit markers for now, we can add treasure markers later if needed
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
        lidarDog.kick();
    }

    void FSMNode::imu_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        set_state(current_state_ == CALIBRATION ? CORRIDOR : current_state_);
        current_angle_ = msg->data;
    }

    void FSMNode::publish_velocity(double linear_x, double angular_z) {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = linear_x;
        cmd_vel.angular.z = angular_z;
        vel_publisher_->publish(cmd_vel);
    }
}

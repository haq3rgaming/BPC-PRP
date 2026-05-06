#include "nodes/lidar_node.hpp"
#include <cmath>

#define DEG_TO_RAD(x) (M_PI * (x)/180.0)

namespace nodes {
    LidarNode::LidarNode() : Node("lidar_node") {
        filter_data.right = 0;
        filter_data.left = 0;
        filter_data.front = 0;
        filter_data.back = 0;
        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/bpc_prp_robot/lidar",
            1,
            std::bind(&LidarNode::on_lidar_callback, this, std::placeholders::_1)
        );
        filtered_lidar = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/bpc_prp_robot/filtered_lidar",
            1
        );
        /*
        timer_ = create_wall_timer(
            std::chrono::milliseconds(2),
            std::bind(&LidarNode::filter, this)
        );
        */
        RCLCPP_INFO(this->get_logger(), "LidarNode initialized");
    }

    void LidarNode::filter(){
        filter_data.right = 0;
        filter_data.left = 0;
        filter_data.front = 0;
        filter_data.back = 0;

        int max = lidar_data_.size();
        int f = 0;
        int b = 0;
        int l = 0;
        int r = 0;

        for(int i = 0 ; i<max;i++){
            if(!std::isnan(lidar_data_[i])&&!std::isinf(lidar_data_[i])){
            if(i*angle_step<=DEG_TO_RAD(10) || i*angle_step>DEG_TO_RAD(350)){
                filter_data.front+=lidar_data_[i];
                f++;
            }

            if(i*angle_step>=DEG_TO_RAD(80) && i*angle_step<DEG_TO_RAD(100)){
                filter_data.left+=lidar_data_[i];
                l++;
            }

            if(i*angle_step>=DEG_TO_RAD(170) && i*angle_step<DEG_TO_RAD(190)){
                filter_data.back+=lidar_data_[i];
                b++;
            }

            if(i*angle_step>=DEG_TO_RAD(260) && i*angle_step<DEG_TO_RAD(280)){
                filter_data.right+=lidar_data_[i];
                r++;
            }
            }
        }

        if(r!=0) filter_data.right = filter_data.right/r;
        if(l!=0) filter_data.left = filter_data.left/l;
        if(f!=0) filter_data.front = filter_data.front/f;
        if(b!=0) filter_data.back = filter_data.back/b;

        auto msg = std_msgs::msg::Float32MultiArray();
        // Order: front, back, left, right
        msg.data = {filter_data.front, filter_data.back , filter_data.left, filter_data.right};
        filtered_lidar->publish(msg);
    }

    void LidarNode::on_lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
        angle_step=msg->angle_increment;
        lidar_data_=msg->ranges; // Is in m
        filter();
    }
}
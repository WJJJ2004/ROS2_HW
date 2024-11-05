#include "wall_follower/wall_follower.hpp"

WallFollower::WallFollower()
    : Node("wall_follower")
{
    // 파라미터 설정
    this->declare_parameter<double>("desired_distance", 0.7);  // 벽과의 목표 거리 2.0 미터로 설정
    desired_distance_ = this->get_parameter("desired_distance").as_double();

    // 토픽 구독 및 퍼블리셔 설정
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&WallFollower::scan_callback, this, std::placeholders::_1)
    );
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
}

void WallFollower::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    float left_distance = msg->ranges[msg->ranges.size() / 4];
    float front_distance = msg->ranges[msg->ranges.size() / 2];          // 전방 거리
    float front_left_distance = msg->ranges[(msg->ranges.size() / 4) * 3]; // 전방 좌측 거리

    float left_end = desired_distance_;                     //좌측 임계거리 
    float left_front_end = desired_distance_*2.236;         // 좌측-전방 45도 방향 임계거리  
    float front_end = desired_distance_*5.0;                // 전방 임계거리 

    geometry_msgs::msg::Twist cmd_vel_msg;

    if (std::isinf(front_distance) || front_distance > front_end)
    {
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.angular.z = -0.5;
    }
    else if (left_distance < left_end)
    {
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.angular.z = -0.2;
    }
    else if (front_left_distance < left_front_end)
    {
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.angular.z = -0.2;
    }
    else
    {
        cmd_vel_msg.linear.x = 0.1; // 전진
        cmd_vel_msg.angular.z = 0.0;
    }

    cmd_vel_publisher_->publish(cmd_vel_msg);
}

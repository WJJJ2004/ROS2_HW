// include/wall_follower/wall_follower.hpp
#ifndef WALL_FOLLOWER_HPP
#define WALL_FOLLOWER_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class WallFollower : public rclcpp::Node 
{
public:
    WallFollower(); // 생성자

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg); // 콜백 함수

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    double desired_distance_; // 벽과의 거리 유지 파라미터
};

#endif // WALL_FOLLOWER_HPP


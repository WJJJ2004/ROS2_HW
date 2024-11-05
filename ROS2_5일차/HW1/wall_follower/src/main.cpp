// src/main.cpp
#include "wall_follower/wall_follower.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WallFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


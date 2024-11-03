#ifndef TURTLESIM_CLI_HPP
#define TURTLESIM_CLI_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include "std_srvs/srv/empty.hpp"
#include <termios.h>
#include <unistd.h>
#include <chrono>
#include <iostream>
#include "turtlesim/srv/kill.hpp"
#include "turtlesim/srv/spawn.hpp"

using namespace std::chrono_literals;

class TurtlesimCli : public rclcpp::Node 
{
public:
    TurtlesimCli();
    void process_input();

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr set_pen_client_;
    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr teleport_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_client_;
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_client_;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist msg;
    std::shared_ptr<rclcpp::AsyncParametersClient> parameter_client; // 파라미터 클라이언트 추가

    void init_msg();
    char get_key();
    void set_pen();
    int get_color_value(const std::string &color_name);
    int get_pen_width();
    void set_background_color();
    void change_turtle_shape();
};

#endif // TURTLESIM_CLI_HPP


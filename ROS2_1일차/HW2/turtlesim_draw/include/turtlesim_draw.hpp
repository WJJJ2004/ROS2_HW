#ifndef TURTLESIM_DRAW_HPP
#define TURTLESIM_DRAW_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <turtlesim/srv/set_pen.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>
#include <chrono>
#include <string>

class TurtlesimDraw : public rclcpp::Node 
{
public:
    TurtlesimDraw();

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist msg;
    int shape_choice_;
    float size_;
    std::string color_;
    int pen_thickness_;

    void init_msg();
    void set_pen(const std::string& color, int thickness);
    void draw_shape();
    void draw_square(float size);
    void draw_circle(float radius);
    void draw_triangle(float size);
};

#endif // TURTLESIM_DRAW_HPP


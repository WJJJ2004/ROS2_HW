#include "turtlesim_draw.hpp"
#include <iostream>
#include <thread>


using namespace std::chrono_literals;

TurtlesimDraw::TurtlesimDraw() : Node("turtlesim_draw_node") 
{
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&TurtlesimDraw::draw_shape, this));
    init_msg();  // 메시지 초기화용 

    std::cout << "그릴 모양을 선택하세요 1: 사각형 2: 원 3: 삼각형";
    std::cin >> shape_choice_;

    std::cout << "원하는 사이즈를 입력하세요";
    std::cin >> size_;

    std::cout << "펜 색상을 입력하세요 (red, green, blue): ";
    std::cin >> color_;

    std::cout << "펜 두께를 입력하세요 ";
    std::cin >> pen_thickness_;

    set_pen(color_, pen_thickness_);
}

void TurtlesimDraw::init_msg() 
{
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
}

void TurtlesimDraw::set_pen(const std::string& color, int thickness) 
{
    auto client = this->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");
    auto request = std::make_shared<turtlesim::srv::SetPen::Request>();

    if (color == "red") 
    {
        request->r = 255; request->g = 0; request->b = 0;
    } 
    else if (color == "green") 
    {
        request->r = 0; request->g = 255; request->b = 0;
    } 
    else if (color == "blue") 
    {
        request->r = 0; request->g = 0; request->b = 255;
    }

    request->width = thickness;
    client->async_send_request(request);
}


void TurtlesimDraw::draw_shape() 
{
    switch (shape_choice_) 
    {
        case 1:
            draw_square(size_);
            break;
        case 2:
            draw_circle(size_);
            break;
        case 3:
            draw_triangle(size_);
            break;
        default:
            RCLCPP_INFO(this->get_logger(), "선택한 색상이 유효하지 않습니다");
            break;
    }
    rclcpp::shutdown();  // 완료 후 노드 종료
}

void TurtlesimDraw::draw_square(float size) 
{
    for (int i = 0; i < 4; ++i) 
    {
        msg.linear.x = size;
        msg.angular.z = 0;
        publisher_->publish(msg);
        std::this_thread::sleep_for(std::chrono::seconds(1));

        msg.linear.x = 0;
        msg.angular.z = 1.5708;  // 90도 회전
        publisher_->publish(msg);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    init_msg();
}

void TurtlesimDraw::draw_circle(float radius) 
{
    for (int i = 0; i < 36; ++i)  
    {
        msg.linear.x = radius;
        msg.angular.z = 1.0;
        publisher_->publish(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    init_msg();
}

void TurtlesimDraw::draw_triangle(float size) 
{
    for (int i = 0; i < 3; ++i) 
    {
        msg.linear.x = size;
        msg.angular.z = 0;
        publisher_->publish(msg);
        std::this_thread::sleep_for(std::chrono::seconds(1));

        msg.linear.x = 0;
        msg.angular.z = 2.0944;  // 120도 회전
        publisher_->publish(msg);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    init_msg();
}


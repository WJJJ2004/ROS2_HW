#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include "std_srvs/srv/empty.hpp"
#include <termios.h>
#include <unistd.h>
#include <chrono>
#include <iostream>

using namespace std::chrono_literals;

class TurtlesimCli : public rclcpp::Node 
{
public:
    TurtlesimCli() : Node("turtlesim_cli_node")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        set_pen_client_ = this->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");
        reset_client_ = this->create_client<std_srvs::srv::Empty>("/reset");
        clear_client_ = this->create_client<std_srvs::srv::Empty>("/clear"); 
        timer_ = this->create_wall_timer(100ms, std::bind(&TurtlesimCli::process_input, this));
        init_msg();
    }
    
    void process_input() 
    {
        char key = get_key();

        switch (key)
        {
            case 'W': case 'w':
                msg.linear.x = 1.0;
                break;
            case 'S': case 's':
                msg.linear.x = -1.0;
                break;
            case 'A': case 'a':
                msg.angular.z = 1.0;
                break;
            case 'D': case 'd':
                msg.angular.z = -1.0;
                break;
            case 'p':
                set_pen();  // 펜 설정 호출
                return;
            case 'b':
                set_background_color();  // 배경색 변경 호출
                return;
            case 'm':
                change_turtle_shape();  // 거북이 모양 변경 호출
                return;
            case 'q':
                rclcpp::shutdown();  // 프로그램 종료
                return;
            default:
                break;
        }

        publisher_->publish(msg);  // 터틀의 움직임 업데이트
        init_msg();  // 명령 초기화
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr set_pen_client_;
    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr teleport_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_client_;  
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist msg;

    void init_msg()
    {
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
    }

    // 키보드 입력을 비차단 방식으로 처리
    char get_key()
    {
        struct termios oldt, newt;
        char ch;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        ch = getchar();
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        return ch;
    }

    // 펜 설정 함수
    void set_pen()
    {
        int r = get_color_value("R");  // R 값 입력
        int g = get_color_value("G");  // G 값 입력
        int b = get_color_value("B");  // B 값 입력
        int width = get_pen_width();   // 펜 굵기 입력

        // 요청 메시지 생성
        auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
        request->r = r;
        request->g = g;
        request->b = b;
        request->width = width;

        // 비동기 서비스 요청 후 콜백 함수 등록
        set_pen_client_->async_send_request(request, 
            [this](rclcpp::Client<turtlesim::srv::SetPen>::SharedFuture response) 
            {
                // 요청이 성공하면 성공 메시지 출력
                if (response.valid()) 
                {
                    std::cout << "펜 설정이 성공적으로 적용되었습니다." << std::endl;
                } 
                else 
                {
                    std::cerr << "펜 설정에 실패했습니다." << std::endl;
                }
            });
    }

    // RGB 값 입력 받는 함수
    int get_color_value(const std::string &color_name)
    {
        int value;
        while (true) 
        {
            std::cout << color_name << " 값을 입력하세요 (0 ~ 255): ";
            std::cin >> value;

            if (value >= 0 && value <= 255) 
            {
                break;  // 유효한 값이면 반복 종료
            } 
            else 
            {
                std::cerr << "잘못된 입력입니다. " << color_name << " 값은 0에서 255 사이여야 합니다." << std::endl;
            }
        }
        return value;
    }

    // 펜 굵기 입력 받는 함수
    int get_pen_width()
    {
        int width;
        while (true) 
        {
            std::cout << "펜의 굵기를 입력하세요 (1 ~ 10): ";
            std::cin >> width;

            if (width >= 1 && width <= 10) 
            {
                break;  // 유효한 값이면 반복 종료
            } 
            else 
            {
                std::cerr << "잘못된 입력입니다. 펜 굵기는 1에서 10 사이여야 합니다." << std::endl;
            }
        }   
        return width;
    }

    // 배경색 설정 함수
    void set_background_color()
    {
        // 배경색을 변경할 파라미터를 선언
        this->declare_parameter("background_r", 0);
        this->declare_parameter("background_g", 0);
        this->declare_parameter("background_b", 0);

        // 사용자로부터 R, G, B 값을 입력받음
        int r = get_color_value("배경 R");
        int g = get_color_value("배경 G");
        int b = get_color_value("배경 B");

        // 배경색을 변경할 파라미터 설정
        this->set_parameter(rclcpp::Parameter("background_r", r));
        this->set_parameter(rclcpp::Parameter("background_g", g));
        this->set_parameter(rclcpp::Parameter("background_b", b));

        // /clear 서비스 호출 (비동기 요청 후 콜백 함수 등록)
        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        clear_client_->async_send_request(request, 
            [this](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture response) 
            {
                // 요청이 성공하면 성공 메시지 출력
                if (response.valid()) 
                {
                    std::cout << "배경색이 성공적으로 변경되었습니다." << std::endl;
                } 
                else 
                {
                    std::cerr << "배경색 변경에 실패했습니다." << std::endl;
                }
            });
    }

    // 거북이 모양 설정 함수 (구현 예정) 10/30/24 
    void change_turtle_shape()
    {
        std::cout << "아직 구현하지 못했습니다. \n";
    }
};

int main(int argc, char *argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtlesimCli>());
    rclcpp::shutdown();
    return 0;
}

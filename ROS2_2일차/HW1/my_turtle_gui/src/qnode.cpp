#include "../include/my_turtle_gui/qnode.hpp"
#include <turtlesim/srv/set_pen.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>
#include <std_srvs/srv/empty.hpp>
#include <chrono>
#include <thread>
#include <iostream>

QNode::QNode()
{
    int argc = 0;
    char** argv = NULL;
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("my_turtle_gui");
    velocity_publisher = node->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
    set_pen_client = node->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");
    teleport_client = node->create_client<turtlesim::srv::TeleportAbsolute>("/turtle1/teleport_absolute");
    clear_client = node->create_client<std_srvs::srv::Empty>("/clear");
    parameter_client = std::make_shared<rclcpp::AsyncParametersClient>(node, "turtlesim");

    this->start();
}

QNode::~QNode()
{
    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }
}

void QNode::run()
{
    rclcpp::WallRate loop_rate(20);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    Q_EMIT rosShutDown();
}

void QNode::publishTwist(float linear, float angular)
{
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = linear;
    msg.angular.z = angular;
    velocity_publisher->publish(msg);

    // 메시지 발행 후 UI에 로그 출력 신호 전달
    QString log = QString("Published Twist - Linear: %1, Angular: %2").arg(linear).arg(angular);
    emit messageToUI(log);  // MainWindow로 로그 메시지 전달
}

void QNode::drawCircle()
{
    for (int i = 0; i < 36; ++i)
    {
        publishTwist(1.0, 1.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    publishTwist(0.0, 0.0);
}

void QNode::drawSquare()
{
    for (int i = 0; i < 4; ++i)
    {
        publishTwist(1.0, 0.0);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        publishTwist(0.0, 1.5708);  // 90도 회전
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    publishTwist(0.0, 0.0);
}

void QNode::drawTriangle()
{
    for (int i = 0; i < 3; ++i)
    {
        publishTwist(1.0, 0.0);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        publishTwist(0.0, 2.0944);  // 120도 회전
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    publishTwist(0.0, 0.0);
}

void QNode::setPen()
{
    auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
    request->r = 255;
    request->g = 0;
    request->b = 0;
    request->width = 3;
    set_pen_client->async_send_request(request);
}

void QNode::setBackground()
{
    if (!parameter_client->wait_for_service(std::chrono::seconds(5)))
    {
        std::cerr << "파라미터 서비스에 연결할 수 없습니다.\n";
        return;
    }

    // 배경색 파라미터 설정
    parameter_client->set_parameters(
        {
            rclcpp::Parameter("background_r", 0),
            rclcpp::Parameter("background_g", 255),
            rclcpp::Parameter("background_b", 0)
        },
        [this](const std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>& result)
        {
            bool success = true;
            for (const auto& res : result.get())
            {
                if (!res.successful)
                {
                    success = false;
                    break;
                }
            }

            if (success)
            {
                // 설정 성공 시 clear 서비스 호출
                auto request = std::make_shared<std_srvs::srv::Empty::Request>();
                clear_client->async_send_request(
                    request,
                    [this](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture response)
                    {
                        if (response.valid())
                        {
                            std::cout << "배경색이 성공적으로 변경되었습니다." << std::endl;
                        }
                        else
                        {
                            std::cerr << "배경색 변경에 실패했습니다." << std::endl;
                        }
                    }
                    );
            }
            else
            {
                std::cerr << "배경색 설정에 실패했습니다." << std::endl;
            }
        }
        );
}

void QNode::setTurtleType()
{
    std::cout << "거북이 모양 변경 기능은 아직 구현되지 않았습니다." << std::endl;
}

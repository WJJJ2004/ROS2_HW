#include "turtlesim_cli.hpp"

TurtlesimCli::TurtlesimCli() : Node("turtlesim_cli_node")
{
    std::cout << "=================== Turtlesim CLI Package ===================" << std::endl;
    std::cout << "This package allows you to control the turtlesim node using keyboard inputs." << std::endl;
    std::cout << "Usage instructions:" << std::endl;
    std::cout << "  - W / w : Move forward" << std::endl;
    std::cout << "  - S / s : Move backward" << std::endl;
    std::cout << "  - A / a : Turn left" << std::endl;
    std::cout << "  - D / d : Turn right" << std::endl;
    std::cout << "  - p     : Set pen color and width" << std::endl;
    std::cout << "  - b     : Change background color" << std::endl;
    std::cout << "  - m     : Change turtle shape" << std::endl;
    std::cout << "  - q     : Quit the program" << std::endl;
    std::cout << "============================================================" << std::endl;

    this->declare_parameter("background_r", 0);
    this->declare_parameter("background_g", 0);
    this->declare_parameter("background_b", 0);

    kill_client_ = this->create_client<turtlesim::srv::Kill>("/kill");
    spawn_client_ = this->create_client<turtlesim::srv::Spawn>("/spawn");
        parameter_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "turtlesim");  // 파라미터 클라이언트 초기화


    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
    set_pen_client_ = this->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");
    reset_client_ = this->create_client<std_srvs::srv::Empty>("/reset");
    clear_client_ = this->create_client<std_srvs::srv::Empty>("/clear");
    timer_ = this->create_wall_timer(100ms, std::bind(&TurtlesimCli::process_input, this));
    init_msg();
}

void TurtlesimCli::process_input()
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
            set_pen();
            return;
        case 'b':
            set_background_color();
            return;
        case 'm':
            change_turtle_shape();
            return;
        case 'q':
            rclcpp::shutdown();
            return;
        default:
            break;
    }

    publisher_->publish(msg);
    init_msg();
}

void TurtlesimCli::init_msg()
{
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
}

char TurtlesimCli::get_key()
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

void TurtlesimCli::set_pen()
{
    int r = get_color_value("R");
    int g = get_color_value("G");
    int b = get_color_value("B");
    int width = get_pen_width();

    auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
    request->r = r;
    request->g = g;
    request->b = b;
    request->width = width;

    set_pen_client_->async_send_request(request,
        [this](rclcpp::Client<turtlesim::srv::SetPen>::SharedFuture response)
        {
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

int TurtlesimCli::get_color_value(const std::string &color_name)
{
    int value;
    while (true)
    {
        std::cout << color_name << " 값을 입력하세요 (0 ~ 255): ";
        std::cin >> value;

        if (value >= 0 && value <= 255) 
        {
            break;
        }
        else
        {
            std::cerr << "잘못된 입력입니다. " << color_name << " 값은 0에서 255 사이여야 합니다." << std::endl;
        }
    }
    return value;
}

int TurtlesimCli::get_pen_width()
{
    int width;
    while (true)
    {
        std::cout << "펜의 굵기를 입력하세요 (1 ~ 10): ";
        std::cin >> width;

        if (width >= 1 && width <= 10)
        {
            break;
        }
        else
        {
            std::cerr << "잘못된 입력입니다. 펜 굵기는 1에서 10 사이여야 합니다." << std::endl;
        }
    }
    return width;
}

void TurtlesimCli::set_background_color()
{

    int r = get_color_value("배경 R");
    int g = get_color_value("배경 G");
    int b = get_color_value("배경 B");

    this->set_parameter(rclcpp::Parameter("background_r", r));
    this->set_parameter(rclcpp::Parameter("background_g", g));
    this->set_parameter(rclcpp::Parameter("background_b", b));

    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    clear_client_->async_send_request(request,
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
        });
}

void TurtlesimCli::change_turtle_shape()
{
    // 사용자에게 선택 사항 안내 및 입력받기
    std::cout << "\n1: ardent\n2: bouncy\n3: crystal\n4: dashing\n";
    int choice;
    std::cin >> choice;

    // 선택에 따라 거북이 모양 설정 (기본값은 ardent)
    std::string turtle_name;
    switch(choice) {
        case 1: turtle_name = "ardent"; break;
        case 2: turtle_name = "bouncy"; break;
        case 3: turtle_name = "crystal"; break;
        case 4: turtle_name = "dashing"; break;
        default:
            std::cerr << "잘못된 선택입니다. 기본값(ardent)으로 설정됩니다.\n";
            turtle_name = "ardent";
    }

    // 기존 거북이 삭제
    auto kill_request = std::make_shared<turtlesim::srv::Kill::Request>();
    if (!kill_client_->wait_for_service(std::chrono::seconds(1))) {
        std::cerr << "Kill 서비스에 연결할 수 없습니다.\n";
        return;
    }
    auto kill_result = kill_client_->async_send_request(kill_request);
    if (rclcpp::spin_until_future_complete(this->shared_from_this(), kill_result) != rclcpp::FutureReturnCode::SUCCESS) {
        std::cerr << "기존 거북이를 삭제하는 데 실패했습니다.\n";
        return;
    }
    std::cout << "기존 거북이 삭제 완료.\n";

    // 새 거북이 모양 파라미터 설정
    auto parameter_client = std::make_shared<rclcpp::AsyncParametersClient>(this->shared_from_this(), "turtlesim");
    if (!parameter_client->wait_for_service(std::chrono::seconds(5))) {
        std::cerr << "파라미터 서비스에 연결할 수 없습니다.\n";
        return;
    }
    auto set_param_result = parameter_client->set_parameters({rclcpp::Parameter("turtle_shape", turtle_name)});
    if (rclcpp::spin_until_future_complete(this->shared_from_this(), set_param_result) != rclcpp::FutureReturnCode::SUCCESS) {
        std::cerr << "거북이 모양 설정에 실패했습니다.\n";
        return;
    }
    std::cout << "거북이 모양이 '" << turtle_name << "'으로 설정되었습니다.\n";

    // 새로운 거북이 생성
    auto spawn_request = std::make_shared<turtlesim::srv::Spawn::Request>();
    spawn_request->x = 5.5;        // 화면 중앙 X 좌표
    spawn_request->y = 5.5;        // 화면 중앙 Y 좌표
    spawn_request->theta = 0.0;    // 초기 방향

    if (!spawn_client_->wait_for_service(std::chrono::seconds(1))) {
        std::cerr << "Spawn 서비스에 연결할 수 없습니다.\n";
        return;
    }
    auto spawn_result = spawn_client_->async_send_request(spawn_request);
    if (rclcpp::spin_until_future_complete(this->shared_from_this(), spawn_result) != rclcpp::FutureReturnCode::SUCCESS) {
        std::cerr << "새 거북이 생성에 실패했습니다.\n";
        return;
    }
    std::cout << "새로운 거북이가 생성되었습니다.\n";
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtlesimCli>());
    rclcpp::shutdown();
    return 0;
}

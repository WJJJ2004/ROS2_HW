#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int64.hpp"

class Talker : public rclcpp::Node
{
public:
  Talker() : Node("talker_node"), count_(0)
  {
    string_publisher_ = this->create_publisher<std_msgs::msg::String>("/chatter_cli", 10);
    count_publisher_ = this->create_publisher<std_msgs::msg::Int64>("/chatter_count", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Talker::publish_message, this));
  }

private:
  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr count_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int64_t count_;
  
  void publish_message()
  {
    auto string_msg = std::make_shared<std_msgs::msg::String>();  
    //std::shared_ptr 객체를 생성 >> 참조 횟수 카운트하기 
    auto count_msg = std::make_shared<std_msgs::msg::Int64>();

    std::cout << "Enter your message: ";
    std::getline(std::cin, string_msg->data);  

    count_++;
    count_msg->data = count_;

    RCLCPP_INFO(this->get_logger(), "Publishing: '%s', Count: '%ld'", string_msg->data.c_str(), count_msg->data);
    string_publisher_->publish(*string_msg);
    count_publisher_->publish(*count_msg);
  }

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Talker>());
  rclcpp::shutdown();
  return 0;
}


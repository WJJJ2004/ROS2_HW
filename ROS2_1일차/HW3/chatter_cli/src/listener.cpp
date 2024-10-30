#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int64.hpp"

class Listener : public rclcpp::Node
{
public:
  Listener() : Node("listener_node")
  {
    string_subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/chatter_cli", 10, std::bind(&Listener::string_callback, this, std::placeholders::_1));
    count_subscription_ = this->create_subscription<std_msgs::msg::Int64>(
        "/chatter_count", 10, std::bind(&Listener::count_callback, this, std::placeholders::_1));
  }

private:
  
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr string_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr count_subscription_;
  std::string last_message_;
  
  void string_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    last_message_ = msg->data;
  }

  void count_callback(const std_msgs::msg::Int64::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Subscribed: '%s' '%ld'", last_message_.c_str(), msg->data);
  }

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Listener>());
  rclcpp::shutdown();
  return 0;
}


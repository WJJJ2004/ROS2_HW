#include "chatter_cli/listener.hpp"

Listener::Listener() : Node("listener_node")
{
  string_subscription_ = this->create_subscription<std_msgs::msg::String>(
      "/chatter_cli", 10, std::bind(&Listener::string_callback, this, std::placeholders::_1));
  count_subscription_ = this->create_subscription<std_msgs::msg::Int64>(
      "/chatter_count", 10, std::bind(&Listener::count_callback, this, std::placeholders::_1));
}

void Listener::string_callback(const std_msgs::msg::String::SharedPtr msg)
{
  last_message_ = msg->data;
}

void Listener::count_callback(const std_msgs::msg::Int64::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Subscribed: '%s' '%ld'", last_message_.c_str(), msg->data);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Listener>());
  rclcpp::shutdown();
  return 0;
}


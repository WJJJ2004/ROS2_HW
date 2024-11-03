#ifndef CHATTER_CLI_LISTENER_HPP
#define CHATTER_CLI_LISTENER_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int64.hpp"

class Listener : public rclcpp::Node
{
public:
  Listener();

private:
  void string_callback(const std_msgs::msg::String::SharedPtr msg);
  void count_callback(const std_msgs::msg::Int64::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr string_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr count_subscription_;
  std::string last_message_;
};

#endif // CHATTER_CLI_LISTENER_HPP


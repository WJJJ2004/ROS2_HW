#ifndef CHATTER_CLI_TALKER_HPP
#define CHATTER_CLI_TALKER_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int64.hpp"

class Talker : public rclcpp::Node
{
public:
  Talker();

private:
  void publish_message();

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr count_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int64_t count_;
};

#endif // CHATTER_CLI_TALKER_HPP


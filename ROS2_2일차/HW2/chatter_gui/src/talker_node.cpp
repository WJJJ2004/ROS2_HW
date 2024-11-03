#include "chatter_gui/talker_node.hpp"

Talker::Talker() : Node("talker")
{
  publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
}

void Talker::publishMessage(const std::string &message)
{
  auto msg = std_msgs::msg::String();
  msg.data = message;
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.c_str());
  publisher_->publish(msg);
}

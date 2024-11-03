#include "../include/chatter_gui/listener_node.hpp"
#include <iostream>

Listener::Listener() : Node("listener")
{
  subscription_ = this->create_subscription<std_msgs::msg::String>(
    "chatter", 10, std::bind(&Listener::messageCallback, this, std::placeholders::_1));
}

void Listener::messageCallback(const std_msgs::msg::String::SharedPtr msg)
{
  QString received_msg = QString::fromStdString(msg->data);
  emit messageReceived(received_msg);
}


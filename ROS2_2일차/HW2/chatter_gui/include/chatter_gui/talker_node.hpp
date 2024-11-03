#ifndef chatter_gui_TALKER_HPP_
#define chatter_gui_TALKER_HPP_


#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#endif

class Talker : public rclcpp::Node
{
public:
    Talker();
    void publishMessage(const std::string &message);
private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

#endif

// talker.hpp
#ifndef TALKER_HPP_
#define TALKER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class TalkerNode : public rclcpp::Node {
public:
    TalkerNode() : Node("talker_node") {
        // 퍼블리셔 초기화
        publisher_ = this->create_publisher<std_msgs::msg::String>("talker_topic", 10);
    }

    void publishMessage(const std::string &message) {
        // 메시지 발행
        std_msgs::msg::String msg;
        msg.data = message;
        publisher_->publish(msg);
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;  // 퍼블리셔
};

#endif // TALKER_HPP_

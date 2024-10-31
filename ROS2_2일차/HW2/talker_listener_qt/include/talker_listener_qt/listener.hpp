// listener.hpp
#ifndef LISTENER_HPP_
#define LISTENER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <QObject>

class ListenerNode : public rclcpp::Node {
    Q_OBJECT  // Qt 시그널과 슬롯 사용을 위해 추가
public:
    ListenerNode() : Node("listener_node") {
        // 서브스크라이버 초기화
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "talker_topic", 10,
            [this](std_msgs::msg::String::SharedPtr msg) {
                // 메시지를 수신하면 시그널을 통해 UI로 전달
                emit messageReceived(QString::fromStdString(msg->data));
            });
    }

signals:
    void messageReceived(const QString &message);  // UI에 메시지 전달을 위한 시그널

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;  // 서브스크라이버
};

#endif // LISTENER_HPP_

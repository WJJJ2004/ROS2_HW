/**
 * @file /src/qnode.cpp
 *
 * @brief ROS communication central!
 *
 * @date August 2024
 **/

#include "../include/parameter_gui/qnode.hpp"

QNode::QNode() {
    rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp::Node>("parameter_gui_node");
}

QNode::~QNode() {
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
}

void QNode::run() {
    rclcpp::spin(node);
    Q_EMIT rosShutDown();  // ROS가 종료되면 신호를 보냅니다.
}

void QNode::setParameter(const std::string &param_name, int value) {
    node->set_parameter(rclcpp::Parameter(param_name, value));
}

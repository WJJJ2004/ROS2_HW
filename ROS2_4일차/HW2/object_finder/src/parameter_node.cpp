#include "../include/object_finder/parameter_node.hpp"
#include <mutex>
#include <queue>

ParameterImgNode::ParameterImgNode(rclcpp::Node::SharedPtr node, QObject* parent)
    : QThread(parent), node_(node)
{
    parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/image_processing_node");

    node_->declare_parameter<int>("roi_x_min", 0);
    node_->declare_parameter<int>("roi_x_max", 640);
    node_->declare_parameter<int>("roi_y_min", 0);
    node_->declare_parameter<int>("roi_y_max", 480);
    node_->declare_parameter<int>("erode_iterations", 1);
    node_->declare_parameter<int>("dilate_iterations", 1);
    node_->declare_parameter<int>("canny_threshold_min", 50);
    node_->declare_parameter<int>("canny_threshold_max", 150);
    node_->declare_parameter<int>("white_h_min", 0);
    node_->declare_parameter<int>("white_h_max", 180);
    node_->declare_parameter<int>("white_s_min", 0);
    node_->declare_parameter<int>("white_s_max", 30);
    node_->declare_parameter<int>("white_v_min", 200);
    node_->declare_parameter<int>("white_v_max", 255);
    node_->declare_parameter<int>("orange_h_min", 10);
    node_->declare_parameter<int>("orange_h_max", 25);
    node_->declare_parameter<int>("orange_s_min", 100);
    node_->declare_parameter<int>("orange_s_max", 255);
    node_->declare_parameter<int>("orange_v_min", 100);
    node_->declare_parameter<int>("orange_v_max", 255);
    node_->declare_parameter<int>("green_h_min", 35);
    node_->declare_parameter<int>("green_h_max", 85);
    node_->declare_parameter<int>("green_s_min", 50);
    node_->declare_parameter<int>("green_s_max", 255);
    node_->declare_parameter<int>("green_v_min", 50);
    node_->declare_parameter<int>("green_v_max", 255);

    RCLCPP_INFO(node_->get_logger(), "ParameterImgNode parameters have been declared.");
}

void ParameterImgNode::run()
{
    rclcpp::spin(node_);
}

void ParameterImgNode::setParameter(const std::string& name, int value)
{
    std::lock_guard<std::mutex> lock(mutex_); // 스레드 안전성 확보
    auto parameter = rclcpp::Parameter(name, value);
    auto future = parameters_client_->set_parameters({parameter});

    if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(node_->get_logger(), "Parameter %s set to %d successfully.", name.c_str(), value);
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to set parameter %s.", name.c_str());
    }
}

ParameterImgNode::~ParameterImgNode() {}

#include "../include/image_analysis/parameter_node.hpp"

parameter_node::parameter_node(rclcpp::Node::SharedPtr node, QObject* parent)
    : QThread(parent), node_(node)
{
    // 이미지 토픽 구독 설정
    image_subscription_ = node_->create_subscription<sensor_msgs::msg::Image>(
        "/camera1/camera/image_raw", 10, std::bind(&ImageNode::imageCallback, this, std::placeholders::_1));
}

void parameter_node::run()
{
    rclcpp::spin(node_);
}

void parameter_node::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try
    {
        // shared_ptr로 cv::Mat을 관리하여 메모리 유효성 보장
        auto frame_ptr = std::make_shared<cv::Mat>(cv_bridge::toCvCopy(msg, "bgr8")->image);

        // QImage가 cv::Mat 데이터를 참조하는 방식
        QImage qImage(frame_ptr->data, frame_ptr->cols, frame_ptr->rows, frame_ptr->step, QImage::Format_BGR888);

        // frame_ptr을 QImage와 함께 전달하여 수명 보장
        emit imageProcessed(qImage.copy());  // 메모리 안정성을 위해 copy 호출
        RCLCPP_INFO(node_->get_logger(), "Original image emitted");
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(node_->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

parameter_node::~parameter_node() {}


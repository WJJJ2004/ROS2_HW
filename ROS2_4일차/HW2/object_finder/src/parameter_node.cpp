#include "../include/object_finder/parameter_node.hpp"

ParameterImgNode::ParameterImgNode(rclcpp::Node::SharedPtr node, QObject* parent)
    : QThread(parent), node_(node)
{
    // AsyncParametersClient 초기화
    parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "image_processing_node");

    // 파라미터 선언
    // ROI 설정 파라미터 (4개)
    node_->declare_parameter<int>("roi_x_min", 0);
    node_->declare_parameter<int>("roi_x_max", 640);
    node_->declare_parameter<int>("roi_y_min", 0);
    node_->declare_parameter<int>("roi_y_max", 480);

    // Erode 및 Dilate 반복 횟수 (2개)
    node_->declare_parameter<int>("erode_iterations", 1);
    node_->declare_parameter<int>("dilate_iterations", 1);

    // Canny 엣지 검출 임계값 (2개)
    node_->declare_parameter<int>("canny_threshold_min", 50);
    node_->declare_parameter<int>("canny_threshold_max", 150);

    // 흰색 HSV 범위 파라미터 (6개)
    node_->declare_parameter<int>("white_h_min", 0);
    node_->declare_parameter<int>("white_h_max", 180);
    node_->declare_parameter<int>("white_s_min", 0);
    node_->declare_parameter<int>("white_s_max", 30);
    node_->declare_parameter<int>("white_v_min", 200);
    node_->declare_parameter<int>("white_v_max", 255);

    // 주황색 HSV 범위 파라미터 (6개)
    node_->declare_parameter<int>("orange_h_min", 10);
    node_->declare_parameter<int>("orange_h_max", 25);
    node_->declare_parameter<int>("orange_s_min", 100);
    node_->declare_parameter<int>("orange_s_max", 255);
    node_->declare_parameter<int>("orange_v_min", 100);
    node_->declare_parameter<int>("orange_v_max", 255);

    // 초록색 HSV 범위 파라미터 (6개)
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
    // 비동기 클라이언트 노드 실행
    rclcpp::spin(node_);
}

void ParameterImgNode::setParameter(const std::string& name, int value)
{
    // 파라미터 설정 요청
    auto parameter = rclcpp::Parameter(name, value);
    parameters_client_->set_parameters({parameter});
}

ParameterImgNode::~ParameterImgNode() {}

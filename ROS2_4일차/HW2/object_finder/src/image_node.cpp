#include "../include/object_finder/image_node.hpp"

ProcessImgNode::ProcessImgNode(rclcpp::Node::SharedPtr node, QObject* parent)
    : QThread(parent), node_(node)
{
    // 이미지 토픽 구독 설정
    image_subscription_ = node_->create_subscription<sensor_msgs::msg::Image>(
        "/camera1/camera/image_raw", 10, std::bind(&ProcessImgNode::imageCallback, this, std::placeholders::_1));

    loadParameters();  // 파라미터 로드
    RCLCPP_INFO(node_->get_logger(), "ProcessImgNode parameters have been declared and loaded.");
}

void ProcessImgNode::loadParameters()
{
    // 파라미터 로드
    node_->get_parameter("roi_x_min", roi_x_min);
    node_->get_parameter("roi_x_max", roi_x_max);
    node_->get_parameter("roi_y_min", roi_y_min);
    node_->get_parameter("roi_y_max", roi_y_max);

    node_->get_parameter("erode_iterations", erode_iterations);
    node_->get_parameter("dilate_iterations", dilate_iterations);

    node_->get_parameter("canny_threshold_min", canny_threshold_min);
    node_->get_parameter("canny_threshold_max", canny_threshold_max);

    node_->get_parameter("white_h_min", white_h_min);
    node_->get_parameter("white_h_max", white_h_max);
    node_->get_parameter("white_s_min", white_s_min);
    node_->get_parameter("white_s_max", white_s_max);
    node_->get_parameter("white_v_min", white_v_min);
    node_->get_parameter("white_v_max", white_v_max);

    node_->get_parameter("orange_h_min", orange_h_min);
    node_->get_parameter("orange_h_max", orange_h_max);
    node_->get_parameter("orange_s_min", orange_s_min);
    node_->get_parameter("orange_s_max", orange_s_max);
    node_->get_parameter("orange_v_min", orange_v_min);
    node_->get_parameter("orange_v_max", orange_v_max);

    node_->get_parameter("green_h_min", green_h_min);
    node_->get_parameter("green_h_max", green_h_max);
    node_->get_parameter("green_s_min", green_s_min);
    node_->get_parameter("green_s_max", green_s_max);
    node_->get_parameter("green_v_min", green_v_min);
    node_->get_parameter("green_v_max", green_v_max);
}

void ProcessImgNode::run()
{
    rclcpp::spin(node_);
}

void ProcessImgNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try
    {
        // 원본 이미지를 cv::Mat으로 변환
        auto frame_ptr = std::make_shared<cv::Mat>(cv_bridge::toCvCopy(msg, "bgr8")->image);

        // 1. 원본 이미지 표시 (USB_CAM)
        QImage qImage_cam(frame_ptr->data, frame_ptr->cols, frame_ptr->rows, frame_ptr->step, QImage::Format_BGR888);
        emit updateLabel("USB_CAM", qImage_cam.copy());

        // 2. HSV 변환
        cv::Mat hsv_image;
        cv::cvtColor(*frame_ptr, hsv_image, cv::COLOR_BGR2HSV);

        // 3. 흰색 필터링 및 이진화
        cv::Mat binary_white;
        cv::inRange(hsv_image, cv::Scalar(white_h_min, white_s_min, white_v_min),
                    cv::Scalar(white_h_max, white_s_max, white_v_max), binary_white);
        cv::erode(binary_white, binary_white, cv::Mat(), cv::Point(-1, -1), erode_iterations);
        cv::dilate(binary_white, binary_white, cv::Mat(), cv::Point(-1, -1), dilate_iterations);
        QImage qImage_white(binary_white.data, binary_white.cols, binary_white.rows, binary_white.step, QImage::Format_Grayscale8);
        emit updateLabel("Binary_white", qImage_white.copy());

        // 4. 오렌지색 필터링 및 이진화
        cv::Mat binary_orange;
        cv::inRange(hsv_image, cv::Scalar(orange_h_min, orange_s_min, orange_v_min),
                    cv::Scalar(orange_h_max, orange_s_max, orange_v_max), binary_orange);
        cv::erode(binary_orange, binary_orange, cv::Mat(), cv::Point(-1, -1), erode_iterations);
        cv::dilate(binary_orange, binary_orange, cv::Mat(), cv::Point(-1, -1), dilate_iterations);
        QImage qImage_orange(binary_orange.data, binary_orange.cols, binary_orange.rows, binary_orange.step, QImage::Format_Grayscale8);
        emit updateLabel("Binary_orange", qImage_orange.copy());

        // 5. 초록색 필터링 및 이진화
        cv::Mat binary_lime;
        cv::inRange(hsv_image, cv::Scalar(green_h_min, green_s_min, green_v_min),
                    cv::Scalar(green_h_max, green_s_max, green_v_max), binary_lime);
        cv::erode(binary_lime, binary_lime, cv::Mat(), cv::Point(-1, -1), erode_iterations);
        cv::dilate(binary_lime, binary_lime, cv::Mat(), cv::Point(-1, -1), dilate_iterations);
        QImage qImage_lime(binary_lime.data, binary_lime.cols, binary_lime.rows, binary_lime.step, QImage::Format_Grayscale8);
        emit updateLabel("Binary_Lime", qImage_lime.copy());

        // 6. Canny 엣지 검출
        cv::Mat edges;
        cv::Canny(binary_white, edges, canny_threshold_min, canny_threshold_max);
        QImage qImage_edges(edges.data, edges.cols, edges.rows, edges.step, QImage::Format_Grayscale8);
        emit updateLabel("Find_object", qImage_edges.copy());

        // 7. Hough 선 검출
        std::vector<cv::Vec4i> lines;
        cv::Mat hough_image;
        cv::cvtColor(edges, hough_image, cv::COLOR_GRAY2BGR);
        cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 50, 50, 10);
        for (const auto& line : lines) {
            cv::line(hough_image, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0, 255, 0), 2);
        }
        QImage qImage_hough(hough_image.data, hough_image.cols, hough_image.rows, hough_image.step, QImage::Format_BGR888);
        emit updateLabel("Hough_Lines", qImage_hough.copy());

        RCLCPP_INFO(node_->get_logger(), "Image labels updated.");
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(node_->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}



cv::Mat ProcessImgNode::performObjectDetection(const cv::Mat &input_image)
{
    return input_image.clone();
}

cv::Mat ProcessImgNode::applyHSVFilter(const cv::Mat &input_image, int h_min, int h_max, int s_min, int s_max, int v_min, int v_max)
{
    cv::Mat hsv_image, binary_image;

    cv::cvtColor(input_image, hsv_image, cv::COLOR_BGR2HSV);

    cv::inRange(hsv_image, cv::Scalar(h_min, s_min, v_min), cv::Scalar(h_max, s_max, v_max), binary_image);

    return binary_image;
}

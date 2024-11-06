#include "../include/object_finder/image_node.hpp"

ProcessImgNode::ProcessImgNode(rclcpp::Node::SharedPtr node, QObject* parent)
    : QThread(parent), node_(node)
{
    // 이미지 토픽 구독 설정
    image_subscription_ = node_->create_subscription<sensor_msgs::msg::Image>(
        "/camera1/camera/image_raw", 10, std::bind(&ProcessImgNode::imageCallback, this, std::placeholders::_1));

    // 파라미터 클라이언트를 초기화
    parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/image_processing_node");

    int service_retry_count = 3;
    while (!parameters_client_->wait_for_service(std::chrono::seconds(5)) && service_retry_count > 0)
    {
        RCLCPP_WARN(node_->get_logger(), "Waiting for parameter client service... %d retries left", service_retry_count);
        --service_retry_count;
    }

    if (service_retry_count == 0)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to connect to 'parameter_client_node' service after multiple attempts.");
    }
    else
    {
        RCLCPP_INFO(node_->get_logger(), "Connected to 'parameter_client_node' for parameter loading.");
    }
}

void ProcessImgNode::loadParameters()
{
    if (!parameters_client_) {
        RCLCPP_ERROR(node_->get_logger(), "Parameters client is not initialized.");
        return;
    }

    // 파라미터 이름 목록 설정
    std::vector<std::string> param_names = {
        "roi_x_min", "roi_x_max", "roi_y_min", "roi_y_max",
        "erode_iterations", "dilate_iterations",
        "canny_threshold_min", "canny_threshold_max",
        "white_h_min", "white_h_max", "white_s_min", "white_s_max", "white_v_min", "white_v_max",
        "orange_h_min", "orange_h_max", "orange_s_min", "orange_s_max", "orange_v_min", "orange_v_max",
        "green_h_min", "green_h_max", "green_s_min", "green_s_max", "green_v_min", "green_v_max"
    };

    // 파라미터 가져오기 비동기 호출
    auto future = parameters_client_->get_parameters(param_names);

    // 타임아웃 설정 (3초 대기)
    if (future.wait_for(std::chrono::seconds(3)) == std::future_status::timeout) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load parameters: Timed out after waiting for 3 seconds. Using default values.");

        // 기본값을 할당
        roi_x_min = 0;
        roi_x_max = 640;
        roi_y_min = 0;
        roi_y_max = 480;
        erode_iterations = 1;
        dilate_iterations = 1;
        canny_threshold_min = 50;
        canny_threshold_max = 150;
        white_h_min = 0;
        white_h_max = 180;
        white_s_min = 0;
        white_s_max = 30;
        white_v_min = 200;
        white_v_max = 255;
        orange_h_min = 10;
        orange_h_max = 25;
        orange_s_min = 100;
        orange_s_max = 255;
        orange_v_min = 100;
        orange_v_max = 255;
        green_h_min = 35;
        green_h_max = 85;
        green_s_min = 50;
        green_s_max = 255;
        green_v_min = 50;
        green_v_max = 255;

        // 기본값 사용을 로그에 기록
        RCLCPP_WARN(node_->get_logger(), "Using default values for parameters due to loading failure.");
        return;
    }

    try {
        auto results = future.get();
        for (const auto& param : results) {
            if (param.get_name() == "roi_x_min") roi_x_min = param.as_int();
            else if (param.get_name() == "roi_x_max") roi_x_max = param.as_int();
            else if (param.get_name() == "roi_y_min") roi_y_min = param.as_int();
            else if (param.get_name() == "roi_y_max") roi_y_max = param.as_int();
            else if (param.get_name() == "erode_iterations") erode_iterations = param.as_int();
            else if (param.get_name() == "dilate_iterations") dilate_iterations = param.as_int();
            else if (param.get_name() == "canny_threshold_min") canny_threshold_min = param.as_int();
            else if (param.get_name() == "canny_threshold_max") canny_threshold_max = param.as_int();
            else if (param.get_name() == "white_h_min") white_h_min = param.as_int();
            else if (param.get_name() == "white_h_max") white_h_max = param.as_int();
            else if (param.get_name() == "white_s_min") white_s_min = param.as_int();
            else if (param.get_name() == "white_s_max") white_s_max = param.as_int();
            else if (param.get_name() == "white_v_min") white_v_min = param.as_int();
            else if (param.get_name() == "white_v_max") white_v_max = param.as_int();
            else if (param.get_name() == "orange_h_min") orange_h_min = param.as_int();
            else if (param.get_name() == "orange_h_max") orange_h_max = param.as_int();
            else if (param.get_name() == "orange_s_min") orange_s_min = param.as_int();
            else if (param.get_name() == "orange_s_max") orange_s_max = param.as_int();
            else if (param.get_name() == "orange_v_min") orange_v_min = param.as_int();
            else if (param.get_name() == "orange_v_max") orange_v_max = param.as_int();
            else if (param.get_name() == "green_h_min") green_h_min = param.as_int();
            else if (param.get_name() == "green_h_max") green_h_max = param.as_int();
            else if (param.get_name() == "green_s_min") green_s_min = param.as_int();
            else if (param.get_name() == "green_s_max") green_s_max = param.as_int();
            else if (param.get_name() == "green_v_min") green_v_min = param.as_int();
            else if (param.get_name() == "green_v_max") green_v_max = param.as_int();
        }

        // 로드된 파라미터 값 출력
        RCLCPP_INFO(node_->get_logger(), "Loaded parameters successfully.");
        RCLCPP_INFO(node_->get_logger(), "roi_x_min=%d, roi_x_max=%d, roi_y_min=%d, roi_y_max=%d", roi_x_min, roi_x_max, roi_y_min, roi_y_max);
        RCLCPP_INFO(node_->get_logger(), "erode_iterations=%d, dilate_iterations=%d", erode_iterations, dilate_iterations);
        RCLCPP_INFO(node_->get_logger(), "canny_threshold_min=%d, canny_threshold_max=%d", canny_threshold_min, canny_threshold_max);
        RCLCPP_INFO(node_->get_logger(), "white_h_min=%d, white_h_max=%d, white_s_min=%d, white_s_max=%d, white_v_min=%d, white_v_max=%d",
                    white_h_min, white_h_max, white_s_min, white_s_max, white_v_min, white_v_max);
        RCLCPP_INFO(node_->get_logger(), "orange_h_min=%d, orange_h_max=%d, orange_s_min=%d, orange_s_max=%d, orange_v_min=%d, orange_v_max=%d",
                    orange_h_min, orange_h_max, orange_s_min, orange_s_max, orange_v_min, orange_v_max);
        RCLCPP_INFO(node_->get_logger(), "green_h_min=%d, green_h_max=%d, green_s_min=%d, green_s_max=%d, green_v_min=%d, green_v_max=%d",
                    green_h_min, green_h_max, green_s_min, green_s_max, green_v_min, green_v_max);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load parameters: %s", e.what());
    }
}

void ProcessImgNode::run()
{
    rclcpp::spin(node_);
}

void ProcessImgNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    std::cout << "detect raw image" << std::endl;
    loadParameters();  // 파라미터 로드
    RCLCPP_INFO(node_->get_logger(), "ProcessImgNode parameters have been declared and loaded.");

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

        // cv::Mat rgb_image;
        // cv::cvtColor(hsv_image, rgb_image, cv::COLOR_HSV2BGR);

        // // 변환한 RGB 이미지를 QImage로 변환하여 UI에 표시
        // QImage qImage_hsv(rgb_image.data, rgb_image.cols, rgb_image.rows, rgb_image.step, QImage::Format_BGR888);
        // emit updateLabel("Hough_Lines", qImage_hsv.copy());



        //teset code
        // QImage qImage_hsv(hsv_image.data, hsv_image.cols, hsv_image.rows, hsv_image.step, QImage::Format_BGR888);
        // emit updateLabel("Hough_Lines", qImage_hsv.copy());

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

        RCLCPP_INFO(node_->get_logger(), "White HSV Min: H=%d, S=%d, V=%d", white_h_min, white_s_min, white_v_min);
        RCLCPP_INFO(node_->get_logger(), "White HSV Max: H=%d, S=%d, V=%d", white_h_max, white_s_max, white_v_max);

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

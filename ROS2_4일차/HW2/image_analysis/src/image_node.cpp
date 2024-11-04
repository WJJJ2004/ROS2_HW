#include "../include/image_analysis/image_node.hpp"
#include <QMetaObject>

ImageNode::ImageNode() {
    int argc = 0;
    char** argv = nullptr;

    // 노드 초기화 및 생성
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("image_analysis");

    // 파라미터 선언 최적화
    std::vector<std::pair<std::string, int>> int_params = {
        {"roi_x", 50}, {"roi_y", 50}, {"roi_width", 200}, {"roi_height", 200},
        {"erode_iterations", 2}, {"dilate_iterations", 2},
        {"white_lower_h", 0}, {"white_lower_s", 0}, {"white_lower_v", 200},
        {"white_upper_h", 180}, {"white_upper_s", 20}, {"white_upper_v", 255},
        {"orange_lower_h", 5}, {"orange_lower_s", 100}, {"orange_lower_v", 100},
        {"orange_upper_h", 15}, {"orange_upper_s", 255}, {"orange_upper_v", 255},
        {"green_lower_h", 35}, {"green_lower_s", 100}, {"green_lower_v", 100},
        {"green_upper_h", 85}, {"green_upper_s", 255}, {"green_upper_v", 255}
    };

    for (const auto& param : int_params) {
        node->declare_parameter(param.first, param.second);
    }

    std::vector<std::pair<std::string, double>> double_params = {
        {"canny_low_thresh", 50.0}, {"canny_high_thresh", 150.0}
    };

    for (const auto& param : double_params) {
        node->declare_parameter(param.first, param.second);
    }

    // 파라미터 변경 콜백 등록
    node->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &parameters) {
        updateParameters();
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    });

    // 이미지 토픽 구독
    image_subscriber_ = node->create_subscription<sensor_msgs::msg::Image>(
        "/camera1/camera/image_raw", 10, std::bind(&ImageNode::imageCallback, this, std::placeholders::_1));

    updateParameters();  // 초기 파라미터 설정
    this->start();
}

ImageNode::~ImageNode() {
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
}

void ImageNode::run() {
    rclcpp::WallRate loop_rate(20);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    Q_EMIT rosShutDown();
}

void ImageNode::updateParameters() {
    // ROI 파라미터 업데이트
    roi_x_ = node->get_parameter("roi_x").as_int();
    roi_y_ = node->get_parameter("roi_y").as_int();
    roi_width_ = node->get_parameter("roi_width").as_int();
    roi_height_ = node->get_parameter("roi_height").as_int();

    // Erode 및 Dilate 반복 횟수 업데이트
    erode_iterations_ = node->get_parameter("erode_iterations").as_int();
    dilate_iterations_ = node->get_parameter("dilate_iterations").as_int();

    // Canny 임계값 업데이트
    canny_low_thresh_ = node->get_parameter("canny_low_thresh").as_double();
    canny_high_thresh_ = node->get_parameter("canny_high_thresh").as_double();

    // 흰색 HSV 범위 업데이트
    white_lower_ = cv::Scalar(
        node->get_parameter("white_lower_h").as_int(),
        node->get_parameter("white_lower_s").as_int(),
        node->get_parameter("white_lower_v").as_int()
        );
    white_upper_ = cv::Scalar(
        node->get_parameter("white_upper_h").as_int(),
        node->get_parameter("white_upper_s").as_int(),
        node->get_parameter("white_upper_v").as_int()
        );

    // 오렌지색 HSV 범위 업데이트
    orange_lower_ = cv::Scalar(
        node->get_parameter("orange_lower_h").as_int(),
        node->get_parameter("orange_lower_s").as_int(),
        node->get_parameter("orange_lower_v").as_int()
        );
    orange_upper_ = cv::Scalar(
        node->get_parameter("orange_upper_h").as_int(),
        node->get_parameter("orange_upper_s").as_int(),
        node->get_parameter("orange_upper_v").as_int()
        );

    // 초록색 HSV 범위 업데이트
    green_lower_ = cv::Scalar(
        node->get_parameter("green_lower_h").as_int(),
        node->get_parameter("green_lower_s").as_int(),
        node->get_parameter("green_lower_v").as_int()
        );
    green_upper_ = cv::Scalar(
        node->get_parameter("green_upper_h").as_int(),
        node->get_parameter("green_upper_s").as_int(),
        node->get_parameter("green_upper_v").as_int()
        );
}

void ImageNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        RCLCPP_DEBUG(node->get_logger(), "Starting image callback");

        // 원본 이미지 수신
        cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;
        if (image.empty()) {
            RCLCPP_ERROR(node->get_logger(), "Received an empty image!");
            return;
        }

        // 1. ROI 설정
        cv::Mat roi_image = image(cv::Rect(roi_x_, roi_y_, roi_width_, roi_height_));
        QImage qimage_roi(roi_image.data, roi_image.cols, roi_image.rows, roi_image.step, QImage::Format_RGB888);
        Q_EMIT imageReceived(qimage_roi.copy(), "roi");

        // 2. 가우시안 필터 및 HSV 변환
        cv::Mat blurred;
        cv::GaussianBlur(roi_image, blurred, cv::Size(5, 5), 0);
        cv::Mat hsv_image;
        cv::cvtColor(blurred, hsv_image, cv::COLOR_BGR2HSV);

        // 3. 색상 검출 및 이진화 (흰색, 라임색, 오렌지색)
        cv::Mat white_mask, lime_mask, orange_mask;
        cv::inRange(hsv_image, white_lower_, white_upper_, white_mask);
        cv::inRange(hsv_image, green_lower_, green_upper_, lime_mask);
        cv::inRange(hsv_image, orange_lower_, orange_upper_, orange_mask);

        // 흰색 검출 결과 출력
        QImage qimage_bw(white_mask.data, white_mask.cols, white_mask.rows, white_mask.step, QImage::Format_Grayscale8);
        Q_EMIT imageReceived(qimage_bw.copy(), "bw");

        // 라임색 검출 결과 출력
        QImage qimage_bl(lime_mask.data, lime_mask.cols, lime_mask.rows, lime_mask.step, QImage::Format_Grayscale8);
        Q_EMIT imageReceived(qimage_bl.copy(), "bl");

        // 오렌지색 검출 결과 출력
        QImage qimage_bo(orange_mask.data, orange_mask.cols, orange_mask.rows, orange_mask.step, QImage::Format_Grayscale8);
        Q_EMIT imageReceived(qimage_bo.copy(), "bo");

        // 4. Erode 및 Dilate 연산 (흰색 이진화 결과에 적용)
        cv::Mat white_processed = white_mask.clone();
        cv::erode(white_processed, white_processed, cv::Mat(), cv::Point(-1, -1), erode_iterations_);
        cv::dilate(white_processed, white_processed, cv::Mat(), cv::Point(-1, -1), dilate_iterations_);

        // 5. Canny 엣지 검출
        cv::Mat edges;
        cv::Canny(white_processed, edges, canny_low_thresh_, canny_high_thresh_);
        QImage qimage_edges(edges.data, edges.cols, edges.rows, edges.step, QImage::Format_Grayscale8);
        Q_EMIT imageReceived(qimage_edges.copy(), "find_object");

        // 6. Hough 변환을 통한 선 검출
        std::vector<cv::Vec4i> lines;
        cv::Mat hough_image = edges.clone();
        cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 50, 50, 10);
        cv::cvtColor(hough_image, hough_image, cv::COLOR_GRAY2BGR);
        for (const auto& line : lines) {
            cv::line(hough_image, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0, 255, 0), 2);
        }
        QImage qimage_hough(hough_image.data, hough_image.cols, hough_image.rows, hough_image.step, QImage::Format_RGB888);
        Q_EMIT imageReceived(qimage_hough.copy(), "hough_lines");

        RCLCPP_DEBUG(node->get_logger(), "Image callback processing completed successfully");

    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(node->get_logger(), "cv_bridge exception: %s", e.what());
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "Standard exception: %s", e.what());
    } catch (...) {
        RCLCPP_ERROR(node->get_logger(), "Unknown error occurred during image processing.");
    }
}

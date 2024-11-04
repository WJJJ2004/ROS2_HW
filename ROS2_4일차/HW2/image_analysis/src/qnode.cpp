#include "../include/image_analysis/qnode.hpp"
#include <QMetaObject>

QNode::QNode()
{
    int argc = 0;
    char** argv = nullptr;
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("image_analysis");

    // 이미지 토픽 구독
    image_subscriber_ = node->create_subscription<sensor_msgs::msg::Image>(
        "/camera1/camera/image_raw", 10, std::bind(&QNode::imageCallback, this, std::placeholders::_1));

    this->start();
}

QNode::~QNode()
{
    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }
}

void QNode::run()
{
    rclcpp::WallRate loop_rate(20);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    Q_EMIT rosShutDown();
}

void QNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try
    {
        RCLCPP_DEBUG(node->get_logger(), "Starting image callback");

        // 1. 원본 이미지 수신 및 전송
        cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;
        if (image.empty()) {
            RCLCPP_ERROR(node->get_logger(), "Received an empty image!");
            return;
        }
        QImage qimage_original(image.data, image.cols, image.rows, image.step, QImage::Format_RGB888);
        Q_EMIT imageReceived(qimage_original.copy(), "usb_cam");  // copy() 사용
        RCLCPP_INFO(node->get_logger(), "Original image sent to usb_cam label");

        // 2. 객체 탐지 (에지 검출)
        cv::Mat edges;
        cv::Canny(image, edges, 50, 150);
        if (edges.empty()) {
            RCLCPP_WARN(node->get_logger(), "Edge detection failed or produced an empty image.");
        } else {
            QImage qimage_edges(edges.data, edges.cols, edges.rows, edges.step, QImage::Format_Grayscale8);
            Q_EMIT imageReceived(qimage_edges.copy(), "find_object");  // copy() 사용
            RCLCPP_INFO(node->get_logger(), "Edge-detected image sent to find_object label");
        }

        // 3. ROI 이미지
        cv::Rect roi_rect(50, 50, 200, 200);
        if (roi_rect.x + roi_rect.width > image.cols || roi_rect.y + roi_rect.height > image.rows) {
            RCLCPP_ERROR(node->get_logger(), "ROI exceeds image boundaries.");
            return;
        }
        cv::Mat roi_image = image(roi_rect);
        if (roi_image.empty()) {
            RCLCPP_WARN(node->get_logger(), "ROI extraction failed or produced an empty image.");
        } else {
            QImage qimage_roi(roi_image.data, roi_image.cols, roi_image.rows, roi_image.step, QImage::Format_RGB888);
            Q_EMIT imageReceived(qimage_roi.copy(), "roi");  // copy() 사용
            RCLCPP_INFO(node->get_logger(), "ROI image sent to roi label");
        }

        // 4. 흰색 이진화
        cv::Mat gray_image, binary_white;
        cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
        cv::threshold(gray_image, binary_white, 200, 255, cv::THRESH_BINARY);
        if (binary_white.empty()) {
            RCLCPP_WARN(node->get_logger(), "Binary white thresholding failed or produced an empty image.");
        } else {
            QImage qimage_bw(binary_white.data, binary_white.cols, binary_white.rows, binary_white.step, QImage::Format_Grayscale8);
            Q_EMIT imageReceived(qimage_bw.copy(), "bw");  // copy() 사용
            RCLCPP_INFO(node->get_logger(), "Binary white image sent to bw label");
        }

        // 5. 라임색 이진화
        cv::Mat binary_lime = (gray_image > 100) & (gray_image < 150);
        if (binary_lime.empty()) {
            RCLCPP_WARN(node->get_logger(), "Binary lime thresholding failed or produced an empty image.");
        } else {
            QImage qimage_bl(binary_lime.data, binary_lime.cols, binary_lime.rows, binary_lime.step, QImage::Format_Grayscale8);
            Q_EMIT imageReceived(qimage_bl.copy(), "bl");  // copy() 사용
            RCLCPP_INFO(node->get_logger(), "Binary lime image sent to bl label");
        }

        // 6. 오렌지색 이진화
        cv::Mat binary_orange = (gray_image > 50) & (gray_image < 100);
        if (binary_orange.empty()) {
            RCLCPP_WARN(node->get_logger(), "Binary orange thresholding failed or produced an empty image.");
        } else {
            QImage qimage_bo(binary_orange.data, binary_orange.cols, binary_orange.rows, binary_orange.step, QImage::Format_Grayscale8);
            Q_EMIT imageReceived(qimage_bo.copy(), "bo");  // copy() 사용
            RCLCPP_INFO(node->get_logger(), "Binary orange image sent to bo label");
        }

        RCLCPP_DEBUG(node->get_logger(), "Image callback processing completed successfully");

    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(node->get_logger(), "cv_bridge exception: %s", e.what());
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "Standard exception: %s", e.what());
    }
    catch (...)
    {
        RCLCPP_ERROR(node->get_logger(), "Unknown error occurred during image processing.");
    }
}

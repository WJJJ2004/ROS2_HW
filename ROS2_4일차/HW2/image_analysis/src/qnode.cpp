#include "../include/image_analysis/qnode.hpp"
#include <QTimer>
#include <QDebug>

QNode::QNode()
{
    int argc = 0;
    char** argv = nullptr;

    qDebug() << "Initializing ROS2...";
    rclcpp::init(argc, argv);  // ROS2 초기화
    node = rclcpp::Node::make_shared("image_analysis");

    qDebug() << "Setting up image subscription...";
    // 이미지 토픽 구독
    image_subscriber_ = node->create_subscription<sensor_msgs::msg::Image>(
        "/camera1/camera/image_raw", 10, std::bind(&QNode::imageCallback, this, std::placeholders::_1));

    qDebug() << "Starting QNode thread...";
    this->start();  // QNode 스레드 시작
}

QNode::~QNode()
{
    if (rclcpp::ok())
    {
        qDebug() << "Shutting down ROS2...";
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
    qDebug() << "Exiting QNode thread...";
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

        QImage qimage_original(image.clone().data, image.cols, image.rows, image.step, QImage::Format_RGB888);
        QTimer::singleShot(0, [this, qimage_original]() {
            Q_EMIT imageReceived(qimage_original.rgbSwapped(), "usb_cam");
            RCLCPP_INFO(node->get_logger(), "Original image sent to usb_cam label");
        });

        // 2. 객체 탐지 (에지 검출)
        cv::Mat edges;
        cv::Canny(image, edges, 50, 150);
        if (!edges.empty()) {
            QImage qimage_edges(edges.clone().data, edges.cols, edges.rows, edges.step, QImage::Format_Grayscale8);
            QTimer::singleShot(0, [this, qimage_edges]() {
                Q_EMIT imageReceived(qimage_edges, "find_object");
                RCLCPP_INFO(node->get_logger(), "Edge-detected image sent to find_object label");
            });
        }

        // 3. 중앙부 사다리꼴 ROI 설정
        int width = image.cols;
        int height = image.rows;
        std::vector<cv::Point> trapezoid_points = {
            cv::Point(width * 0.3, height * 0.6),  // 좌측 상단
            cv::Point(width * 0.7, height * 0.6),  // 우측 상단
            cv::Point(width * 0.9, height * 0.9),  // 우측 하단
            cv::Point(width * 0.1, height * 0.9)   // 좌측 하단
        };

        cv::Mat mask = cv::Mat::zeros(height, width, CV_8UC1);
        cv::fillConvexPoly(mask, trapezoid_points, cv::Scalar(255));

        cv::Mat trapezoid_roi;
        image.copyTo(trapezoid_roi, mask);
        if (!trapezoid_roi.empty()) {
            QImage qimage_roi(trapezoid_roi.clone().data, trapezoid_roi.cols, trapezoid_roi.rows, trapezoid_roi.step, QImage::Format_RGB888);
            QTimer::singleShot(0, [this, qimage_roi]() {
                Q_EMIT imageReceived(qimage_roi.rgbSwapped(), "roi");
                RCLCPP_INFO(node->get_logger(), "Trapezoidal ROI image sent to roi label");
            });
        }

        // 4. 흰색 이진화
        cv::Mat gray_image, binary_white;
        cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
        cv::threshold(gray_image, binary_white, 200, 255, cv::THRESH_BINARY);
        if (!binary_white.empty()) {
            QImage qimage_bw(binary_white.clone().data, binary_white.cols, binary_white.rows, binary_white.step, QImage::Format_Grayscale8);
            QTimer::singleShot(0, [this, qimage_bw]() {
                Q_EMIT imageReceived(qimage_bw, "bw");
                RCLCPP_INFO(node->get_logger(), "Binary white image sent to bw label");
            });
        }

        // 5. 라임색 이진화
        cv::Mat binary_lime = (gray_image > 100) & (gray_image < 150);
        if (!binary_lime.empty()) {
            QImage qimage_bl(binary_lime.clone().data, binary_lime.cols, binary_lime.rows, binary_lime.step, QImage::Format_Grayscale8);
            QTimer::singleShot(0, [this, qimage_bl]() {
                Q_EMIT imageReceived(qimage_bl, "bl");
                RCLCPP_INFO(node->get_logger(), "Binary lime image sent to bl label");
            });
        }

        // 6. 오렌지색 이진화
        cv::Mat binary_orange = (gray_image > 50) & (gray_image < 100);
        if (!binary_orange.empty()) {
            QImage qimage_bo(binary_orange.clone().data, binary_orange.cols, binary_orange.rows, binary_orange.step, QImage::Format_Grayscale8);
            QTimer::singleShot(0, [this, qimage_bo]() {
                Q_EMIT imageReceived(qimage_bo, "bo");
                RCLCPP_INFO(node->get_logger(), "Binary orange image sent to bo label");
            });
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

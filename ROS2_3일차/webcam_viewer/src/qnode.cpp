/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date August 2024
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/webcam_viewer/qnode.hpp"

QNode::QNode()
{
  int argc = 0;
  char** argv = NULL;
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("webcam_viewer");

  // /camera1/camera/image_raw 토픽 구독 설정
  image_subscription = node->create_subscription<sensor_msgs::msg::Image>(
      "/camera1/camera/image_raw", 10,
      std::bind(&QNode::imageCallback, this, std::placeholders::_1)
      );

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
    rclcpp::spin(node);
    Q_EMIT rosShutDown();
}

void QNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try
    {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

        // 데이터를 복사하여 QImage로 변환
        QImage qimage = QImage(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_BGR888).copy();
        Q_EMIT imageReceived(qimage);  // QImage로 변환된 이미지를 시그널로 전송
    }
    catch (const cv_bridge::Exception& e)
    {
        std::cerr << "cv_bridge exception: " << e.what() << std::endl;  // 콘솔에 예외 메시지 출력
    }
}

#ifndef IMAGE_ANALYSIS_IMAGE_NODE_HPP_
#define IMAGE_ANALYSIS_IMAGE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <QThread>
#include <QImage>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class ImageNode : public QThread {
    Q_OBJECT

public:
    ImageNode();
    ~ImageNode();

protected:
    void run() override;

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void updateParameters();  // ROS2 파라미터 업데이트 함수

    std::shared_ptr<rclcpp::Node> node;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;

    int roi_x_, roi_y_, roi_width_, roi_height_;
    int erode_iterations_, dilate_iterations_;
    double canny_low_thresh_, canny_high_thresh_;
    cv::Scalar white_lower_, white_upper_;
    cv::Scalar orange_lower_, orange_upper_;
    cv::Scalar green_lower_, green_upper_;

Q_SIGNALS:
    void rosShutDown();
    void imageReceived(const QImage &image, const QString &label_name);
};

#endif  // IMAGE_ANALYSIS_IMAGE_NODE_HPP_

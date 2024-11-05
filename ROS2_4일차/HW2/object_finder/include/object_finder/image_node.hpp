#ifndef CAM_PARAMS_PKG_PROCESS_IMG_NODE_H
#define CAM_PARAMS_PKG_PROCESS_IMG_NODE_H

#include <QThread>
#include <QImage>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ProcessImgNode : public QThread
{
    Q_OBJECT

public:
    explicit ProcessImgNode(rclcpp::Node::SharedPtr node, QObject* parent = nullptr);

protected:
    void run() override;

signals:
    void imageProcessed(const QImage& image);  // 이미지 처리 완료 시그널
    void updateLabel(const QString &labelName, const QImage &image);  // UI 업데이트 시그널 추가

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;

    // 이미지 콜백 함수
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    // 파라미터 변수 추가
    int roi_x_min, roi_x_max, roi_y_min, roi_y_max;
    int erode_iterations, dilate_iterations;
    int canny_threshold_min, canny_threshold_max;
    int white_h_min, white_h_max, white_s_min, white_s_max, white_v_min, white_v_max;
    int orange_h_min, orange_h_max, orange_s_min, orange_s_max, orange_v_min, orange_v_max;
    int green_h_min, green_h_max, green_s_min, green_s_max, green_v_min, green_v_max;

    // 파라미터 로드 함수
    void loadParameters();

    // 이미지 처리 함수
    cv::Mat performObjectDetection(const cv::Mat &input_image);
    cv::Mat applyHSVFilter(const cv::Mat &input_image, int h_min, int h_max, int s_min, int s_max, int v_min, int v_max);
};

#endif  // CAM_PARAMS_PKG_PROCESS_IMG_NODE_H

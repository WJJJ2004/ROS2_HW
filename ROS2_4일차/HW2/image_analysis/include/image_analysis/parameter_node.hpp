#ifndef IMAGE_ANALYSIS_PARAMETER_NODE_H
#define IMAGE_ANALYSIS_PARAMETER_NODE_H

#include <QThread>
#include <QImage>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class parameter_node : public QThread
{
    Q_OBJECT

public:
    explicit parameter_node(rclcpp::Node::SharedPtr node, QObject* parent = nullptr);
    ~parameter_node();

protected:
    void run() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

signals:
    void imageProcessed(const QImage& image);
};

#endif  // CAM_PARAMS_PKG_PROCESS_IMG_NODE_H

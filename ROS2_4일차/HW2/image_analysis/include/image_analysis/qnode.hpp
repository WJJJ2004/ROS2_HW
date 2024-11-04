#ifndef IMAGE_ANALYSIS_QNODE_HPP_
#define IMAGE_ANALYSIS_QNODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <QThread>
#include <QImage>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class QNode : public QThread {
    Q_OBJECT

public:
    QNode();
    ~QNode();

protected:
    void run();

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    std::shared_ptr<rclcpp::Node> node;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;

Q_SIGNALS:
    void rosShutDown();
    void imageReceived(const QImage &image, const QString &label_name);
};

#endif /* IMAGE_ANALYSIS_QNODE_HPP_ */

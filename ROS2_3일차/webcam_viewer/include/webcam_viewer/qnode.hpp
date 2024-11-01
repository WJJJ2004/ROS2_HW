/**
 * @file /include/webcam_viewer/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef webcam_viewer_QNODE_HPP_
#define webcam_viewer_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <QImage>

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#endif
#include <QThread>

/*****************************************************************************
** Class
*****************************************************************************/
class QNode : public QThread
{
  Q_OBJECT
public:
  QNode();
  ~QNode();

protected:
  void run();

private:
  std::shared_ptr<rclcpp::Node> node;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription;

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

Q_SIGNALS:
  void rosShutDown();
  void imageReceived(const QImage &image);  // 새로운 시그널 추가
};

#endif /* webcam_viewer_QNODE_HPP_ */

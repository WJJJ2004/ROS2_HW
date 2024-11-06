#ifndef OBJECT_FINDER_MAIN_WINDOW_NODE_H
#define OBJECT_FINDER_MAIN_WINDOW_NODE_H

#include <QThread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>

class ParameterImgNode : public QThread
{
  Q_OBJECT

public:
  explicit ParameterImgNode(rclcpp::Node::SharedPtr node, QObject* parent = nullptr);
  ~ParameterImgNode();

  void setParameter(const std::string& name, int value);

protected:
  void run() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::AsyncParametersClient> parameters_client_;
  std::mutex mutex_;
};

#endif  // CAM_PARAMS_PKG_PARAMETER_IMG_NODE_H

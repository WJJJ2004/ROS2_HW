/**
 * @file /include/parameter_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date August 2024
 **/

#ifndef PARAMETER_GUI_QNODE_HPP_
#define PARAMETER_GUI_QNODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <QThread>
#include <QString>

/**
 * @brief QNode class for handling ROS2 node operations.
 */
class QNode : public QThread {
    Q_OBJECT

public:
    QNode();
    ~QNode();

    // ROS2 파라미터 설정 함수
    void setParameter(const std::string &param_name, int value);

protected:
    void run() override;

private:
    std::shared_ptr<rclcpp::Node> node;

Q_SIGNALS:
    void rosShutDown();
};

#endif /* PARAMETER_GUI_QNODE_HPP_ */

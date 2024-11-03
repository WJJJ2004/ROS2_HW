/**
 * @file /include/my_turtle_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date August 2024
 **/
#ifndef MY_TURTLE_GUI_QNODE_HPP_
#define MY_TURTLE_GUI_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/srv/set_pen.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>
#include <std_srvs/srv/empty.hpp>
#endif
#include <QThread>
#include <memory>

/*****************************************************************************
** Class
*****************************************************************************/
class QNode : public QThread
{
    Q_OBJECT
public:
    QNode();
    ~QNode();

    void publishTwist(float linear, float angular);
    void drawCircle();
    void drawSquare();
    void drawTriangle();
    void setPen();
    void setBackground();
    void setTurtleType();

protected:
    void run() override;

Q_SIGNALS:
    void rosShutDown();
    void messageToUI(const QString &message);

private:
    std::shared_ptr<rclcpp::Node> node;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher;
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr set_pen_client;
    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr teleport_client;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_client;
    std::shared_ptr<rclcpp::AsyncParametersClient> parameter_client;
};

#endif /* MY_TURTLE_GUI_QNODE_HPP_ */
